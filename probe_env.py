"""
============================================================================
===[ ENVIRONMENT PROBE ]===
============================================================================
Answers the Phase-0 questions for putting real objects into the maze, WITHOUT
flying anything, arming anything, or taking API control.

    py probe_env.py                 # the gate: which server am I talking to?
    py probe_env.py --grep table    # search the asset registry
    py probe_env.py --calibrate     # work out the scale that fills one cell
    py probe_env.py --shot          # save a picture of whatever was spawned

WHY THIS EXISTS
---------------
MAZE_SWARM_AIRSIM_RT.py's build_maze() spawns the asset named "Cube" at scale
(3.122, 3.122, 2.0), a number calibrated against the packaged NeurIPS binary's
own mesh.  That binary is a 2019 build whose server does NOT implement
simListAssets and predates blueprint spawning, so no imported asset can ever be
spawned into it.  The Blocks project built from AirSimOfficial DOES implement
both (WorldSimApi.cpp:80 and :124).

`--gate` is therefore the single question that matters first: if simListAssets
answers, you are on the new server and the whole prop plan is open.  If it
raises, you are still on the old binary and nothing else here will work.

TWO SHARP EDGES THIS SCRIPT IS BUILT AROUND
-------------------------------------------
1. A BAD ASSET NAME CRASHES THE SIMULATOR, not this script.  WorldSimApi.cpp:94
   does `asset_map.Find(name)` and then calls `load_asset->IsValid()` on the
   result WITHOUT a null check, so an unknown name is a null dereference inside
   UE.  Every spawn below is therefore refused unless the name came back from
   simListAssets first.  Do not remove that guard.

2. The registry is keyed by BARE ASSET NAME, not by content path
   (AirBlueprintLib.cpp:238 stores `asset.AssetName.ToString()`).  So a table
   imported as SM_Table_01 spawns as "SM_Table_01" -- never "/Game/Props/
   SM_Table_01".  The full path is what you would use in the editor, and it is
   NOT what this API wants.

3. REUSING AN OBJECT NAME RIGHT AFTER DESTROYING IT KILLS THE SIMULATOR.
   Learned the hard way (2026-08-26): destroying probe_cal_0 and immediately
   respawning it took the editor down with

       Fatal error: [LevelActor.cpp:417]
       An actor of name 'probe_cal_0' already exists in level ...
       WorldSimApi::createNewStaticMeshActor()  WorldSimApi.cpp:151

   simDestroyObject is DEFERRED: UE keeps the FName reserved until garbage
   collection, and SpawnActor with an explicit duplicate FName is fatal rather
   than recoverable.  spawnObject's own dedup (WorldSimApi.cpp:106) does not
   save you, because by then the actor is already gone from the name search
   while its FName is still taken.

   MAZE_SWARM_AIRSIM_RT.build_maze() survives the same destroy-then-respawn
   pattern only because its pre-clean (:2331) retries with a 0.5 s pause.  That
   pause is load-bearing.  This script sidesteps the problem entirely by giving
   every spawn a run-unique name, so it never reuses one at all.
"""

import argparse
import sys
import time

# One cell of the occupancy grid, from MAZE_SWARM_AIRSIM_RT.py:140.  A wall cube
# is scaled to fill exactly this, so it is also the number --calibrate solves for.
CELL_SIZE = 8.0

# Where the calibration cubes go: well clear of the maze footprint (which spans
# +/-68 m for a default 8x8) so nothing lands on a parked drone or a wall.
PROBE_X, PROBE_Y = 200.0, 0.0

# Names worth flagging when the registry is long.  Purely a convenience filter
# for reading the output -- it does not restrict what can be spawned.
INTERESTING = ("table", "chair", "desk", "shelf", "crate", "box", "barrel",
               "pallet", "container", "cabinet", "bench", "sofa", "lamp",
               "prop", "furniture", "sm_", "bp_")


def connect():
    """Plain client, no API control taken.  Returns None if nothing answers."""
    try:
        import airsim
    except ImportError as exc:
        print("  cannot import airsim (%s)" % exc)
        return None, None
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
    except Exception as exc:                          # noqa: BLE001
        print("  no simulator on 127.0.0.1:41451 (%s)" % exc)
        print("  -> start the sim first.  For the Blocks project that means")
        print("     opening Blocks.uproject in UE 4.27 and pressing Play.")
        return None, None
    return client, airsim


def list_assets(client):
    """THE GATE.  Returns a sorted list, or None if this server is too old.

    The 2019 NeurIPS binary has no simListAssets bound at all, so the RPC comes
    back as a 'method not found'-flavoured error rather than an empty list.  An
    empty list would mean something different (a server that supports the call
    but has an empty registry), so the two are reported separately.
    """
    try:
        assets = client.simListAssets()
    except Exception as exc:                          # noqa: BLE001
        print("  simListAssets FAILED: %s" % exc)
        print()
        print("  This is the old packaged server (AirSimExe.exe, 2019 build).")
        print("  It cannot spawn imported assets no matter what you do to it --")
        print("  its content is sealed in .pak files and there is no source")
        print("  project for it.  Switch to the Blocks project instead:")
        print("    Documents\\AirSimOfficial\\Unreal\\Environments\\Blocks\\Blocks.uproject")
        return None
    return sorted(assets)


def report_assets(assets, pattern=None):
    if pattern:
        pat = pattern.lower()
        hits = [a for a in assets if pat in a.lower()]
        print("  %d of %d assets match %r:" % (len(hits), len(assets), pattern))
        for a in hits:
            print("     %s" % a)
        if not hits:
            print("     (none -- import it into the project's Content first,")
            print("      then restart PIE so the registry is rebuilt)")
        return

    print("  %d assets in the registry" % len(assets))
    flagged = [a for a in assets if any(k in a.lower() for k in INTERESTING)]
    if flagged:
        print("  possibly useful as props:")
        for a in flagged[:40]:
            print("     %s" % a)
        if len(flagged) > 40:
            print("     ... and %d more" % (len(flagged) - 40))
    print("  full list:")
    for a in assets:
        print("     %s" % a)


def calibrate(client, airsim, assets, asset, scale):
    """Spawn one mesh at a known scale so its real-world size can be measured.

    There is no bounding-box RPC in this API version, so the size cannot be read
    back numerically -- which is exactly why this spawns a RULER too: a second
    copy placed CELL_SIZE metres away.  If the chosen scale is correct for one
    grid cell, the two cubes touch exactly, with no gap and no overlap.  That is
    a far more reliable check than trusting the asset's name.
    """
    if asset not in assets:
        print("  REFUSED: %r is not in the registry." % asset)
        print("  Spawning an unknown name would null-dereference inside UE and")
        print("  take the simulator down (WorldSimApi.cpp:94).  Check the exact")
        print("  spelling with:  py probe_env.py --grep %s" % asset[:12])
        return

    print("  spawning two %r at scale %g, %g m apart" % (asset, scale, CELL_SIZE))
    print("  -> if the scale is right for one cell they touch exactly")
    # Run-unique suffix: never reuse a name a previous run may have destroyed,
    # because a still-reserved FName makes SpawnActor FATAL (see header note 3).
    stamp = int(time.time()) % 100000
    spawned = []
    for i in (0, 1):
        name = "probe_cal_%d_%d" % (stamp, i)
        pose = airsim.Pose(
            airsim.Vector3r(PROBE_X + i * CELL_SIZE, PROBE_Y, -1.0),
            airsim.to_quaternion(0, 0, 0))
        try:
            actual = client.simSpawnObject(
                name, asset, pose, airsim.Vector3r(scale, scale, scale))
            spawned.append(actual)
            print("     %s -> actor %r" % (name, actual))
        except Exception as exc:                      # noqa: BLE001
            print("     spawn failed: %s" % exc)
            return
    print()
    print("  Look at them in the viewport, or:  py probe_env.py --shot")
    print("  Clean up with:                     py probe_env.py --clean")
    return spawned


def shot(client, airsim, path):
    """Grab one picture from an external camera pointed at the probe area.

    Uses the same compress=True / raw-PNG-write trick swarm_capture.py uses, so
    it needs no opencv and no numpy.
    """
    try:
        client.simSetCameraPose  # noqa: B018 - presence check only
    except AttributeError:
        pass
    req = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, True)]
    try:
        resp = client.simGetImages(req)
    except Exception as exc:                          # noqa: BLE001
        print("  simGetImages failed: %s" % exc)
        return
    if not resp or not resp[0].image_data_uint8:
        print("  empty frame (render target not ready?)")
        return
    data = resp[0].image_data_uint8
    if not isinstance(data, (bytes, bytearray)):
        data = bytes(data)
    airsim.write_file(path, data)
    print("  wrote %s (%d bytes, %dx%d)"
          % (path, len(data), resp[0].width, resp[0].height))


def clean(client):
    """Remove anything this script spawned.  Safe to run at any time."""
    removed = 0
    try:
        for obj in client.simListSceneObjects():
            if obj.startswith("probe_"):
                try:
                    client.simDestroyObject(obj)
                    removed += 1
                except Exception:                     # noqa: BLE001
                    pass
    except Exception as exc:                          # noqa: BLE001
        print("  scene scan failed: %s" % exc)
        return
    print("  removed %d probe object(s)" % removed)


def vehicles(client):
    """What vehicles exist -- swarm_capture's --drones N assumes drone_1..N."""
    try:
        objs = client.simListSceneObjects()
    except Exception as exc:                          # noqa: BLE001
        print("  scene scan failed: %s" % exc)
        return
    drones = sorted(o for o in objs if o.lower().startswith("drone"))
    print("  %d scene objects; drone-named actors: %s"
          % (len(objs), drones if drones else "(none)"))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Probe a running AirSim for asset-spawning capability. "
                    "Never arms, flies, or takes API control.")
    ap.add_argument("--grep", metavar="TEXT",
                    help="list only assets whose name contains TEXT "
                         "(case-insensitive)")
    ap.add_argument("--all", action="store_true",
                    help="print the whole asset registry, not just the summary")
    ap.add_argument("--calibrate", action="store_true",
                    help=f"spawn two cubes {CELL_SIZE:g} m apart to check the "
                         f"scale that fills one grid cell")
    ap.add_argument("--asset", default="1M_Cube",
                    help="asset to calibrate with (default 1M_Cube, the mesh "
                         "Blocks ships; the packaged NeurIPS env calls its own "
                         "cube 'Cube' instead)")
    ap.add_argument("--scale", type=float, default=CELL_SIZE,
                    help=f"scale to spawn at (default {CELL_SIZE:g}, correct if "
                         f"the source mesh really is 1 m)")
    ap.add_argument("--shot", metavar="PATH", nargs="?", const="probe_shot.png",
                    help="save one frame to PATH (default probe_shot.png)")
    ap.add_argument("--clean", action="store_true",
                    help="destroy every object this script spawned and exit")
    args = ap.parse_args(argv)

    client, airsim = connect()
    if client is None:
        return 2
    print("connected.")

    if args.clean:
        clean(client)
        return 0

    print()
    print("-- gate: does this server support asset listing? --")
    assets = list_assets(client)
    if assets is None:
        return 1
    print("  OK -- simListAssets answered, so this is a modern server.")
    print()

    print("-- vehicles --")
    vehicles(client)
    print()

    if args.grep or args.all:
        print("-- assets --")
        report_assets(assets, args.grep)
        print()
    else:
        print("-- assets (summary; use --all or --grep to see more) --")
        print("  %d in the registry" % len(assets))
        flagged = [a for a in assets if any(k in a.lower() for k in INTERESTING)]
        print("  %d look like props/meshes worth trying" % len(flagged))
        for a in flagged[:15]:
            print("     %s" % a)
        print()

    if args.calibrate:
        print("-- calibration --")
        calibrate(client, airsim, assets, args.asset, args.scale)
        print()

    if args.shot:
        print("-- screenshot --")
        shot(client, airsim, args.shot)
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
