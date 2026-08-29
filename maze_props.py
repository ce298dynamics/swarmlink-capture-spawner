"""
============================================================================
===[ MAZE PROPS ]===
============================================================================
Places real objects (tables, chairs, couches...) into the maze so the FPV
capture has something to look at besides grey cubes.

    py maze_props.py --list                 # catalogue + measured sizes
    py maze_props.py --demo                 # spawn a row of props to look at
    py maze_props.py --clear                # remove every prop this places

WHY PROPS GO IN *FREE* CELLS, NEVER IN WALL CELLS
-------------------------------------------------
It is tempting to swap a table in for a wall cube.  Do not.  The numbers make
it a non-starter, and all three were measured live rather than assumed:

    one wall cube ....... 8.0 m x 8.0 m footprint, ~5.1 m tall (top at 3.56 m)
    drone altitudes ..... 1.8 m (ALT_BASE) to 3.3 m for the top layer
    SM_TableRound ....... 1.12 m wide, 0.75 m tall

A table is ~7x too short, so every drone flies straight over it.  Worse, the
planner would not notice: MAZE_SWARM_AIRSIM_RT._is_open() (:2396) tests the 0/1
occupancy grid, never the spawned actors.  Putting a table in a wall cell leaves
that cell logically solid, so drones route around an obstacle that is not
physically there, while simGetCollisionInfo reports nothing when they clip the
space a wall used to occupy.  The rendered world and the planner's world model
would drift apart silently.

Props therefore go ONLY in cells the grid already marks free, and the grid is
never modified.  Flight behaviour is bit-for-bit unchanged; the cameras simply
see more.  Verify that with an A/B run at the same seed: coverage time and
per-drone collisions must be identical with and without props.

The one prop tall enough to reach wall height is SM_PillarFrame (4.21 m), but it
is 0.19 m wide -- a frame, not a wall.  Nothing in this catalogue can replace a
wall cube, which is why no code here tries.

TWO WAYS TO KILL THE SIMULATOR, BOTH AVOIDED HERE
-------------------------------------------------
1. Spawning an asset name that is not in the registry.  WorldSimApi.cpp:94 does
   asset_map.Find(name) and then load_asset->IsValid() with no null check, so an
   unknown name is a null dereference inside UE.  place() therefore validates
   every asset against simListAssets() first and drops anything missing.

2. Reusing an object name that was just destroyed.  simDestroyObject is
   DEFERRED -- UE holds the FName until garbage collection, and SpawnActor with
   a duplicate explicit FName is a FATAL error (LevelActor.cpp:417), not an
   exception.  Every name here carries a run-unique stamp so a name is never
   reused.  (build_maze() gets away with reusing swarm_wall_* only because its
   pre-clean at :2331 pauses 0.5 s for GC.  That pause is load-bearing.)
"""

import argparse
import math
import random
import time

# Prefix for everything this module spawns, so clear() can find them and so they
# never collide with build_maze()'s swarm_wall_* actors.
PROP_PREFIX = "swarm_prop"

# Measured live on 2026-08-27 against UE 4.27 Blocks, by spawning each asset at
# scale 1 and sizing its segmentation silhouette at a known distance.  Sizes are
# the projected bounding box, so width is the widest axis seen face-on -- good
# to ~0.1 m, which is all the placement logic needs.
#
#   asset        : bare registry name (NOT a /Game/... path -- the registry is
#                  keyed by asset name, AirBlueprintLib.cpp:238)
#   w, h         : metres at scale 1.0
#   label        : semantic tag written into the capture manifest
#   blueprint    : pass is_blueprint=True for this asset (see note below)
#
# All of these are StaticMeshes today.  A Fab *blueprint* prop would set
# blueprint=True; the flag must match the asset's real type, because the
# static-mesh branch does dynamic_cast<UStaticMesh*> on whatever it gets
# (WorldSimApi.cpp:127) and silently yields null for a UBlueprint.
CATALOG = {
    "table":   dict(asset="SM_TableRound",  w=1.12, h=0.75, label="table",   blueprint=False),
    "chair":   dict(asset="SM_Chair",       w=0.94, h=1.12, label="chair",   blueprint=False),
    "couch":   dict(asset="SM_Couch",       w=2.99, h=1.12, label="couch",   blueprint=False),
    "shelf":   dict(asset="SM_Shelf",       w=2.06, h=0.19, label="shelf",   blueprint=False),
    "stairs":  dict(asset="SM_Stairs",      w=2.06, h=0.47, label="stairs",  blueprint=False),
    "bush":    dict(asset="SM_Bush",        w=1.12, h=1.31, label="bush",    blueprint=False),
    # THE HUMAN: a RenderPeople photogrammetry scan (free, FBX, no rig).
    # Preferred over the mannequin below for two reasons that matter here:
    #   * it is a STATIC MESH, so it spawns directly -- no blueprint, none of
    #     the is_blueprint plumbing, and it cannot be mistaken for a skeletal
    #     mesh (which AirSim cannot spawn at all);
    #   * having no animation rig, it carries no weapon-ready arm posture.  The
    #     UE mannequin's whole animation set sits on a rifle-carry rig, so BOTH
    #     of its poses hold the arms up at chest height.
    # z=+0.73 because the mesh pivot sits 0.73 m BELOW the feet -- spawned at
    # ground level it hovers.  Measured, not guessed: at z=+0.73 the feet land
    # at world z=+0.06.
    "person":  dict(asset="rp_posed_00178_29", w=0.69, h=1.81, z=0.73,
                    label="human", blueprint=False),
    # The stock UE mannequin, kept as a fallback.  Engine BLUEPRINT, so it needs
    # is_blueprint=True -- the static-mesh branch would dynamic_cast<UStaticMesh*>
    # a UBlueprint and get null.  BP_Human_Walk / BP_Human_Idle (built by
    # ue_make_human_bp.py) are posed variants of the same mannequin.
    "mannequin": dict(asset="TutorialCharacter", w=0.66, h=1.87,
                      label="human", blueprint=True),
    # --- taller than the lowest flight layer: NOT placed by default ---
    "door":    dict(asset="SM_Door",        w=0.94, h=2.06, label="door",    blueprint=False),
    "rock":    dict(asset="SM_Rock",        w=2.43, h=2.90, label="rock",    blueprint=False),
    "pillar":  dict(asset="SM_PillarFrame", w=0.19, h=4.21, label="pillar",  blueprint=False),
}

# THE HEIGHT CEILING, and why it matters more than it looks.
#
# Props live in FREE cells -- which is exactly where drones fly.  So a prop is
# only safe if the lowest drone passes OVER it.  From MAZE_SWARM_AIRSIM_RT:140,
# ALT_BASE = -1.8, i.e. drone 0 cruises at 1.8 m and each higher id adds
# ALT_SEP = 0.5 m.  Anything taller than ~1.8 m is therefore in drone 0's path.
#
# This is not a cosmetic concern.  A prop that gets hit shows up as a WALL strike
# in check_collisions() (:2244), which classifies by object-name prefix and would
# blame the maze.  It would also change coverage time, quietly invalidating any
# A/B comparison against a no-props run -- the exact regression check that proves
# props are inert.
ALT_BASE_M = 1.8            # mirrors ALT_BASE in MAZE_SWARM_AIRSIM_RT.py:144
PROP_CLEARANCE_M = 0.3      # margin for altitude sag and prop bounding slop
MAX_SAFE_HEIGHT_M = ALT_BASE_M - PROP_CLEARANCE_M   # 1.5 m

# Placed by default: everything that fits under the lowest flight layer.
# "human" is included because place() auto-scales it to fit (see _fit_scale).
SAFE_KINDS = ("table", "chair", "couch", "shelf", "stairs", "bush", "person")

# Props are placed on the ground.  These meshes have their pivot at the base, so
# the spawn z IS the floor -- confirmed by eye: a table spawned at the drone's
# own z sits correctly on the ground rather than half-buried or floating.
GROUND_Z = 0.0

# Keep a prop clear of the cell edge so it cannot poke into a neighbouring
# corridor a drone is flying down.  A cell is 8 m; the widest prop is 3 m.
CELL_MARGIN_M = 0.6


def _fit_scale(spec, scale=1.0):
    """Scale this prop down if it would otherwise reach drone 0's flight layer.

    Returns `scale` unchanged when the prop already fits.  Only the height is
    considered: a prop can be as wide as it likes (the widest is the 3 m couch,
    comfortably inside an 8 m cell) but it must pass UNDER the lowest drone.
    """
    if spec["h"] <= 0:
        return scale
    # height at scale s is spec["h"] * s, so the cap is MAX / spec["h"]
    return min(scale, MAX_SAFE_HEIGHT_M / spec["h"])


def available(client, catalog=None):
    """Which catalogue entries this simulator can actually spawn.

    Checked against simListAssets() rather than assumed, because an unlisted
    name is a null dereference in UE, not a Python error -- it takes the whole
    simulator down (see header note 1).
    """
    catalog = catalog or CATALOG
    try:
        registry = set(client.simListAssets())
    except Exception as exc:                          # noqa: BLE001
        # Old NeurIPS server: simListAssets is not bound at all.  Refuse to
        # spawn anything rather than risk probing names blind.
        raise RuntimeError(
            "simListAssets unavailable (%s) -- this looks like the packaged "
            "2019 AirSim binary, which cannot spawn imported assets at all. "
            "Use the Blocks UE 4.27 project instead." % exc)
    ok = {k: v for k, v in catalog.items() if v["asset"] in registry}
    missing = sorted(set(catalog) - set(ok))
    return ok, missing


def place(client, positions, kinds=None, seed=0, scale=1.0, quiet=False):
    """Spawn one prop at each (x, y) in `positions`.

    `positions` are world metres and must already be the centres of FREE cells --
    this function does not consult the grid and will happily put a table inside a
    wall if handed one.  Callers should pass only cells where grid[r][c] == 0.

    Returns [(actor_name, kind, x, y), ...] for the manifest.
    """
    ok, missing = available(client)
    if not ok:
        raise RuntimeError("none of the catalogue assets are in this registry; "
                           "did StarterContent get imported, and was PIE "
                           "restarted afterwards?")
    if missing and not quiet:
        print("  props unavailable in this environment: %s" % ", ".join(missing))

    kinds = [k for k in (kinds or SAFE_KINDS) if k in ok]
    if not kinds:
        raise RuntimeError("no usable prop kinds requested")

    rng = random.Random(seed)
    # Run-unique stamp: never reuse an FName a previous run may have destroyed
    # (see header note 2 -- reuse before GC is fatal, not catchable).
    stamp = int(time.time()) % 100000
    placed = []
    for i, (x, y) in enumerate(positions):
        kind = kinds[i % len(kinds)] if len(kinds) > 1 else kinds[0]
        spec = ok[kind]
        # Shrink anything that would otherwise stand in drone 0's flight layer.
        # For the human this means ~1.68 m instead of 1.87 m, which is still an
        # entirely ordinary adult height -- the prop stays realistic and the
        # flight stays untouched.
        s = _fit_scale(spec, scale)
        if s < scale and not quiet:
            print("  %-7s scaled %.2f -> %.2f so it clears the %.1f m flight layer"
                  % (kind, scale, s, ALT_BASE_M))
        name = "%s_%d_%d" % (PROP_PREFIX, stamp, i)
        yaw = rng.uniform(0.0, 2.0 * math.pi)      # vary heading so the capture
                                                   # does not see identical views
        try:
            import airsim
            # Per-prop pivot offset, in metres BELOW the feet, and it has to
            # scale with the prop: shrink the mesh and the pivot-to-feet gap
            # shrinks with it, so a fixed offset would bury or float it.
            z = GROUND_Z + spec.get("z", 0.0) * s
            pose = airsim.Pose(airsim.Vector3r(float(x), float(y), z),
                               airsim.to_quaternion(0, 0, yaw))
            actor = client.simSpawnObject(
                name, spec["asset"], pose,
                airsim.Vector3r(s, s, s),
                False, bool(spec["blueprint"]))
            placed.append((actor, kind, float(x), float(y)))
        except Exception as exc:                   # noqa: BLE001
            if not quiet:
                print("  prop %s (%s) failed: %s" % (name, spec["asset"], exc))
    if not quiet:
        print("  placed %d prop(s)" % len(placed))
    return placed


def clear(client, quiet=False):
    """Destroy every prop this module has ever spawned.  Safe to call anytime.

    Note this does NOT wait for GC afterwards.  That is fine because place()
    never reuses a name -- but if you ever add name reuse, pause here.
    """
    removed = 0
    try:
        for obj in client.simListSceneObjects():
            if obj.startswith(PROP_PREFIX):
                try:
                    client.simDestroyObject(obj)
                    removed += 1
                except Exception:                  # noqa: BLE001
                    pass
    except Exception as exc:                       # noqa: BLE001
        if not quiet:
            print("  scene scan failed: %s" % exc)
        return 0
    if not quiet:
        print("  removed %d prop(s)" % removed)
    return removed


def free_cell_positions(grid, grid_to_world, density=0.15, seed=0,
                        skip=()):
    """Pick world positions from the occupancy grid's FREE cells.

    `grid` is MAZE_SWARM_AIRSIM_RT's list-of-lists where 1 == wall, and
    `grid_to_world(r, c)` is its mapper.  Cells in `skip` (e.g. the start cell,
    or anywhere a drone is staged) are excluded.

    The grid itself is never modified -- that is the whole point.
    """
    rng = random.Random(seed)
    cells = [(r, c)
             for r, row in enumerate(grid)
             for c, v in enumerate(row)
             if v == 0 and (r, c) not in skip]
    rng.shuffle(cells)
    n = max(0, int(len(cells) * density))
    out = []
    for r, c in cells[:n]:
        x, y, _ = grid_to_world(r, c)
        out.append((x, y))
    return out


def _main(argv=None):
    ap = argparse.ArgumentParser(
        description="Place real objects into the maze for the FPV capture. "
                    "Props go in FREE cells only and never touch the grid.")
    ap.add_argument("--list", action="store_true",
                    help="show the catalogue with measured sizes and exit")
    ap.add_argument("--demo", type=int, nargs="?", const=6, metavar="N",
                    help="spawn N props in a row in front of drone_1 (default 6)")
    ap.add_argument("--clear", action="store_true",
                    help="remove every prop and exit")
    ap.add_argument("--scale", type=float, default=1.0)
    args = ap.parse_args(argv)

    if args.list:
        print("prop catalogue (sizes measured at scale 1.0):")
        print("  %-9s %-16s %6s %6s  %s" % ("kind", "asset", "w(m)", "h(m)", "note"))
        for k, v in sorted(CATALOG.items(), key=lambda t: -t[1]["h"]):
            note = "reaches wall height" if v["h"] >= 3.6 else "floor prop"
            print("  %-9s %-16s %6.2f %6.2f  %s" % (k, v["asset"], v["w"], v["h"], note))
        print()
        print("  a wall cube is 8.0 x 8.0 m and ~5.1 m tall; drones fly 1.8-3.3 m")
        print("  -> nothing here can substitute for a wall, hence free cells only")
        return 0

    import airsim
    # Fail with a sentence, not a msgpackrpc traceback.  "Retry connection over
    # the limit" is what the RPC layer raises when nothing is listening, and it
    # tells a first-time user nothing about what to do.
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
    except Exception as exc:                          # noqa: BLE001
        print("  no simulator on 127.0.0.1:41451 (%s)" % type(exc).__name__)
        print("  -> start the sim first, and press Play if you are running")
        print("     from the Unreal Editor (the RPC server only runs in PIE).")
        return 2

    if args.clear:
        clear(client)
        return 0

    if args.demo:
        ok, missing = available(client)
        print("  usable props: %s" % ", ".join(sorted(ok)))
        if missing:
            print("  missing: %s" % ", ".join(missing))
        p = client.simGetVehiclePose("drone_1")
        yaw = math.pi                       # the open direction from the origin
        base_x, base_y = p.position.x_val, p.position.y_val
        pos = []
        for i in range(args.demo):
            d = 8.0 + i * 3.0
            lateral = (i % 3 - 1) * 3.0
            pos.append((base_x + d * math.cos(yaw) - lateral * math.sin(yaw),
                        base_y + d * math.sin(yaw) + lateral * math.cos(yaw)))
        placed = place(client, pos, seed=1, scale=args.scale)
        for name, kind, x, y in placed:
            print("     %-24s %-8s (%.1f, %.1f)" % (name, kind, x, y))
        print()
        print("  clear them with:  py maze_props.py --clear")
        return 0

    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
