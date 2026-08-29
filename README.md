# swarmlink-capture-spawner

Two tools for making an AirSim swarm simulation produce useful **vision data**.
They are independent — use either alone — but they solve two halves of one
problem: photographs of an empty grey maze teach a model nothing, and objects
nobody photographs are equally useless.

| | tool | needs |
| --- | --- | --- |
| **1. Recording** | [`swarm_capture.py`](#1-recording-the-flight) | Python + AirSim. Nothing else. |
| **2. Populating** | [`maze_props.py`](#2-populating-the-world) and the `ue_*.py` scripts | Unreal Editor 4.27, two plugins, and an editable project. |

**The prerequisites differ sharply**, which is why they are documented
separately. Part 1 is `pip install` and go. Part 2 needs an Unreal project you
can import assets into — a packaged AirSim binary cannot do it at all, because
its content is sealed in `.pak` files and `simSpawnObject` can only resolve names
already cooked into them.

Both were extracted from a larger swarm drone-racing project, which is what the
`MAZE_SWARM_AIRSIM_RT.py` and `ASTAR_DRONE` references in the source comments
refer to. Neither tool imports anything from it.

---

# 1. Recording the flight

Records the **first-person view of every drone in an AirSim swarm to disk while
the flight is happening**, one PNG per drone per capture round, without slowing
the flight-control loop down.

Single file, no package to install, stdlib-only at import time.

## Requirements

- **Python 3**, then `pip install -r requirements.txt`.
- **AirSim already running**, with the vehicles spawned. The recorder connects
  over the usual RPC port (41451) and never launches the simulator itself.
- **Vehicles named `drone_1`, `drone_2`, … `drone_N`** — but only for the
  standalone smoke test, which builds those names from `--drones N`. The
  `FPVRecorder` class itself takes whatever names you give it, so if your
  `settings.json` uses different ones, pass your own mapping (see
  [Using it in your own flight loop](#using-it-in-your-own-flight-loop)).

If the smoke test reports `camera 'front_center' resolved on none of [...]`,
**check the vehicle names before the camera name.** That message is printed
whenever the per-vehicle probe fails, and a vehicle the recorder cannot reach
is a far more common cause than a genuinely missing camera.

## Quickstart

```bash
pip install -r requirements.txt
```

Start AirSim, then run the following with the drones stationary. Nothing flies
and no API control is taken:

```bash
py swarm_capture.py --drones 2 --frames 5
```

```
FPV smoke test: 5 rounds at 2.0 Hz, camera 'front_center', vehicles ['drone_1', 'drone_2']
  -> .../results/frames/smoke_20260823_223738
  FPV capture: 15 frames (7/drone), 0 slots dropped, 0 errors, 183 ms mean grab
  OK   drone_1: 8 png (first 43383 bytes)
  OK   drone_2: 7 png (first 43463 bytes)
```

Run this smoke test before anything else: it confirms your camera name resolves
on every vehicle before you commit a real flight to it. Flags: `--drones`,
`--frames`, `--hz`, `--camera`, `--out`.

`--frames` is approximate. The smoke test captures for a fixed duration with a
little slack, so asking for 5 rounds at 2 Hz yields 7-8 frames per drone rather
than exactly 5, and the two drones can differ by one at the tail. What matters
in the output above is `0 slots dropped, 0 errors` and an `OK` line per vehicle.

## Output layout

```
<out_dir>/
    drone_1/000000_t0.548.png
            000001_t1.053.png
    drone_2/000000_t0.548.png
            000001_t1.053.png
    manifest.csv
```

Every drone targets the **same wall-clock slot schedule**, so frame `000123` is
the same instant for all of them (within a few ms) and the folders stack
directly into a synchronized multi-view video. A drone that misses a slot skips
that index rather than renumbering, so indices never drift apart.

`manifest.csv` carries one row per frame:

| column | meaning |
| --- | --- |
| `t` | seconds since the caller's `t0`, also in the filename |
| `drone`, `vehicle` | simulation id and AirSim vehicle name |
| `frame`, `file` | slot index and path relative to `out_dir` |
| `cam_x/y/z`, `q_w/x/y/z` | camera pose **at render time**, taken from the image response. This costs no extra RPC and describes the pixels more accurately than a separately polled body pose |
| `airsim_ts`, `rpc_ms` | AirSim's own timestamp, and how long the grab took |
| `mode`, `cell_x`, `cell_y` | whatever your `state_fn` returned, blank if unused |

Pass your flight loop's own `t0` and the manifest joins directly against your
flight log on `(t, drone)`.

## Using it in your own flight loop

```python
from swarm_capture import FPVRecorder

rec = FPVRecorder({0: "drone_1", 1: "drone_2"}, out_dir, t0=t0)
if rec.start():            # False = capture unavailable, flight continues
    ...fly...
rec.stop()                 # idempotent; safe to call from both wrap-up paths
print(rec.summary_line())
```

- `names` takes `{drone_id: "vehicle_name"}` or a bare sequence of names.
- `start()` blocks until every thread has connected and probed its camera (up to
  20 s), so an incorrect camera name is reported *before* the flight rather than
  raising once per drone per round for the whole run. A vehicle whose camera
  does not resolve is dropped and the remaining vehicles still record, so a
  partial configuration still produces usable data.
- `state_fn(drone_id) -> (mode, cell_x, cell_y)` is an optional annotation hook,
  so a frame can be interpreted without cross-referencing anything. It is called
  **from the capture threads**, so it must be read-only and must not touch your
  flight client. Exceptions raised inside it are suppressed, since annotation is
  never fatal.
- `stop()` joins the threads and flushes the manifest. The threads are daemons,
  so a stalled image RPC cannot hold your process open, but you must still call
  `stop()`. If your `__main__` ends in `os._exit(0)`, daemon threads are killed
  with no flush.
- `stats()` returns `(captured, dropped, errors, mean_rpc_ms)`.

## Why a thread and a client per drone

This design follows from measurement rather than from anticipated need.

**A `simGetImages` call costs render-sync latency far above an ordinary RPC,
and that cost is per call — not per image, and not per pixel.** Measured on 4
stationary drones, 256x144 Scene:

| | |
| --- | --- |
| one `simGetImages` call, 1 image | **335 ms** |
| one call, 4 images (same vehicle) | 334 ms, so batching is **free** |
| `compress=True` (43 KB PNG) vs `False` (110 KB raw) | no difference |
| 256x144 vs 320x240 | no difference, so **resolution is free** |
| plain `getMultirotorState` / `simGetVehiclePose` | 2 ms |

So the cost is neither PNG encoding, nor resolution, nor bytes on the wire.
`vehicle_name` is a per-call argument, so there is no batch form across
vehicles and a serial loop pays that latency once per drone:

| | |
| --- | --- |
| serial, 1 client, 4 drones/round | 1331 ms/round |
| parallel, 4 clients on 4 threads | **342 ms/round** |

The overlap is nearly perfect, because the cost is **latency, not throughput**:
the drones wait on the render thread concurrently. That makes the ceiling
**flat in drone count** — roughly one round per grab, however many drones you
record.

**The absolute number is scene-dependent; the structure is not.** Re-measured
later against a lighter scene:

| configuration | mean grab | dropped | errors |
| --- | --- | --- | --- |
| 2 drones, `front_center` (256x144) | 183 ms | 0 | 0 |
| 2 drones, `fpv_cam` (320x240) | 82 ms | 0 | 0 |
| 4 drones, `front_center` (256x144) | 93 ms | 0 | 0 |
| full 88 s flight, 2 drones | 87 ms | 0 | 0 |

Grab time varied by 3-4x with scene load, but **4 drones still cost no more
than 2** — the overlap holds, which is the part the design depends on. So
treat 335 ms as the worst case to size against: at that floor the ceiling is
~2.9 rounds/s, which is why `CAPTURE_HZ` defaults to 2.0, and a lighter scene
simply hands you headroom. The `dropped` counter in the wrap-up line is the
ground truth for your sim — raise `--capture-hz` until it starts climbing.

**Capture does not interfere with the flight loop.** With all four capture
threads running continuously, a separate client polling `getMultirotorState` and
`simGetVehiclePose` measured median 1.00 ms / p90 1.01 / max 1.08, against a
no-capture baseline of median 1.00 / p90 1.01 / max 3.01. A 20 Hz control loop
has a 50 ms budget, so a grab of even 87 ms would consume whole passes, and one
at the 335 ms worst case would eat seven. That is why capture runs on its own
threads rather than inside the loop.

## Two constraints worth knowing

**Cameras are declared per vehicle** in `Documents\AirSim\settings.json`. The
default is `front_center` deliberately: it is one of the five built-in cameras
(`front_center` 0, `front_right` 1, `front_left` 2, `bottom_center` 3,
`back_center` 4) that every multirotor receives automatically, so it resolves
everywhere, including on vehicles added at runtime via `simAddVehicle` that were
never in `settings.json`. A custom name such as `fpv_cam` resolves only on
vehicles that declare one. Since **resolution costs nothing at grab time**,
declaring a larger camera on every vehicle is the least expensive way to obtain
a higher-resolution image.

**The client must be constructed inside the thread that uses it.**
`msgpack-rpc-python` derives its tornado IOLoop from `IOLoop.current()`, which is
**thread-local**, so a client constructed on the main thread and then driven
from another runs a foreign thread's event loop. `FPVRecorder` therefore
constructs its own clients inside its own threads and never accepts one from the
caller. It also never touches your flight client, so it composes with a
single-threaded, lock-free flight loop.

## Known limitation

A body-fixed camera **pitches with the airframe**. Over one 88 s run, `|pitch|`
measured median 2.8°, p90 10.1°, worst −24.9° / +31.7°. Roughly 10% of frames
tilt past 10°, and some point at open sky during hard braking. This is expected
behaviour for a body-fixed camera rather than a defect. A stabilized view would
require `simSetCameraPose` counter-rotation on each grab, which is not
implemented here.

---

# 2. Populating the world

Places real objects — furniture, and a photoreal scanned **person** — into a
maze so the capture above has something worth photographing.

## Requirements

Unlike part 1, this needs an **Unreal project you can import into**:

- **Unreal Editor 4.27** with your AirSim environment as an editable project.
  A *packaged* AirSim binary cannot do this at all: its content is sealed in
  `.pak` files, and older builds do not even implement `simListAssets`.
- **Two editor plugins enabled** in your `.uproject`:
  `PythonScriptPlugin` and `EditorScriptingUtilities`.
- Python + `airsim`, as in part 1.

Check you are on a capable simulator before anything else:

```bash
py probe_env.py
```

If `simListAssets` answers, you are fine. If it fails as an unknown method, you
are on an old packaged binary and no imported asset can ever be spawned into it.

## The one rule: props go in FREE cells, never in wall cells

It is tempting to swap a table in for a wall block. Don't. Measured against the
maze this came from:

| | footprint | height |
| --- | --- | --- |
| one wall block | 8.0 × 8.0 m | **5.1 m** |
| drone flight layers | — | 1.8 – 3.3 m |
| a table | 1.12 m | **0.75 m** |

A table is ~7× too short, so every drone flies straight over it. Worse, the
planner never notices: obstacle avoidance reads a **0/1 occupancy grid**, not the
spawned actors. A table in a wall cell leaves that cell logically solid — drones
route around geometry that is not there, while collision reporting stays silent
when they clip the space a wall used to fill.

So `maze_props.py` places only into cells the grid already marks free, and
**never modifies the grid**. Flight behaviour is unchanged by construction.

Check it rather than trusting it: run the same seed with and without props.
**Collision counts must be identical** — that is the real test, and it is exact.
Coverage time will differ slightly (the loop is real-time, not tick-based), so
treat a fraction of a second as noise and a systematic slowdown as a problem.

Measured on a 5x5 seed-3 maze, 2 drones, 14 props:

| | no props | 14 props |
| --- | --- | --- |
| coverage time | 89.0 s | 88.5 s |
| collisions | 0 | **0** |
| frames captured | 356 | 354 |

Drones passed within 0.3 m horizontally of props and flew clean over them.

## Getting a person in

`ue_import_person.py` imports an FBX as a spawnable static mesh, imports the
textures beside it, builds a material and assigns it.

```bash
set AIRSIM_PROP_FBX=C:\path\to\model.fbx
UE4Editor-Cmd.exe YourProject.uproject -run=pythonscript -script="ue_import_person.py"
```

Run it with the **editor closed** — Unreal locks the project. Then restart the
editor, press Play, and confirm the name:

```bash
py probe_env.py --grep <asset name>
```

[RenderPeople's free models](https://renderpeople.com/free-3d-people/) work well
(FBX, free commercial use, no registration). **Get your own download** — this
repo ships the scripts, not the model.

### Why a posed scan rather than a rigged character

- **AirSim cannot spawn a SkeletalMesh at all.** The registry admits only static
  meshes and blueprints, so a rigged character never even appears in
  `simListAssets()`. A posed scan has no rig, so it spawns directly.
- **A rigged character carries its animation set's posture.** Unreal's stock
  mannequin animates on a rifle-carry rig, so *every* pose holds the arms up at
  chest height. A scan is simply the pose the person was photographed in.

`ue_make_human_bp.py` builds posed blueprints from a rigged character if you do
need that route — but expect the posture problem.

## Placing them

```bash
py maze_props.py --list     # catalogue with measured sizes
py maze_props.py --demo     # spawn a row of props to look at
py maze_props.py --clear    # remove everything it placed
```

`free_cell_positions(grid, grid_to_world, …)` takes your occupancy grid and
world-mapper **as arguments**, so the module never imports your flight code. To
wire it into a real run, call it after your maze is built and pass your drone
start cell as `skip` — otherwise a drone can spawn inside a couch.

The catalogue's `person` entry names a specific RenderPeople asset. `available()`
filters to whatever is actually in your registry, so a missing entry is skipped
rather than fatal — add your own model as a new entry with its measured size.

## Three ways this breaks, all guarded against

- **An unknown asset name crashes the simulator.** The spawn call looks a name up
  and dereferences the result before null-checking it — a typo is not a Python
  exception, it takes the sim down. Every asset is validated against
  `simListAssets()` first.
- **Reusing a just-destroyed object name is fatal.** Destruction is deferred;
  Unreal holds the name reserved until garbage collection, and spawning into a
  reserved name is a fatal engine error. Every spawned object gets a run-unique
  name.
- **Props share space with drones.** Anything taller than the lowest flight layer
  gets hit — and logged as a *wall* strike, quietly invalidating the very A/B
  test that proves props are inert. Props are auto-scaled to clear it.

## Measuring a new prop

There is no bounding-box call in this API, so `probe_env.py` sizes objects
optically: spawn at a known distance, measure the silhouette. Two details matter.

**Use segmentation frames, not scene frames.** Animated skies make two *idle*
scene frames differ by hundreds of pixels; segmentation frames are flat-shaded
IDs and measured **0 px** of drift.

**The silhouette is set by the near face**, half the object's width closer than
its centre. Measuring to the centre inflates the result by ~9%.

Also check the **pivot**: scan pivots are frequently not at the feet, so a prop
can hover. The catalogue carries a per-prop `z` offset, and it scales with the
prop — shrink the mesh and the pivot-to-feet gap shrinks with it.
