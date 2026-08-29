# swarmlink-capture-spawner

Two tools for an AirSim swarm simulation.

| tool | what it does | needs |
| --- | --- | --- |
| [`swarm_capture.py`](#swarm_capturepy--fpv-recording) | records each drone's first-person view to PNGs during a flight | Python + AirSim |
| [`maze_props.py`](#maze_propspy--placing-objects) + the `ue_*.py` scripts | spawns furniture and people into the world | Unreal Editor 4.27, editable project |

They are independent — use either on its own.

Both come from a larger swarm drone-racing project; that is what the
`MAZE_SWARM_AIRSIM_RT.py` and `ASTAR_DRONE` mentions in the source comments refer
to. Neither imports anything from it.

---

# `swarm_capture.py` — FPV recording

Writes one PNG per drone per capture round while a flight is running, plus a
`manifest.csv` describing each frame. Runs on its own threads and its own AirSim
connections, so it does not slow the flight-control loop.

Single file, stdlib-only at import time (`airsim` is imported lazily).

## Requirements

- Python 3, then `pip install -r requirements.txt`
- AirSim already running with vehicles spawned. It connects on port 41451 and
  never launches the simulator itself.
- For the built-in smoke test only: vehicles named `drone_1 … drone_N`. The
  `FPVRecorder` class takes whatever names you give it.

## Usage

Start AirSim, then run the smoke test. Nothing flies and no API control is taken:

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

Run this before a real flight to confirm your camera name resolves on every
vehicle. Flags: `--drones`, `--frames`, `--hz`, `--camera`, `--out`.

`--frames` is approximate — the smoke test records for a fixed duration with some
slack, so 5 rounds at 2 Hz gives 7–8 frames per drone, and the drones can differ
by one at the tail. Check `0 slots dropped, 0 errors` and an `OK` line per vehicle.

### In your own flight loop

```python
from swarm_capture import FPVRecorder

rec = FPVRecorder({0: "drone_1", 1: "drone_2"}, out_dir, t0=t0)
if rec.start():            # False = capture unavailable, flight continues
    ...fly...
rec.stop()                 # idempotent
print(rec.summary_line())
```

- `names` takes `{drone_id: "vehicle_name"}` or a plain sequence of names.
- `start()` blocks until every thread has connected and probed its camera (up to
  20 s), so a bad camera name is reported before the flight rather than once per
  drone per round throughout it. A vehicle whose camera does not resolve is
  dropped; the rest still record.
- `state_fn(drone_id) -> (mode, cell_x, cell_y)` is an optional callback for
  annotating frames. It runs on the capture threads, so keep it read-only and
  away from your flight client. Exceptions inside it are swallowed.
- `stop()` joins the threads and flushes the manifest. The threads are daemons,
  so a stalled RPC will not hold your process open — but still call `stop()`. If
  your `__main__` ends in `os._exit(0)`, daemons die without flushing.
- `stats()` returns `(captured, dropped, errors, mean_rpc_ms)`.

## Output

```
<out_dir>/
    drone_1/000000_t0.548.png
            000001_t1.053.png
    drone_2/000000_t0.548.png
    manifest.csv
```

All drones share one wall-clock slot schedule, so frame `000123` is the same
instant for each of them (within a few ms) and the folders line up for a
multi-view video. A drone that misses a slot skips that index instead of
renumbering.

`manifest.csv`, one row per frame:

| column | meaning |
| --- | --- |
| `t` | seconds since the caller's `t0`, also in the filename |
| `drone`, `vehicle` | simulation id and AirSim vehicle name |
| `frame`, `file` | slot index, and path relative to `out_dir` |
| `cam_x/y/z`, `q_w/x/y/z` | camera pose at render time, taken from the image response |
| `airsim_ts`, `rpc_ms` | AirSim's timestamp, and how long the grab took |
| `mode`, `cell_x`, `cell_y` | whatever `state_fn` returned, blank if unused |

Pass your flight loop's `t0` and the manifest joins your flight log on
`(t, drone)`.

## Performance notes

One thread and one client per drone, because `simGetImages` costs render-sync
latency per *call*. Measured on 4 stationary drones, 256x144:

| | |
| --- | --- |
| one call, 1 image | 335 ms |
| one call, 4 images (same vehicle) | 334 ms — batching is free |
| `compress=True` (43 KB PNG) vs `False` (110 KB raw) | no difference |
| 256x144 vs 320x240 | no difference — resolution is free |
| `getMultirotorState` / `simGetVehiclePose` | 2 ms |

`vehicle_name` is a per-call argument, so there is no batch form across vehicles:

| | |
| --- | --- |
| serial, 1 client, 4 drones/round | 1331 ms |
| parallel, 4 clients on 4 threads | 342 ms |

The cost is latency rather than throughput, so the drones wait concurrently and
the ceiling is roughly flat in drone count.

The absolute number varies with scene load. Re-measured on a lighter scene:

| configuration | mean grab | dropped | errors |
| --- | --- | --- | --- |
| 2 drones, `front_center` (256x144) | 183 ms | 0 | 0 |
| 2 drones, `fpv_cam` (320x240) | 82 ms | 0 | 0 |
| 4 drones, `front_center` | 93 ms | 0 | 0 |
| full 88 s flight, 2 drones | 87 ms | 0 | 0 |

Grab time varied 3–4x, but 4 drones still cost no more than 2. Size
`CAPTURE_HZ` against the 335 ms worst case (≈2.9 rounds/s, hence the 2.0
default) and use the `dropped` counter to find your actual headroom.

Capture does not disturb the flight loop: with four capture threads running, a
separate client polling `getMultirotorState` and `simGetVehiclePose` measured
median 1.00 ms / p90 1.01 / max 1.08, against a no-capture baseline of
1.00 / 1.01 / 3.01.

## Things to know

**Cameras are per vehicle**, declared in `Documents\AirSim\settings.json`. The
default `front_center` is one of five built-ins every multirotor gets, so it
resolves everywhere — including on vehicles added at runtime with
`simAddVehicle`. A custom name like `fpv_cam` only resolves on vehicles that
declare it. Resolution is free at grab time, so declaring a larger camera is
cheap.

**Clients are constructed inside the threads that use them.**
`msgpack-rpc-python` takes its tornado IOLoop from `IOLoop.current()`, which is
thread-local, so a client built on one thread and driven from another runs a
foreign event loop. `FPVRecorder` never accepts a client from the caller and
never touches yours.

**The camera pitches with the airframe.** Over one 88 s run, `|pitch|` was
median 2.8°, p90 10.1°, worst −24.9° / +31.7°; about 10% of frames tilt past
10°. A stabilised view would need `simSetCameraPose` counter-rotation per grab,
which is not implemented.

**If the smoke test says `camera 'front_center' resolved on none of [...]`**,
check the vehicle names before the camera name. That message appears whenever
the per-vehicle probe fails, and an unreachable vehicle is the more common cause.

---

# `maze_props.py` — placing objects

Spawns furniture and people into an AirSim world, into cells your occupancy grid
marks as free.

## Requirements

This half needs an Unreal project you can import assets into:

- Unreal Editor 4.27 with your AirSim environment as an editable project. A
  packaged AirSim binary will not work — its content is sealed in `.pak` files,
  and older builds do not implement `simListAssets` at all.
- Two editor plugins enabled in your `.uproject`: `PythonScriptPlugin` and
  `EditorScriptingUtilities`.
- Python + `airsim`, as above.

Check your simulator first:

```bash
py probe_env.py
```

If `simListAssets` answers, you are set. If it fails as an unknown method, you
are on an old packaged binary and cannot spawn imported assets into it.

## Usage

```bash
py maze_props.py --list     # catalogue with measured sizes
py maze_props.py --demo     # spawn a row of props to look at
py maze_props.py --clear    # remove everything it placed
```

`free_cell_positions(grid, grid_to_world, …)` takes your occupancy grid and
world-mapper as arguments, so the module does not import your flight code. To use
it in a real run, call it after your maze is built and pass your drone start cell
as `skip` — drones all launch from that cell, so a prop there means spawning one
inside furniture.

The catalogue's `person` entry names a specific RenderPeople asset. `available()`
filters to what is actually in your registry, so a missing entry is skipped
rather than fatal. Add your own models as new entries with their measured sizes.

## Props go in free cells, not wall cells

Sizes from the maze this came from:

| | footprint | height |
| --- | --- | --- |
| one wall block | 8.0 × 8.0 m | 5.1 m |
| drone flight layers | — | 1.8 – 3.3 m |
| a table | 1.12 m | 0.75 m |

A table is about 7x too short to work as a wall — drones fly over it. And the
planner would not notice, because obstacle avoidance reads the 0/1 occupancy
grid, not the spawned actors: a table in a wall cell leaves that cell logically
solid, so drones route around geometry that is not there.

`maze_props.py` therefore only places into free cells and never modifies the
grid. To confirm flight behaviour is unaffected, run the same seed with and
without props. Collision counts should be identical; coverage time varies a
little because the loop is real-time, so treat a fraction of a second as noise.

Measured on a 5x5 seed-3 maze, 2 drones, 14 props:

| | no props | 14 props |
| --- | --- | --- |
| coverage time | 89.0 s | 88.5 s |
| collisions | 0 | 0 |
| frames captured | 356 | 354 |

Drones passed within 0.3 m horizontally of props and flew over them.

## Importing a person

`ue_import_person.py` imports an FBX as a spawnable static mesh, imports the
textures beside it, builds a material and assigns it.

```bash
set AIRSIM_PROP_FBX=C:\path\to\model.fbx
UE4Editor-Cmd.exe YourProject.uproject -run=pythonscript -script="ue_import_person.py"
```

Run it with the editor closed — Unreal locks the project. Then restart the
editor, press Play, and check the name:

```bash
py probe_env.py --grep <asset name>
```

[RenderPeople's free models](https://renderpeople.com/free-3d-people/) work well
(FBX, free commercial use, no registration). Get your own download — this repo
has the scripts, not the model.

A posed scan is easier to use than a rigged character, for two reasons:

- AirSim cannot spawn a SkeletalMesh. The registry only holds static meshes and
  blueprints, so a rigged character never appears in `simListAssets()`. A posed
  scan has no rig and spawns directly.
- A rigged character carries its animation set's posture. Unreal's stock
  mannequin animates on a rifle-carry rig, so every pose holds the arms at chest
  height.

`ue_make_human_bp.py` builds posed blueprints from a rigged character if you need
that route.

## Failure modes to be aware of

- **An unknown asset name crashes the simulator.** The spawn call looks the name
  up and dereferences the result without a null check, so a typo takes the sim
  down rather than raising in Python. `maze_props.py` validates every asset
  against `simListAssets()` first.
- **Reusing a just-destroyed object name is a fatal engine error.** Destruction is
  deferred and Unreal keeps the name reserved until garbage collection. Every
  spawned object gets a run-unique name.
- **Props share space with drones**, since they go in free cells. Anything taller
  than the lowest flight layer will be hit, and gets logged as a wall strike.
  Props are auto-scaled to stay under it.

## Measuring a new prop

There is no bounding-box call in this API, so `probe_env.py` measures optically:
spawn at a known distance and measure the silhouette. Two things to watch:

- Use segmentation frames, not scene frames. Animated skies make two idle scene
  frames differ by hundreds of pixels; segmentation frames showed 0 px drift.
- The silhouette is set by the near face, half the object's width closer than its
  centre. Measuring to the centre overestimates by about 9%.

Also check the pivot — scan pivots are often not at the feet, so a prop can
hover. The catalogue carries a per-prop `z` offset, and it scales with the prop.
