# swarm-fpv-capture

Records the **first-person view of every drone in an AirSim swarm to disk while
the flight is happening**, one PNG per drone per capture round, without slowing
the flight-control loop down.

Single file, no package to install, stdlib-only at import time. Extracted from a
larger swarm drone-racing project, which is what the `MAZE_SWARM_AIRSIM_RT.py`
and `ASTAR_DRONE` references in the source comments refer to.

## Quickstart

```bash
pip install -r requirements.txt
```

Start AirSim, then run the following with the drones stationary. Nothing flies
and no API control is taken:

```bash
py swarm_capture.py --drones 4 --frames 5
```

```
FPV smoke test: 5 rounds at 2.0 Hz, camera 'front_center', vehicles ['drone_1', ...]
  -> .../results/frames/smoke_20260823_212715
  FPV capture: 20 frames (5/drone), 0 slots dropped, 0 errors, 212 ms mean grab
  OK   drone_1: 5 png (first 43897 bytes)
  OK   drone_2: 5 png (first 44112 bytes)
```

Run this smoke test before anything else: it confirms your camera name resolves
on every vehicle before you commit a real flight to it. Flags: `--drones`,
`--frames`, `--hz`, `--camera`, `--out`.

## Output layout

```
<out_dir>/
    drone_1/000000_t0.000.png
            000001_t0.500.png
    drone_2/000000_t0.001.png
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

This design follows from measurement rather than from anticipated need. On one
machine, 4 stationary drones, 256x144 Scene:

| | |
| --- | --- |
| one `simGetImages` call, 1 image | **335 ms** |
| one call, 4 images (same vehicle) | 334 ms, so batching is **free** |
| `compress=True` (43 KB PNG) vs `False` (110 KB raw) | no difference |
| 256x144 vs 320x240 | no difference, so **resolution is free** |
| plain `getMultirotorState` / `simGetVehiclePose` | 2 ms |

The ~335 ms is therefore a fixed per-**call** render-sync latency: not PNG
encoding, not resolution, not bytes on the wire. `vehicle_name` is a per-call
argument, so there is no batch form across vehicles and a serial loop pays that
latency once per drone:

| | |
| --- | --- |
| serial, 1 client, 4 drones/round | 1331 ms/round |
| parallel, 4 clients on 4 threads | **342 ms/round** |

The overlap is nearly perfect, because the cost is **latency, not throughput**:
the drones wait on the render thread concurrently. The practical ceiling is
therefore **~2.9 rounds/s regardless of drone count**, which is why `CAPTURE_HZ`
defaults to 2.0. Requesting a higher rate only increments the `dropped` counter.

**Treat 335 ms as a worst case, not a constant.** The same calls re-measured
against a lighter scene ran considerably faster, with no dropped frames and no
errors in any run:

| configuration | mean grab | dropped | errors |
| --- | --- | --- | --- |
| 2 drones, `front_center` (256x144) | 183 ms | 0 | 0 |
| 2 drones, `fpv_cam` (320x240) | 82 ms | 0 | 0 |
| 4 drones, `front_center` (256x144) | 93 ms | 0 | 0 |

Note that 4 drones cost no more than 2, which is the overlap described above
holding up. Size `CAPTURE_HZ` against the worst case, then let the `dropped`
counter in the wrap-up line tell you what your sim is actually doing.

**Capture does not interfere with the flight loop.** With all four capture
threads running continuously, a separate client polling `getMultirotorState` and
`simGetVehiclePose` measured median 1.00 ms / p90 1.01 / max 1.08, against a
no-capture baseline of median 1.00 / p90 1.01 / max 3.01. A 20 Hz control loop
has a 50 ms budget, and a 335 ms blocking call inside it would consume seven
whole passes. That is the reason capture runs on its own threads.

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
