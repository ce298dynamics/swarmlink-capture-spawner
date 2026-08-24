# swarm-fpv-capture

Records the **first-person view of every drone in an AirSim swarm to disk while
the flight is happening**, one PNG per drone per capture round, without slowing
the flight-control loop down.

Single file, no package to install, stdlib-only at import time. Extracted from a
larger swarm drone-racing project — that is what the `MAZE_SWARM_AIRSIM_RT.py`
and `ASTAR_DRONE` references in the source comments refer to.

## Quickstart

```bash
pip install -r requirements.txt
```

Start AirSim, then — with the drones just sitting there, no flying, no API
control taken:

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

That smoke test is the thing to run first: it confirms your camera name
resolves on every vehicle before you commit a real flight to it. Flags:
`--drones`, `--frames`, `--hz`, `--camera`, `--out`.

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
| `t` | seconds since the caller's `t0` — also in the filename |
| `drone`, `vehicle` | simulation id and AirSim vehicle name |
| `frame`, `file` | slot index and path relative to `out_dir` |
| `cam_x/y/z`, `q_w/x/y/z` | camera pose **at render time**, straight out of the image response — costs no extra RPC and labels the pixels more truthfully than a separately-polled body pose |
| `airsim_ts`, `rpc_ms` | AirSim's own timestamp, and how long the grab took |
| `mode`, `cell_x`, `cell_y` | whatever your `state_fn` returned (blank if unused) |

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

- `names` takes `{drone_id: "vehicle_name"}` **or** a bare sequence of names.
- `start()` blocks until every thread has connected and probed its camera (up to
  20 s), so a bad camera name is reported *before* the flight instead of
  throwing once per drone per round for the whole run. A vehicle whose camera
  doesn't resolve is dropped and the rest still record — 3 of 4 beats none.
- `state_fn(drone_id) -> (mode, cell_x, cell_y)` is an optional annotation hook
  so a frame can be read without cross-referencing anything. It is called **from
  the capture threads**, so it must be read-only and must not touch your flight
  client. Exceptions from it are swallowed — annotation is never fatal.
- `stop()` joins the threads and flushes the manifest. The threads are daemons
  so a wedged image RPC can't hold your process open, but you must still call
  `stop()` — if your `__main__` ends in `os._exit(0)`, daemons die with no flush.
- `stats()` returns `(captured, dropped, errors, mean_rpc_ms)`.

## Why a thread and a client per drone

Not premature optimization — forced by measurement. On one machine, 4 parked
drones, 256x144 Scene:

| | |
| --- | --- |
| one `simGetImages` call, 1 image | **335 ms** |
| one call, 4 images (same vehicle) | 334 ms — batching is **free** |
| `compress=True` (43 KB PNG) vs `False` (110 KB raw) | no difference |
| 256x144 vs 320x240 | no difference — **resolution is free** |
| plain `getMultirotorState` / `simGetVehiclePose` | 2 ms |

So the ~335 ms is a fixed per-**call** render-sync latency: not PNG encoding,
not resolution, not bytes on the wire. `vehicle_name` is a per-call argument, so
there is no batch form across vehicles and a serial loop pays it once per drone:

| | |
| --- | --- |
| serial, 1 client, 4 drones/round | 1331 ms/round |
| parallel, 4 clients on 4 threads | **342 ms/round** |

Nearly perfect overlap — the stall is **latency, not throughput**, so the drones
wait on the render thread concurrently. Practical ceiling is therefore
**~2.9 rounds/s regardless of drone count**, which is why `CAPTURE_HZ` defaults
to a comfortable 2.0. Asking for more just fills the `dropped` counter.

**It does not leak into the flight loop.** With all four capture threads running
flat out, a separate client polling `getMultirotorState` + `simGetVehiclePose`
measured median 1.00 ms / p90 1.01 / max 1.08 — against a no-capture baseline of
median 1.00 / p90 1.01 / max 3.01. A 20 Hz control loop has a 50 ms budget; a
335 ms blocking call inside it would eat seven whole passes, which is exactly
why capture lives out here.

## Two things that will bite you first

**Cameras are declared per vehicle** in `Documents\AirSim\settings.json`. The
default is `front_center` on purpose: it's one of the five built-ins
(`front_center` 0, `front_right` 1, `front_left` 2, `bottom_center` 3,
`back_center` 4) that every multirotor gets for free, so it resolves everywhere,
including on vehicles added at runtime via `simAddVehicle` that were never in
`settings.json` at all. A custom name like `fpv_cam` only resolves on vehicles
that declare one. Since **resolution costs nothing at grab time**, declaring a
bigger camera on every vehicle is the cheap way to a better picture.

**The client must be built inside the thread that uses it.** `msgpack-rpc-python`
derives its tornado IOLoop from `IOLoop.current()`, which is **thread-local** —
a client constructed on the main thread and then driven from another is running
a foreign thread's event loop. `FPVRecorder` therefore builds its own clients
inside its own threads and never accepts one from the caller. It never touches
your flight client either, so this composes with a single-threaded,
lock-free flight loop.

## Known, not a bug

A body-fixed camera **pitches with the airframe**. Over one 88 s run: `|pitch|`
median 2.8°, p90 10.1°, worst −24.9° / +31.7°. Roughly 10% of frames tilt past
10°, and some point at open sky during a hard brake. That is honest FPV. A
stabilized view would need `simSetCameraPose` counter-rotation per grab, which
is not implemented here.
