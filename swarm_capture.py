"""
============================================================================
===[ FPV FRAME CAPTURE ]===
============================================================================
Records the FIRST-PERSON view of every drone in a swarm run to disk, one PNG
per drone per capture round, while the flight is happening.

WHY A THREAD (AND A CLIENT) PER DRONE
-------------------------------------
MAZE_SWARM_AIRSIM_RT.py is built around one hard rule: the AirSim msgpack-rpc
client is NOT thread-safe, so the whole body file uses exactly ONE client on
exactly ONE thread and carries zero locks (see its header + the AirSimSwarm
docstring; it is the same rule that makes ASTAR_DRONE's harness poll instead
of .join()).  This module therefore never touches the flight client — it builds
its own.  And it builds them INSIDE the capture threads, never handing one
across: msgpack-rpc-python derives its tornado IOLoop from IOLoop.current(),
which is THREAD-LOCAL, so a client constructed on the main thread and then
driven from another would be running a foreign thread's event loop.

The per-drone fan-out follows from measurement rather than from anticipated
need.  The numbers below are from this machine (2026-08-23, 4 parked
drones, 256x144 Scene):

    one simGetImages call, 1 image ................  335 ms
    one simGetImages call, 4 images (same vehicle).  334 ms   <- SAME
    compress=True (PNG 43 KB) vs False (raw 110 KB).  no difference
    256x144 (front_center) vs 320x240 (fpv_cam) ....  no difference
    plain getMultirotorState / simGetVehiclePose ...    2 ms

So the ~335 ms is a fixed per-CALL render-sync latency: it is not PNG
encoding, not resolution, and not bytes on the wire.  Since `vehicle_name` is
a per-call argument there is no batch form across vehicles, and a serial loop
pays that latency once per drone:

    serial,   1 client,  4 drones/round ............ 1331 ms/round
    parallel, 4 clients / 4 threads ................  342 ms/round

Nearly perfect overlap — the stall is latency, not throughput, so the drones
wait on the render thread concurrently.  That makes the practical ceiling
~1/0.34 = 2.9 rounds/s REGARDLESS of drone count, which is why CAPTURE_HZ
defaults to 2.0.  Requesting a higher rate only increments the dropped
counter.

Treat 335 ms as a WORST CASE rather than a constant.  Re-measured the same
day against a lighter scene: 183 ms (2 drones, front_center), 82 ms (2
drones, fpv_cam) and 93 ms (4 drones, front_center), every run with 0
dropped and 0 errors — so actual headroom is often several times better,
and 4 drones cost no more than 2 (which is the overlap above, holding).
Size CAPTURE_HZ against the worst case and let the dropped counter in the
wrap-up line tell you what the sim is really doing.

The 335 ms wait also does NOT leak into the flight loop.  Measured with all
four capture threads running flat out against a separate client polling
getMultirotorState + simGetVehiclePose the way run() does:

    baseline (no capture) ..... median 1.00 ms, p90 1.01, max 3.01
    capture flat-out x4 ....... median 1.00 ms, p90 1.01, max 1.08

i.e. no measurable effect.  The flight loop runs at CONTROL_DT = 0.05 s
(20 Hz); a 335 ms blocking call inside it would consume seven whole passes,
which is why capture runs on its own threads instead.

OUTPUT LAYOUT
-------------
    <out_dir>/
        drone_1/000000_t0.000.png
                000001_t0.500.png
        drone_2/000000_t0.001.png
        manifest.csv

Every drone targets the SAME wall-clock slot schedule, so frame 000123 is the
same instant for all of them (within a few ms) and the folders can be stacked
into a synchronized multi-view video.  A drone that misses a slot skips that
index rather than renumbering, so the indices never drift apart.

The `t` in every filename and manifest row is seconds since the caller's t0.
Pass run()'s own t0 and the manifest joins directly against that run's
results/traces/trace_rt_*.csv on (t, drone).

Stdlib only at import time (airsim is imported in start()), so importing this
module can never break --plan / --print-settings on a machine with no
simulator installed.
"""

import argparse
import csv
import os
import threading
import time

# ---- knobs -----------------------------------------------------------------
# Capture ROUNDS per second (one round = one frame from every drone, all
# grabbed concurrently).  The measured per-call render-sync floor is ~335 ms,
# so ~2.9 Hz is the conservative ceiling no matter how many drones there are;
# 2.0 Hz leaves headroom for a loaded sim.  A lighter scene has measured 3-4x
# faster than that floor, so raising this with --capture-hz is often fine —
# watch the dropped counter in the wrap-up line to find out.
CAPTURE_HZ = 2.0

# The DEFAULT camera is a built-in one on purpose.  Cameras in AirSim are
# declared PER VEHICLE, so a custom name resolves ONLY on the vehicles whose
# settings.json actually declares it, and the set of vehicles is not even
# fixed: ensure_vehicles() can simAddVehicle drones at runtime that were
# never in settings.json at all.  "front_center" is one of the five cameras
# every multirotor receives automatically, so it resolves on all of them (at
# AirSim's default 256x144), which makes it the only safe default.
#
# Since resolution costs nothing at grab time, declaring a camera on every
# vehicle in settings.json is the least expensive way to obtain a
# higher-resolution image — then select it with --capture-camera.  The probe
# in _run_one reports per vehicle, so a name that resolves on only some of
# them records those and drops the rest rather than failing the run.
CAPTURE_CAMERA = "front_center"

# The five built-in multirotor cameras, with their legacy numeric ids.  Printed
# as a hint when the requested camera name does not resolve.
BUILTIN_CAMERAS = (("front_center", "0"), ("front_right", "1"),
                   ("front_left", "2"), ("bottom_center", "3"),
                   ("back_center", "4"))

MANIFEST_FLUSH_S = 1.0      # s — same instrumentation cadence the flight
                            # loop's trace CSV uses
PROBE_TIMEOUT_S = 20.0      # s — how long start() waits for the threads to
                            # connect + probe cameras before giving up on a
                            # synchronous verdict


class FPVRecorder:
    """Background first-person-view recorder for a swarm of AirSim vehicles.

    Usage (the caller owns start/stop; stop() is idempotent):

        rec = FPVRecorder(swarm.names, out_dir, t0=t0)
        if rec.start():                 # False = capture unavailable
            ...fly...
        rec.stop()

    `names` is the body's id -> vehicle-name mapping ({0: "drone_1", ...}), so
    the manifest can log the SIMULATION drone id while the folders carry the
    AirSim vehicle name that every other log in the run uses.
    """

    def __init__(self, names, out_dir, t0=None, hz=CAPTURE_HZ,
                 camera=CAPTURE_CAMERA, state_fn=None, quiet=False):
        # dict {drone_id: "drone_N"}; also accepts a bare sequence of names
        if isinstance(names, dict):
            self.names = dict(names)
        else:
            self.names = {i: n for i, n in enumerate(names)}
        self.out_dir = out_dir
        self.t0 = time.time() if t0 is None else t0
        self.hz = float(hz)
        self.period = 1.0 / self.hz if self.hz > 0 else 1.0
        self.camera = str(camera)
        # Optional main-thread state accessor: state_fn(drone_id) -> (mode, cx,
        # cy).  Called from the capture threads without a lock deliberately — a
        # Python attribute read is atomic under the GIL, and a value that is one
        # 50 ms flight pass stale is entirely adequate as a manifest annotation.
        # It must never mutate anything.
        self.state_fn = state_fn
        self.quiet = quiet

        self._stop = threading.Event()    # tells every capture thread to finish
        self._go = threading.Event()      # released once the output tree exists
        self._threads = []
        self._probe_done = {}             # id -> Event, set after connect+probe
        self._probe_ok = {}               # id -> True/False
        self._lock = threading.Lock()     # guards the manifest + the counters
        self._man_f = None
        self._man = None
        self._man_lastflush = 0.0
        self._airsim = None

        self.enabled = False              # probe succeeded, frames are flowing
        self.error = None                 # human-readable reason if not
        self.captured = 0                 # frames actually written
        self.dropped = 0                  # capture slots skipped (fell behind)
        self.errors = 0                   # RPCs that raised or came back empty
        self._rpc_ms = 0.0                # running sum for the mean
        self._rpc_n = 0

    # -- lifecycle ------------------------------------------------------------
    def start(self):
        """Spawn one capture thread per drone and BLOCK until every one has
        connected and probed its camera, so a bad camera name is reported before
        the flight starts instead of throwing once per drone per round for the
        whole run.

        Returns True if at least one drone is capturing.  The threads are
        daemons so a stalled image RPC can never hold the process open — but the
        caller must still stop() them, because MAZE_SWARM_AIRSIM_RT's __main__
        ends with os._exit(0), which terminates daemon threads immediately
        with no flush."""
        if self._threads:
            return self.enabled

        # Import once here rather than having four threads import concurrently.
        # Failing in the caller's thread also gives a clean error on a machine
        # with no airsim installed.
        try:
            import airsim
        except Exception as exc:                     # noqa: BLE001
            self.error = f"import airsim failed ({exc})"
            if not self.quiet:
                print(f"  FPV capture DISABLED: {self.error}")
            return False
        self._airsim = airsim

        for did in sorted(self.names):
            self._probe_done[did] = threading.Event()
            self._probe_ok[did] = False
            th = threading.Thread(target=self._run_one, args=(did,),
                                  name=f"fpv-{self.names[did]}", daemon=True)
            th.start()
            self._threads.append(th)

        # Wait for all probes (bounded), then keep only the drones that answered.
        deadline = time.time() + PROBE_TIMEOUT_S
        for did in sorted(self.names):
            self._probe_done[did].wait(timeout=max(0.0, deadline - time.time()))
        good = {d: n for d, n in self.names.items() if self._probe_ok.get(d)}
        if not good:
            self.error = (f"camera '{self.camera}' resolved on none of "
                          f"{sorted(self.names.values())}")
            self._go.set()                           # let the threads exit
            self.stop(timeout=2.0)
            if not self.quiet:
                print(f"  FPV capture DISABLED: {self.error}")
            return False
        if len(good) < len(self.names) and not self.quiet:
            missing = sorted(set(self.names.values()) - set(good.values()))
            print(f"  FPV capture: recording {len(good)} of "
                  f"{len(self.names)} drones (no camera on {missing})")

        # Build the output tree only now that we know something can be captured.
        try:
            for name in good.values():
                os.makedirs(os.path.join(self.out_dir, name), exist_ok=True)
            self._man_f = open(os.path.join(self.out_dir, "manifest.csv"),
                               "w", newline="")
            self._man = csv.writer(self._man_f)
            self._man.writerow(["t", "drone", "vehicle", "frame", "file",
                                "cam_x", "cam_y", "cam_z",
                                "q_w", "q_x", "q_y", "q_z",
                                "airsim_ts", "rpc_ms",
                                "mode", "cell_x", "cell_y"])
        except OSError as exc:
            self.error = f"cannot write to {self.out_dir} ({exc})"
            self._go.set()
            self.stop(timeout=2.0)
            if not self.quiet:
                print(f"  FPV capture DISABLED: {self.error}")
            return False

        # ONE slot clock shared by every thread, so frame N is the same instant
        # for all drones without any inter-thread coupling (a barrier would make
        # the slowest drone throttle the rest).
        self._slot_t0 = time.time()
        self.enabled = True
        self._go.set()
        return True

    def stop(self, timeout=5.0):
        """Signal the threads, join them, and make sure the manifest is on disk.
        Safe to call twice (run()'s wrap-up and shutdown()'s redundant safety
        call both invoke it)."""
        if not self._threads:
            return
        self._stop.set()
        self._go.set()                    # unblock any thread still pre-launch
        deadline = time.time() + timeout
        for th in self._threads:
            th.join(timeout=max(0.0, deadline - time.time()))
        alive = [t.name for t in self._threads if t.is_alive()]
        self._threads = []
        if alive:
            # Daemon threads stuck in an RPC.  Their last rows may be unflushed;
            # everything older than MANIFEST_FLUSH_S is already on disk.  Do not
            # close the manifest out from under a live writer.
            print(f"  FPV capture: {len(alive)} thread(s) still busy after "
                  f"{timeout:.0f} s ({', '.join(alive)}) — the tail of the "
                  f"manifest may be short")
            return
        with self._lock:
            if self._man_f is not None:
                try:
                    self._man_f.close()
                except OSError:
                    pass
                self._man_f = None

    # -- reporting ------------------------------------------------------------
    def stats(self):
        """(captured, dropped, errors, mean_rpc_ms) for the run's wrap-up."""
        with self._lock:
            mean_ms = (self._rpc_ms / self._rpc_n) if self._rpc_n else 0.0
            return self.captured, self.dropped, self.errors, mean_ms

    def summary_line(self):
        cap, drop, err, ms = self.stats()
        n = max(1, len(self.names))
        line = (f"  FPV capture: {cap} frames ({cap // n}/drone), "
                f"{drop} slots dropped, {err} errors, "
                f"{ms:.0f} ms mean grab -> {self.out_dir}")
        if drop > cap * 0.25:
            # ~335 ms per grab is the render-sync floor; requesting more rounds
            # per second than that only increments this counter.
            line += (f"\n    (heavy dropping at {self.hz:g} Hz — the measured "
                     f"ceiling is ~{1.0 / 0.34:.1f} Hz; lower --capture-hz)")
        return line

    # -- one capture thread, one drone, one client ----------------------------
    def _run_one(self, did):
        airsim = self._airsim
        name = self.names[did]

        try:
            client = airsim.MultirotorClient()       # OUR socket, OUR ioloop
            client.ping()
        except Exception as exc:                     # noqa: BLE001
            print(f"  FPV capture: {name} cannot reach AirSim ({exc})")
            self._probe_done[did].set()
            return

        # Camera probe: cheaper than an image and gives a clear per-vehicle
        # result.  A vehicle whose camera does not resolve is dropped rather
        # than failing the whole recorder, so a partial configuration still
        # produces usable data.
        try:
            client.simGetCameraInfo(self.camera, vehicle_name=name)
            self._probe_ok[did] = True
        except Exception as exc:                     # noqa: BLE001
            hint = ", ".join(f"{n} ({i})" for n, i in BUILTIN_CAMERAS)
            print(f"  FPV capture: camera '{self.camera}' not available on "
                  f"{name} ({exc}); built-ins are {hint}")
        finally:
            self._probe_done[did].set()

        self._go.wait()
        if self._stop.is_set() or not self._probe_ok[did] or not self.enabled:
            return

        # One request object, reused for every grab.  All four ImageRequest
        # arguments are passed EXPLICITLY because the class attribute default is
        # compress=False while the constructor default is compress=True, which
        # is an easy discrepancy to overlook.  compress=True means AirSim
        # returns finished PNG bytes, so writing a frame is a raw file write:
        # no opencv, and no numpy reshape (airsim's own string_to_uint8_array
        # still calls np.fromstring, removed in numpy >= 1.23).  It also
        # measured identical to compress=False, so the PNG costs nothing.
        request = [airsim.ImageRequest(self.camera, airsim.ImageType.Scene,
                                       False, True)]

        k = 0                                        # slot index == frame index
        while not self._stop.is_set():
            target = self._slot_t0 + k * self.period
            now = time.time()
            if now < target:
                # Wake early if stop() fires; 50 ms slices keep shutdown
                # responsive without busy-waiting.
                self._stop.wait(min(target - now, 0.05))
                continue
            if now > target + self.period:
                # Fell behind (the grab itself is ~335 ms, so this is normal at
                # any hz near the ceiling).  SKIP the missed slots outright and
                # advance to the current one — never attempt to catch up, which
                # would issue a burst of image RPCs precisely when the sim is
                # already under load, and would desynchronize this drone's frame
                # indices from the rest of the swarm.
                missed = int((now - target) / self.period)
                with self._lock:
                    self.dropped += missed
                k += missed
                continue
            self._grab(airsim, client, request, did, name, k)
            k += 1

    def _grab(self, airsim, client, request, did, name, frame):
        t = time.time() - self.t0
        rpc_t0 = time.time()
        try:
            resp = client.simGetImages(request, vehicle_name=name)
        except Exception as exc:                     # noqa: BLE001
            with self._lock:
                self.errors += 1
                n = self.errors
            if n <= 3:
                print(f"  FPV capture: simGetImages failed for {name} ({exc})")
            return
        rpc_ms = (time.time() - rpc_t0) * 1000.0

        r = resp[0] if resp else None
        data = r.image_data_uint8 if r is not None else None
        if not data or r.width == 0:
            # AirSim occasionally returns an empty frame when the render target
            # is not ready yet (right after takeoff, or under load).
            with self._lock:
                self.errors += 1
            return
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)

        fname = f"{frame:06d}_t{t:.3f}.png"
        try:
            airsim.write_file(os.path.join(self.out_dir, name, fname), data)
        except OSError as exc:
            with self._lock:
                self.errors += 1
                n = self.errors
            if n <= 3:
                print(f"  FPV capture: cannot write {fname} ({exc})")
            return

        # Pose is returned INSIDE the image response — logging it costs no
        # extra RPC, and it is the camera's own pose at render time, which
        # describes the pixels more accurately than a separately-polled body
        # pose.
        p, q = r.camera_position, r.camera_orientation
        mode = cx = cy = ""
        if self.state_fn is not None:
            try:
                st = self.state_fn(did)
                if st:
                    mode, cx, cy = st
            except Exception:                        # noqa: BLE001
                pass                                 # annotation only, never fatal

        with self._lock:
            self.captured += 1
            self._rpc_ms += rpc_ms
            self._rpc_n += 1
            if self._man is None:
                return
            self._man.writerow([f"{t:.3f}", did, name, frame,
                                f"{name}/{fname}",
                                f"{p.x_val:.3f}", f"{p.y_val:.3f}",
                                f"{p.z_val:.3f}",
                                f"{q.w_val:.5f}", f"{q.x_val:.5f}",
                                f"{q.y_val:.5f}", f"{q.z_val:.5f}",
                                r.time_stamp, f"{rpc_ms:.1f}",
                                mode, cx, cy])
            if time.time() - self._man_lastflush >= MANIFEST_FLUSH_S:
                self._man_lastflush = time.time()
                self._man_f.flush()


# ============================================================================
# ===[ STANDALONE SMOKE TEST ]===
# ============================================================================
# Validates the camera configuration against a running sim WITHOUT flying:
#     py swarm_capture.py --drones 4 --frames 5
# Use it to confirm a camera name resolves on every vehicle before committing
# a flight to it.
def _smoke(argv=None):
    ap = argparse.ArgumentParser(
        description="Grab a few FPV frames from parked AirSim drones to check "
                    "the camera configuration (nothing flies and no API "
                    "control is taken).")
    ap.add_argument("--drones", type=int, default=4,
                    help="capture drone_1 .. drone_N (default 4)")
    ap.add_argument("--frames", type=int, default=5,
                    help="number of capture ROUNDS (each round = one frame "
                         "from every drone)")
    ap.add_argument("--hz", type=float, default=CAPTURE_HZ,
                    help=f"capture rounds per second (default {CAPTURE_HZ}; "
                         f"the render-sync ceiling is ~2.9)")
    ap.add_argument("--camera", default=CAPTURE_CAMERA,
                    help=f"camera name (default {CAPTURE_CAMERA}; try fpv_cam "
                         f"if settings.json declares one)")
    ap.add_argument("--out", default=None,
                    help="output directory (default results/frames/smoke_<stamp>)")
    args = ap.parse_args(argv)

    import datetime
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "results", "frames", f"smoke_{stamp}")

    names = {i: f"drone_{i + 1}" for i in range(args.drones)}
    print(f"FPV smoke test: {args.frames} rounds at {args.hz} Hz, "
          f"camera '{args.camera}', vehicles {sorted(names.values())}")
    print(f"  -> {out}")

    rec = FPVRecorder(names, out, hz=args.hz, camera=args.camera)
    if not rec.start():
        print("FAILED — capture never came up.")
        return 1
    time.sleep(args.frames / max(args.hz, 0.001) + 1.0)
    rec.stop()

    print(rec.summary_line())
    ok = True
    for name in sorted(rec.names.values()):
        d = os.path.join(out, name)
        pngs = sorted(f for f in os.listdir(d)) if os.path.isdir(d) else []
        pngs = [f for f in pngs if f.endswith(".png")]
        size = os.path.getsize(os.path.join(d, pngs[0])) if pngs else 0
        print(f"  {'OK  ' if pngs else 'FAIL'} {name}: {len(pngs)} png "
              f"(first {size} bytes)")
        ok = ok and bool(pngs)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_smoke())
