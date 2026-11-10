#!/usr/bin/env python3
"""Live Jetson GMSL + FORSENSE IMU tracking on a shared RAW clock."""
import argparse
import collections
import contextlib
import fcntl
import json
import math
import os
from pathlib import Path
import platform
import select
import signal
import stat
import struct
import subprocess
import sys
import termios
import threading
import time

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cam_imu_motion"))
import imu
import timing_core

NS = 1_000_000_000
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1440
PIXELS = IMAGE_WIDTH * IMAGE_HEIGHT
WARMUP_US = 10_000_000
CLOCK_BLOCK = 500  # About one second at the configured IMU rate.
CAMERA_ANCHOR_NS = NS // 4
MAX_CAMERA_ANCHORS = 32
MAX_CAMERA_GAP_NS = 950_000_000
NORMAL_IMU_SAMPLES = 2500  # 5 s of 500-Hz IMU; startup history above this can be removed.
MAX_RECOVERY_IMU_SAMPLES = 4000


def mapped_tick(tick, model):
    elapsed = (tick - model["device_reference_us"]) * 1000
    return model["host_reference_raw_ns"] + round(
        model["host_relative_intercept_ns"] + elapsed * model["host_ns_per_device_ns"])


def check_clock_fit(model):
    q = model["receipt_residual_ms"]
    if (abs(model["apparent_rate_difference_ppm_over_trial"]) > 500 or
            q["p95"] - q["p05"] > 6 or max(abs(q["min"]), abs(q["max"])) > 50):
        raise RuntimeError(f"Unstable IMU timer-to-RAW fit: {model}")


class RollingClock:
    """Keep the sample clock continuous while following gradual oscillator drift."""
    def __init__(self, initial, warm):
        self.mapping = initial
        self.anchors = collections.deque(maxlen=30)
        self.residual_ms = []
        self.block_start = None
        self.last_update = None
        self.refits = 0
        self.max_median_ms = 0.
        # Seed the rolling fit with the same clean receipts as the startup fit.
        for i in range(0, len(warm) - CLOCK_BLOCK + 1, CLOCK_BLOCK):
            group = warm[i:i+CLOCK_BLOCK]
            residuals = [(h - mapped_tick(t, initial)) / 1e6 for t, h, _ in group]
            self.add_anchor(group[0][0], group[-1][0], timing_core.distribution(residuals)["median"])

    def add_anchor(self, first_tick, last_tick, median_ms):
        center = (first_tick + last_tick) // 2
        self.anchors.append((center, mapped_tick(center, self.mapping) + round(median_ms * 1e6)))

    def stamp(self, tick, receipt):
        stamp = mapped_tick(tick, self.mapping)
        residual = (receipt - stamp) / 1e6
        if abs(residual) > 50:
            raise RuntimeError(f"IMU receipt differs from its clock mapping by {residual:.1f} ms.")
        if not self.residual_ms:
            self.block_start = tick
        self.residual_ms.append(residual)
        if len(self.residual_ms) < CLOCK_BLOCK:
            return stamp
        q = timing_core.distribution(self.residual_ms)
        self.residual_ms.clear()
        self.max_median_ms = max(self.max_median_ms, abs(q["median"]))
        if abs(q["median"]) > 3 or q["p95"] - q["p05"] > 10:
            raise RuntimeError(f"Unstable live IMU-to-RAW receipt residuals: {q}")
        self.add_anchor(self.block_start, tick, q["median"])
        if len(self.anchors) >= 10:
            ticks, raw = zip(*self.anchors)
            fit = timing_core.imu_mapping(ticks, raw)
            check_clock_fit(fit)
            # Change only the rate for FUTURE samples. Converge to the rolling fit's
            # phase over 10 s, capped at 100 ppm, without ever stepping an emitted stamp.
            phase_ns = mapped_tick(tick, fit) - stamp
            slew = max(-0.0001, min(0.0001, phase_ns / (10 * NS)))
            self.mapping = {"device_reference_us": tick, "host_reference_raw_ns": stamp,
                            "host_relative_intercept_ns": 0, "host_ns_per_device_ns":
                                fit["host_ns_per_device_ns"] + slew}
            self.last_update = {"mapping": self.mapping, "target_phase_error_ns": phase_ns,
                                "window_anchors": len(ticks), "window_device_us": ticks[-1] - ticks[0]}
            self.refits += 1
        return stamp


class CaptureStopped(Exception):
    """The operator stopped between complete camera packets."""


class Packets:
    """Allow a partial packet on opening the port. Require continuous CRC-valid AHRS thereafter."""
    def __init__(self):
        self.buf = bytearray()
        self.last_timer = None
        self.tick = 0

    def feed(self, data):
        self.buf.extend(data)
        if self.last_timer is None:
            pos = imu.first_valid_sample(self.buf)
            if pos is None:
                del self.buf[:max(0, len(self.buf) - imu.FRAME_BYTES + 1)]
                return
            del self.buf[:pos]
        while len(self.buf) >= imu.FRAME_BYTES:
            frame = bytes(self.buf[:imu.FRAME_BYTES])
            del self.buf[:imu.FRAME_BYTES]
            if (not frame.startswith(imu.AHRS_HEADER) or
                    imu.crc32_forsense(frame[:-4]) != struct.unpack_from("<I", frame, 50)[0]):
                raise ValueError("AHRS framing or CRC changed during streaming.")
            timer, *fields = struct.unpack_from("<I10f", frame, 6)
            values = (*[v * 9.80665 for v in fields[3:6]],
                      *[math.radians(v) for v in fields[6:9]])
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Nonfinite accelerometer/gyro measurement.")
            if self.last_timer is None:
                self.tick = timer
            else:
                dt = (timer - self.last_timer) & 0xFFFFFFFF
                if not 1500 <= dt <= 2500:
                    raise ValueError(f"IMU timer discontinuity: {dt} us (expected about 2000).")
                self.tick += dt
            self.last_timer = timer
            yield self.tick, values


class Streams:
    def __init__(self):
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.error = None
        self.imu = collections.deque()
        self.first_sent = False  # True once the first real image has been paired, before its pipe write completes.
        self.latest = None
        # Keep a sparse timeline of genuine images if the tracker briefly stops
        # reading. The latest-only path is unchanged while it keeps up.
        self.camera_anchors = collections.deque()
        self.camera_shift_ns = None
        self.last_selected_ns = None
        self.pending_camera_ns = None
        self.last_camera_read_ns = None
        self.last_camera_read_mono = None
        self.model = None
        self.clock_update = None
        self.finish_requested = False
        self.phase = ("startup", time.monotonic())
        self.transfer = None
        self.last_sent_ns = None
        self.failure_context = None
        self.stats = dict(frames_read=0, frames_superseded=0, frames_sent=0, imu_read=0,
                          imu_sent=0, max_frame_interval_s=0.0, max_clock_median_ms=0.0,
                          max_camera_anchors=0, catchup_frames=0, max_imu_queue=0)

    def remember_camera(self, raw_ns, pixels):
        """Called with the condition held; an anchor is never replaced by a duplicate."""
        self.last_camera_read_ns = raw_ns
        self.last_camera_read_mono = time.monotonic()
        self.stats["frames_read"] += 1
        if self.latest is not None:
            if self.camera_anchors:
                reference = self.camera_anchors[-1][0] + self.camera_shift_ns
            else:
                reference = self.pending_camera_ns or self.last_selected_ns
            if reference is not None and self.latest[0] + self.camera_shift_ns - reference >= CAMERA_ANCHOR_NS:
                if len(self.camera_anchors) >= MAX_CAMERA_ANCHORS:
                    raise RuntimeError(f"Reached {MAX_CAMERA_ANCHORS} buffered camera anchors; "
                                       f"IMU backlog={len(self.imu)}. Recovery window exhausted.")
                self.camera_anchors.append(self.latest)
                self.stats["max_camera_anchors"] = max(self.stats["max_camera_anchors"], len(self.camera_anchors))
            else:
                self.stats["frames_superseded"] += 1
        self.latest = (raw_ns, pixels)

    def check_imu_backlog(self):
        """Keep every inter-image IMU after startup. Extend capacity only for a recoverable camera timeline."""
        count = len(self.imu)
        self.stats["max_imu_queue"] = max(self.stats["max_imu_queue"], count)
        if count <= NORMAL_IMU_SAMPLES:
            return
        if not self.first_sent:
            # Startup pre-roll only. No tracking interval has started yet.
            while len(self.imu) > NORMAL_IMU_SAMPLES:
                self.imu.popleft()
            return
        if count > MAX_RECOVERY_IMU_SAMPLES:
            raise RuntimeError(f"IMU backlog reached {count} samples (limit {MAX_RECOVERY_IMU_SAMPLES}); "
                               "recovery window exhausted.")

        times = []
        if self.pending_camera_ns is not None:
            times.append(self.pending_camera_ns)
        times.extend(ns + self.camera_shift_ns for ns, _ in self.camera_anchors)
        if self.latest is not None:
            times.append(self.latest[0] + self.camera_shift_ns)
        previous = self.last_selected_ns
        for stamp in times:
            if not 0 < stamp - previous < MAX_CAMERA_GAP_NS:
                raise RuntimeError(f"IMU backlog={count}; retained camera gap {(stamp-previous)/1e9:.3f} s cannot be bridged.")
            previous = stamp
        imu_minus_camera = (self.imu[-1][0] - previous) / NS
        if not times or abs(imu_minus_camera) >= MAX_CAMERA_GAP_NS / NS:
            raise RuntimeError(f"IMU backlog={count}; retained images do not reach current IMU time "
                               f"(IMU minus last camera {imu_minus_camera:.3f} s).")

    def mark(self, stage):
        self.phase = (stage, time.monotonic())

    def snapshot(self):
        """Capture bridge state before teardown. Monotonic ages are in seconds."""
        with self.condition:
            now = time.monotonic()
            phase, started = self.phase
            read_at = self.last_camera_read_mono
            transfer = self.transfer
            return {
                "phase": phase, "phase_elapsed_s": round(now - started, 3),
                "queued_imu": len(self.imu), "imu_first_ns": self.imu[0][0] if self.imu else None,
                "imu_last_ns": self.imu[-1][0] if self.imu else None,
                "retained_intermediate_images": len(self.camera_anchors),
                "last_camera_read_raw_ns": self.last_camera_read_ns,
                "last_camera_read_age_s": round(now - read_at, 3) if read_at is not None else None,
                "last_selected_camera_on_imu_ns": self.last_selected_ns,
                "pending_camera_on_imu_ns": self.pending_camera_ns,
                "last_fully_sent_camera_on_imu_ns": self.last_sent_ns,
                "transfer": dict(transfer) if transfer is not None else None,
                "streams": dict(self.stats),
            }

    def worker(self, fn, *args):
        try:
            fn(*args)
        except Exception as exc:
            with self.condition:
                if not self.stop.is_set() and self.error is None:
                    self.error = f"{fn.__name__}: {exc}"
                    self.failure_context = self.snapshot()
                self.condition.notify_all()

    def check(self):
        if self.error:
            raise RuntimeError(self.error)

    def wait(self, predicate, seconds, message):
        if not self.condition.wait_for(lambda: self.finish_requested or self.error or predicate(), timeout=seconds):
            raise TimeoutError(message)
        if self.finish_requested:
            raise CaptureStopped
        self.check()


@contextlib.contextmanager
def serial_port():
    path = Path(imu.DEVICE).resolve(strict=True)
    info = path.stat()
    if not stat.S_ISCHR(info.st_mode):
        raise ValueError(f"Expected a character device at {path}")
    imu.check_owners(path, info, lambda msg: print(msg, flush=True))
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        try:
            original = termios.tcgetattr(fd)
            try:
                desired = imu.host_settings(original, 921600)
                termios.tcsetattr(fd, termios.TCSANOW, desired)
                if termios.tcgetattr(fd)[4:6] != desired[4:6]:
                    raise RuntimeError("Cannot set host serial port to 921600 baud.")
                yield fd
            finally:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            fcntl.ioctl(fd, termios.TIOCNXCL)
    finally:
        os.close(fd)


def read_imu(s):
    parser = Packets()
    warm = []
    clock = None
    with serial_port() as fd:
        last_packet = time.monotonic()
        # Drain stale input after applying host termios before locking to a CRC-valid frame.
        # The clock fit and subsequent stream still require contiguous valid packets.
        discard_before = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) + NS // 2
        while not s.stop.is_set():
            if time.monotonic() - last_packet > 2:
                raise TimeoutError("No CRC-valid IMU packet in 2 s at 921600 baud.")
            if not select.select([fd], [], [], .15)[0]:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            receipt = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            if not data:
                raise OSError("Serial device disconnected.")
            if receipt < discard_before:
                continue
            batch = []
            for tick, values in parser.feed(data):
                last_packet = time.monotonic()
                s.stats["imu_read"] += 1
                if clock is None:
                    warm.append((tick, receipt, values))
                    if tick - warm[0][0] < WARMUP_US:
                        continue
                    model = timing_core.imu_mapping([p[0] for p in warm], [p[1] for p in warm])
                    check_clock_fit(model)
                    clock = RollingClock(model, warm)
                    # Keep a recent pre-roll so the first frame has a sample on its left.
                    batch.extend((mapped_tick(t, model), v) for t, _, v in warm[-500:-1])
                    warm.clear()
                stamp = clock.stamp(tick, receipt)
                batch.append((stamp, values))
            if batch:
                with s.condition:
                    s.model = model
                    s.clock_update = clock.last_update
                    s.stats["clock_refits"] = clock.refits
                    s.stats["max_clock_median_ms"] = clock.max_median_ms
                    s.imu.extend(batch)
                    s.check_imu_backlog()
                    s.condition.notify_all()


def read_camera(s, pipe):
    previous = 0
    while not s.stop.is_set():
        header = pipe.read(8)
        if len(header) != 8:
            raise EOFError("Camera helper exited; inspect camera.log.")
        stamp, = struct.unpack("<q", header)
        if stamp <= previous:
            raise ValueError("Nonmonotonic camera timestamp.")
        previous = stamp
        pixels = pipe.read(PIXELS)
        if len(pixels) != PIXELS:
            raise EOFError("Incomplete grayscale frame from camera helper.")
        with s.condition:
            s.remember_camera(stamp, pixels)
            s.condition.notify_all()


def take_packet(s, shift, previous):
    with s.condition:
        s.mark("waiting_camera")
        s.wait(lambda: s.camera_anchors or s.latest is not None, 5, "No camera frame in 5 s; inspect camera.log.")
        if s.camera_anchors:
            stamp, pixels = s.camera_anchors.popleft()
            s.stats["catchup_frames"] += 1
        else:
            stamp, pixels = s.latest
            s.latest = None
        s.camera_shift_ns = shift
        stamp += shift  # The camera helper maps the driver clock only; this is the sole Kalibr shift.
        s.pending_camera_ns = stamp
        s.mark("waiting_imu")
        s.wait(lambda: s.imu and s.imu[-1][0] > stamp, 2, "IMU does not cover camera timestamp.")
        samples = []
        while s.imu and s.imu[0][0] <= stamp:
            samples.append(s.imu.popleft())
        if not samples or stamp - samples[-1][0] > 3_000_000:
            raise ValueError("No bracketing IMU sample near the image.")
        if previous is None:
            samples = samples[-1:]
        else:
            dt = (stamp - previous) / NS
            if not 0 < stamp - previous < MAX_CAMERA_GAP_NS:
                raise RuntimeError(f"Tracked frames {dt:.3f} s apart; check processing speed/clock.")
            s.stats["max_frame_interval_s"] = max(s.stats["max_frame_interval_s"], dt)
        s.first_sent = True
        s.pending_camera_ns = None
        s.last_selected_ns = stamp
    return stamp, samples, pixels


def packet_header(stamp, samples):
    header = [f"{stamp} {len(samples)}\n"]
    header.extend(f"{t} " + " ".join(str(v) for v in values) + "\n" for t, values in samples)
    return "".join(header).encode("ascii")


def send_packet(s, fd, stamp, samples, pixels):
    encoded = packet_header(stamp, samples)
    start = time.monotonic()
    end = start + 8
    s.transfer = {"camera_ns": stamp, "imu_count": len(samples),
                  "packet_bytes": len(encoded) + len(pixels), "written_bytes": 0}
    for phase, part in (("writing_packet_header", encoded), ("writing_image", pixels)):
        s.mark(phase)
        view = memoryview(part)
        while view:
            s.check()
            if time.monotonic() > end:
                raise TimeoutError("ORB input blocked for 8 s; inspect orb.log.")
            if select.select([], [fd], [], .2)[1]:
                try:
                    n = os.write(fd, view)
                    if n == 0:
                        raise BrokenPipeError("No bytes written to ORB")
                    s.transfer["written_bytes"] += n
                    view = view[n:]
                except BlockingIOError:
                    continue
    s.stats["frames_sent"] += 1
    s.stats["imu_sent"] += len(samples)
    s.last_sent_ns = stamp
    s.stats["max_packet_write_s"] = max(s.stats.get("max_packet_write_s", 0.0), time.monotonic() - start)
    s.transfer = None


def wait_ready(child, log_path):
    print("Loading ORB vocabulary; progress in", log_path, flush=True)
    deadline = time.monotonic() + 180
    tail = b""
    with log_path.open("rb") as stream:
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError(f"ORB exited during initialization ({child.returncode}); inspect orb.log.")
            chunk = stream.read(65536)
            tail = tail + chunk
            if b"ORB_LIVE_READY\n" in tail:
                return
            tail = tail[-64:]
            if not chunk:
                time.sleep(.15)
    raise TimeoutError("ORB startup took more than 180 s; inspect orb.log.")


def terminate(child):
    if child is None or child.poll() is not None:
        return
    child.terminate()
    try:
        child.wait(3)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def monitor_runner(s, child):
    code = child.wait()
    if not s.stop.is_set():
        raise RuntimeError(f"ORB exited during capture (code {code}); inspect orb.log.")


def run(args, out, shift):
    feeds = Streams()
    cam = orb = None
    threads = []
    report = {"camera_shift_ns": shift, "duration_requested_s": args.seconds,
              "duration_mode": "until_stopped" if args.seconds == 0 else "timed",
              "camera_mode": "full", "input_image_size": [IMAGE_WIDTH, IMAGE_HEIGHT],
              "status": "INCOMPLETE", "streams": feeds.stats, "viewer": args.viewer}
    stop_signal = None
    streaming = False
    def request_stop(signum, _frame):
        nonlocal stop_signal
        if stop_signal is not None or not streaming:
            raise KeyboardInterrupt(f"signal {signum}")
        stop_signal = signum
        report["stop_signal"] = signum
        # Sensor threads notify their condition regularly. Only the receiving loop
        # checks this flag: a frame already being written must finish before EOF.
        feeds.finish_requested = True
        print("Finishing the current frame and finalizing the session.", flush=True)

    def start(fn, *params):
        t = threading.Thread(target=feeds.worker, args=(fn, feeds, *params), daemon=True)
        t.start()
        threads.append(t)

    original_handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        with (out / "orb.log").open("xb") as orb_log, (out / "camera.log").open("xb") as cam_log:
            binary = "orb_live_viewer" if args.viewer else "orb_live_runner"
            orb = subprocess.Popen([args.bindir / binary, args.vocabulary, args.yaml],
                                   stdin=subprocess.PIPE, stdout=orb_log, stderr=subprocess.STDOUT,
                                   cwd=out, start_new_session=True)
            feeds.mark("loading_orb")
            wait_ready(orb, out / "orb.log")
            start(monitor_runner, orb)
            print("Estimating the IMU timer from 10 s of fresh serial data.", flush=True)
            feeds.mark("imu_clock_startup")
            start(read_imu)
            with feeds.condition:
                feeds.wait(lambda: feeds.model is not None, 18, "IMU clock startup timed out.")
            cam = subprocess.Popen([args.bindir / "camera_live"], stdout=subprocess.PIPE,
                                   stderr=cam_log, cwd=out, start_new_session=True)
            start(read_camera, cam.stdout)
            fd = orb.stdin.fileno()
            os.set_blocking(fd, False)
            streaming = True
            previous = None
            start_time = None
            next_notice = time.monotonic()
            while stop_signal is None and (args.seconds == 0 or start_time is None or
                                           time.monotonic() - start_time < args.seconds):
                try:
                    stamp, samples, pixels = take_packet(feeds, shift, previous)
                except CaptureStopped:
                    break
                if feeds.finish_requested:
                    break
                try:
                    send_packet(feeds, fd, stamp, samples, pixels)
                except BrokenPipeError as exc:
                    raise RuntimeError("ORB closed its input before capture finished; inspect orb.log.") from exc
                previous = stamp
                now = time.monotonic()
                if start_time is None:
                    start_time = now
                    report["first_frame_ns"] = stamp
                    print("LIVE: move the rig with both translation and rotation in a textured scene.", flush=True)
                report["last_frame_ns"] = stamp
                if now >= next_notice:
                    print(f"Live {now-start_time:.1f}s, sent {feeds.stats['frames_sent']} frames, "
                          f"superseded {feeds.stats['frames_superseded']}.", flush=True)
                    next_notice = now + 5
            feeds.check()
            feeds.stop.set()
            terminate(cam)
            orb.stdin.close()  # Native backend finalizes trajectories on clean EOF.
            print("Input closed; waiting for ORB to finish writing the session.", flush=True)
            feeds.mark("finalizing_orb")
            orb.wait()
            report["orb_exit_code"] = orb.returncode
            if orb.returncode == 3:
                report["status"] = "TRACKING_REVIEW_REQUIRED"
                if args.viewer:
                    raise RuntimeError("Final tracking or export requires review. See tracking.csv for frame states, poses.csv for recorded camera poses, and orb.log for details.")
                raise RuntimeError("Live acquisition completed; inertial tracking or trajectories require review. See tracking.csv and orb.log.")
            if orb.returncode:
                raise RuntimeError(f"ORB exited {orb.returncode}; inspect orb.log.")
            report["status"] = "STREAM_COMPLETED"
    except BaseException as exc:
        report["error"] = str(exc) or type(exc).__name__
        report["failure_context"] = feeds.failure_context or feeds.snapshot()
        report["orb_exit_at_failure"] = orb.poll() if orb is not None else None
        report["camera_exit_at_failure"] = cam.poll() if cam is not None else None
        raise
    finally:
        feeds.stop.set()
        terminate(cam)
        if orb is not None and orb.stdin and not orb.stdin.closed:
            transfer = feeds.transfer
            partial = transfer and 0 < transfer["written_bytes"] < transfer["packet_bytes"]
            if partial:
                # The protocol cannot resynchronize inside a gray image. Never report a clean EOF here.
                report["orb_input_close"] = "aborted_partial_packet"
                terminate(orb)
            orb.stdin.close()
            if not partial and report.get("error") and feeds.stats["frames_sent"] and orb.poll() is None:
                # The complete prefix can still yield trajectories, but the session remains incomplete.
                report["orb_input_close"] = "clean_eof_after_failure"
                try:
                    orb.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    report["orb_input_close"] = "unresponsive_after_clean_eof"
        terminate(orb)
        for t in threads:
            t.join(timeout=3)
        if cam is not None and cam.stdout:
            cam.stdout.close()
        if orb is not None:
            report["orb_exit_code"] = orb.returncode
        report["imu_clock_fit"] = feeds.model
        report["last_imu_clock_update"] = feeds.clock_update
        with (out / "live.json").open("x") as f:
            json.dump(report, f, indent=2, allow_nan=False)
            f.write("\n")
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        print("Trial report:", out / "live.json", flush=True)


def camera_shift(path):
    timing = json.loads(path.read_text())
    shift = timing["timeshift_cam_imu_rounded_ns"]
    if (type(shift) is not int or abs(shift) >= NS or
            shift != round(timing["timeshift_cam_imu_seconds"] * NS)):
        raise ValueError("Inconsistent camera shift in timing JSON.")
    return shift


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ("bindir", "vocabulary", "yaml", "timing", "out"):
        ap.add_argument(name, type=lambda p: Path(p).expanduser().resolve())
    ap.add_argument("--seconds", type=int, help="tracked seconds after startup; 0 runs until Ctrl+C "
                                               "(default: 0 with --viewer, 60 otherwise)")
    ap.add_argument("--viewer", action="store_true", help="show the native ORB map and current frame windows")
    args = ap.parse_args()
    if sys.platform != "linux" or platform.machine() != "aarch64":
        ap.error("Run this command on the Jetson Linux aarch64 host.")
    if args.seconds is None:
        args.seconds = 0 if args.viewer else 60
    if args.seconds < 0:
        ap.error("--seconds must be 0 (until Ctrl+C) or a positive integer.")
    if args.viewer and not os.environ.get("DISPLAY"):
        ap.error("Open the live viewer in a Jetson graphical desktop terminal with DISPLAY set.")
    for p in (args.bindir / "camera_live", args.bindir / ("orb_live_viewer" if args.viewer else "orb_live_runner")):
        if not p.is_file() or not os.access(p, os.X_OK):
            ap.error(f"Missing executable: {p}")
    for p in (args.vocabulary, args.yaml, args.timing):
        if not p.is_file():
            ap.error(f"Missing file: {p}")
    shift = camera_shift(args.timing)
    args.out.mkdir(mode=0o700, exist_ok=False)
    print("New live output:", args.out, flush=True)
    if args.seconds == 0:
        print("Continuous tracking. Press Ctrl+C once to finish and save.", flush=True)
    run(args, args.out, shift)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        raise SystemExit(str(error) or type(error).__name__)
