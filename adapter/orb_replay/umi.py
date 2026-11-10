#!/usr/bin/env python3
"""Capture real image/IMU packets once; solve a take at its own pace."""
import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

sys.dont_write_bytecode = True
import live
from umi_quality import (FRAME_FIELDS, checked_frames, checked_holds, dump_json, export_trajectory,
                         file_sha256, load_frames, measure)

CLOCK = time.CLOCK_MONOTONIC_RAW
RESERVE = 512 * 1024 * 1024
FLOAT32_MAX = 3.4028234663852886e38  # ORB's IMU parser reads six C++ floats.


def require_room(folder):
    if shutil.disk_usage(folder).free < RESERVE:
        raise OSError("Insufficient disk space: keep at least 512 MiB free.")


def record(folder, camera):
    # The launcher has reserved a new directory and copied the exact settings used.
    shift = live.camera_shift(folder / "timing.json")
    feeds = live.Streams()
    thread = threading.Thread(target=feeds.worker, args=(live.read_imu, feeds), daemon=True)
    cap = {"status": "RECORDING", "image_format": "mono8", "width": 1920, "height": 1440,
           "camera_shift_ns": shift, "clock": "CLOCK_MONOTONIC_RAW", "frames": 0,
           "driver_missing": 0, "imu_samples": 0,
           "sha256": {name: file_sha256(folder / name) for name in ("settings.yaml", "timing.json")}}
    digest = hashlib.sha256()
    dump_json(folder / "capture.json", cap)
    proc = None

    def stop(sig, frame):
        # Repeated terminal signals can arrive while fsync is still running.
        feeds.finish_requested = True

    handlers = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (folder / "frames.csv").open("x", newline="", buffering=1) as table, \
                (folder / "stream.bin").open("xb") as stream, (folder / "camera.log").open("x") as log:
            fcntl.flock(table, fcntl.LOCK_EX | fcntl.LOCK_NB)
            writer = csv.writer(table)
            writer.writerow(FRAME_FIELDS)
            require_room(folder)
            print("Preparing IMU clock (about 10 s).", flush=True)
            thread.start()
            with feeds.condition:
                feeds.wait(lambda: feeds.model is not None, 18, "IMU clock startup timed out.")
            proc = subprocess.Popen([camera], stdout=subprocess.PIPE, stderr=log, start_new_session=True)
            previous = previous_seq = None
            while not feeds.finish_requested:
                data = proc.stdout.read(12)
                if len(data) != 12:
                    raise EOFError("Camera stream stopped. See camera.log.")
                raw, seq = struct.unpack("<qI", data)
                image = proc.stdout.read(live.PIXELS)
                if len(image) != live.PIXELS:
                    raise EOFError("Incomplete source image.")
                gap = 0 if previous_seq is None else (seq - previous_seq - 1) % 2**32
                if gap >= 2**31:
                    raise ValueError("Driver sequence repeated or moved backwards.")
                with feeds.condition:
                    feeds.remember_camera(raw, image)
                try:
                    stamp, samples, _ = live.take_packet(feeds, shift, previous)
                except live.CaptureStopped:
                    break
                if cap["frames"] % 15 == 0:
                    require_room(folder)
                offset = stream.tell()
                header = live.packet_header(stamp, samples)
                stream.write(header)
                stream.write(image)
                digest.update(header)
                digest.update(image)
                writer.writerow((cap["frames"], seq, raw, stamp, len(samples), offset, len(header) + len(image)))
                cap["frames"] += 1
                cap["driver_missing"] += gap
                cap["imu_samples"] += len(samples)
                if cap["frames"] == 1 or cap["frames"] % 150 == 0:
                    print(f"Recorded {cap['frames']} frames, driver sequence gaps {cap['driver_missing']}.", flush=True)
                previous, previous_seq = stamp, seq
            feeds.check()
            if cap["frames"] < 2:
                raise ValueError("Recording ended before two complete frames.")
            print("Finishing recording; syncing sensor data to disk.", flush=True)
            stream.flush()
            table.flush()
            os.fsync(stream.fileno())
            os.fsync(table.fileno())
        cap["sha256"].update({"stream.bin": digest.hexdigest(), "frames.csv": file_sha256(folder / "frames.csv")})
        cap["status"] = "COMPLETE"
    except BaseException as error:
        cap.update(status="INCOMPLETE", error=str(error) or type(error).__name__)
        raise
    finally:
        try:
            feeds.stop.set()
            live.terminate(proc)
            if proc and proc.stdout:
                proc.stdout.close()
            if thread.ident is not None:
                thread.join(timeout=4)
            cap["imu_clock_fit"] = feeds.model
            cap["clock_last_update"] = feeds.clock_update
            cap["max_clock_median_ms"] = feeds.stats.get("max_clock_median_ms")
            dump_json(folder / "capture.json", cap)
            print(cap["status"], folder, flush=True)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def verified_stream(path, frames):
    digest = hashlib.sha256()
    previous_imu = previous_image = 0
    with path.open("rb") as stream:
        before = os.fstat(stream.fileno())
        for f in frames:
            header = stream.read(f.size - live.PIXELS)
            lines = header.split(b"\n")
            if (len(lines) != f.imu_count + 2 or lines[-1] or not header.isascii() or b"_" in header or
                    lines[0] != f"{f.camera_ns} {f.imu_count}".encode() or
                    (f.frame_idx == 0 and f.imu_count != 1)):
                raise ValueError(f"Packet header differs from index at frame {f.frame_idx}.")
            for line in lines[1:-1]:
                fields = line.split()
                try:
                    stamp = int(fields[0])
                    values = tuple(map(float, fields[1:]))
                except (IndexError, ValueError) as exc:
                    raise ValueError(f"Malformed IMU row at frame {f.frame_idx}.") from exc
                if (len(values) != 6 or not all(math.isfinite(v) and abs(v) <= FLOAT32_MAX for v in values) or
                        not max(previous_imu, previous_image) < stamp <= f.camera_ns):
                    raise ValueError(f"Invalid IMU time or SI values at frame {f.frame_idx}.")
                # The recorder enforces 1500–2500 us on the device clock. Leave
                # margin for the bounded rate correction to the host RAW clock.
                if previous_imu and not 1_498_000 <= stamp - previous_imu <= 2_502_000:
                    raise ValueError(f"IMU cadence discontinuity at frame {f.frame_idx}.")
                previous_imu = stamp
            if f.camera_ns - previous_imu > 3_000_000:
                raise ValueError(f"No near-image IMU sample at frame {f.frame_idx}.")
            pixels = stream.read(live.PIXELS)
            if len(pixels) != live.PIXELS:
                raise ValueError(f"Short image at frame {f.frame_idx}.")
            digest.update(header)
            digest.update(pixels)
            previous_image = f.camera_ns
            if (f.frame_idx + 1) % 500 == 0:
                print(f"已核验 {f.frame_idx + 1}/{len(frames)} 帧", flush=True)
        if stream.read(1) or os.fstat(stream.fileno()).st_mtime_ns != before.st_mtime_ns:
            raise ValueError("Extra stream data or file changed during verification.")
    return digest.hexdigest()


def recover(folder):
    # Shared on the index blocks a recorder but does not look like one to umi-mark.
    # Serialize auditors on the final, immutable capture report.
    with (folder / "frames.csv").open("rb") as table, (folder / "capture.json").open("rb") as manifest:
        try:
            fcntl.flock(table, fcntl.LOCK_SH | fcntl.LOCK_NB)
            fcntl.flock(manifest, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError("Recorder or another verification is active. Wait for it to finish.") from exc
        original = manifest.read()
        cap = json.loads(original)
        if cap["status"] != "INCOMPLETE" or cap.get("error") != "KeyboardInterrupt":
            raise ValueError("umi-recover requires a closed INCOMPLETE take with error KeyboardInterrupt.")
        if (cap["image_format"], cap["width"], cap["height"], cap["clock"]) != (
                "mono8", 1920, 1440, "CLOCK_MONOTONIC_RAW"):
            raise ValueError("Unexpected recording format or clock.")
        if cap["camera_shift_ns"] != live.camera_shift(folder / "timing.json"):
            raise ValueError("Camera shift differs from timing.json.")
        index_digest = file_sha256(folder / "frames.csv")
        captured = cap["sha256"]
        if captured.get("frames.csv", index_digest) != index_digest:
            raise ValueError("frames.csv differs from the recorder checksum.")
        frames = checked_frames(folder, {**cap, "sha256": {**captured, "frames.csv": index_digest}})
        stream_bytes = frames[-1].offset + frames[-1].size
        print(f"核验 {len(frames)} 帧、{stream_bytes:,} 字节，进度每 500 帧更新。", flush=True)
        stream_digest = verified_stream(folder / "stream.bin", frames)
        if captured.get("stream.bin", stream_digest) != stream_digest:
            raise ValueError("stream.bin differs from the recorder checksum.")
        marks = folder / "marks.json"
        marks_status = "ABSENT"
        marks_digest = None
        if marks.is_file():
            marks_digest = file_sha256(marks)
            try:
                checked_holds(folder, frames)
                marks_status = "IN_RANGE"
            except (ValueError, KeyError, TypeError) as exc:
                marks_status = f"REVIEW: {exc}"
        proof = {"format": "umi-recovery-v1", "source_capture_sha256": hashlib.sha256(original).hexdigest(),
                 "frames": len(frames), "imu_samples": cap["imu_samples"], "driver_missing": cap["driver_missing"],
                 "stream_bytes": stream_bytes, "marks": marks_status, "marks_sha256": marks_digest,
                 "sha256": {"frames.csv": index_digest, "stream.bin": stream_digest}}
        path = folder / "recovery.json"
        if path.exists():
            if json.loads(path.read_bytes()) != proof:
                raise ValueError("The current data differs from the existing recovery report.")
        else:
            dump_json(path, proof)
        print(f"RECOVERY_VERIFIED frames={len(frames)} imu_samples={cap['imu_samples']} marks={marks_status}", flush=True)


def mark(folder):
    target = folder / "marks.json"
    if target.exists():
        raise ValueError("This take already has A/B/A return marks.")
    with (folder / "frames.csv").open() as active:
        def check_recording():
            try:
                fcntl.flock(active, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError:
                return
            fcntl.flock(active, fcntl.LOCK_UN)
            raise ValueError("This directory has no active recorder.")

        check_recording()
        if not next(csv.DictReader(active), None):
            raise ValueError("Waiting for first camera image. Re-run after record.log says Recorded.")
        holds = {}
        for name, label in (("A", "起点 A"), ("B", "另一固定姿态 B"), ("return_A", "返回同一限位、同一朝向 A")):
            input(f"将组件放在{label}，手离开、等它稳住后按回车。")
            check_recording()
            start = time.clock_gettime_ns(CLOCK)
            time.sleep(3)
            end = time.clock_gettime_ns(CLOCK)
            check_recording()
            holds[name] = [start, end]
            print(f"{label} 的 3 秒静止段已记录。", flush=True)
        # Link publication refuses races with a second marking terminal.
        with tempfile.NamedTemporaryFile(mode="w", dir=folder) as f:
            json.dump({"clock": "CLOCK_MONOTONIC_RAW", "holds": holds}, f)
            f.flush()
            os.link(f.name, target)
    print(target)


def next_solve(folder):
    for n in range(1, 10000):
        attempt = folder / f"solve_{n:02d}"
        try:
            attempt.mkdir(mode=0o700)
            return attempt
        except FileExistsError:
            continue
    raise ValueError("Solve directory numbers exhausted.")


def solve(folder, binary, vocabulary, show):
    cap, frames = load_frames(folder)
    out = next_solve(folder)
    print(f"逐帧求解目录 {out}", flush=True)
    command = [binary, vocabulary, folder / "settings.yaml", "--umi" if show else "--umi-no-view"]
    progress = {"status": "PROCESSING", "expected_frames": len(frames), "viewer": show,
                "runner_sha256": file_sha256(binary), "settings_sha256": cap["sha256"]["settings.yaml"]}
    progress["source_capture_status"] = cap["status"]
    if cap["status"] == "RECOVERED":
        progress["source_recovery_sha256"] = cap["recovery_sha256"]
    library = binary.parent.parent / "lib/libORB_SLAM3.so"
    if not library.is_file():
        library = vocabulary.parent.parent / "lib/libORB_SLAM3.so"  # Pre-Docker installation.
    if library.is_file():
        progress["orb_library_sha256"] = file_sha256(library)
    dump_json(out / "solve.json", progress)
    try:
        with (out / "orb.log").open("xb") as log, (folder / "stream.bin").open("rb") as source:
            child = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                                     cwd=out, start_new_session=True)
            try:
                live.wait_ready(child, out / "orb.log")
                scheduled = time.monotonic()
                last = frames[0].camera_ns
                digest = hashlib.sha256()
                for f in frames:
                    packet = source.read(f.size)
                    if len(packet) != f.size or not packet.startswith(f"{f.camera_ns} {f.imu_count}\n".encode()):
                        raise ValueError(f"Damaged capture packet {f.frame_idx}")
                    digest.update(packet)
                    scheduled += (f.camera_ns - last) / live.NS
                    time.sleep(max(0, scheduled - time.monotonic()))
                    # When the backend is slow, delay the next frame. Never catch
                    # up by dropping images or stretching their sensor timestamps.
                    scheduled = time.monotonic()
                    child.stdin.write(packet)
                    last = f.camera_ns
                if source.read(1) or digest.hexdigest() != cap["sha256"]["stream.bin"]:
                    raise ValueError("Recorded stream length or checksum changed.")
                child.stdin.close()
                code = child.wait()
                progress["orb_exit_code"] = code
            finally:
                live.terminate(child)
                if not child.stdin.closed:
                    try:
                        child.stdin.close()
                    except BrokenPipeError:
                        pass
        if code != 0:
            raise RuntimeError(f"ORB exited {code}. Inspect {out}/orb.log and tracking.csv.")
        result = export_trajectory(folder, out, cap, frames)
        progress.update(status="EXPORTED", **result)
    except BaseException as error:
        progress.update(status="REVIEW", error=str(error) or type(error).__name__)
        raise
    finally:
        dump_json(out / "solve.json", progress)
    print(f"逐帧轨迹 {out/'camera_trajectory.csv'}，有效 {result['valid_frames']}/{len(frames)}。")
    print("运行 umi-check 查看实测重复性。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    cmd = parser.add_subparsers(dest="mode", required=True)
    rec = cmd.add_parser("record")
    rec.add_argument("folder", type=Path)
    rec.add_argument("camera", type=Path)
    cmd.add_parser("mark").add_argument("folder", type=Path)
    cmd.add_parser("recover").add_argument("folder", type=Path)
    run = cmd.add_parser("track")
    run.add_argument("folder", type=Path)
    run.add_argument("binary", type=Path)
    run.add_argument("vocabulary", type=Path)
    run.add_argument("--view", action="store_true")
    qa = cmd.add_parser("check")
    qa.add_argument("folder", type=Path, help="one solve_XX directory")
    for name in ("distance-mm", "angle-deg", "distance-tol-mm", "angle-tol-deg"):
        qa.add_argument("--" + name, type=float)
    args = parser.parse_args()
    folder = args.folder.expanduser().resolve()
    if args.mode == "record":
        record(folder, args.camera.resolve())
    elif args.mode == "mark":
        mark(folder)
    elif args.mode == "recover":
        recover(folder)
    elif args.mode == "track":
        solve(folder, args.binary.resolve(), args.vocabulary.resolve(), args.view)
    else:
        measure(folder, args)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, EOFError, KeyError, KeyboardInterrupt) as exc:
        sys.exit(str(exc) or type(exc).__name__)
