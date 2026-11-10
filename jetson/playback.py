#!/usr/bin/env python3
"""Feed calibrated camera and IMU packets to the original ORB-SLAM3 viewer."""
import argparse
import csv
import hashlib
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time


TOOLS = Path(__file__).resolve().parent / "tools" / "orb_replay"
sys.dont_write_bytecode = True
sys.path.insert(0, str(TOOLS))
import live
from umi_quality import FRAME_FIELDS, Frame, load_frames


def check_packet(packet, frame):
    expected = f"{frame.camera_ns} {frame.imu_count}\n".encode()
    if len(packet) != frame.size or not packet.startswith(expected):
        raise ValueError(f"Damaged packet at frame {frame.frame_idx}.")


def send_packet(child, packet):
    # Preserve packet boundaries, but time out if the Viewer has stopped
    # consuming input. The capture process has its own file writer.
    fd = child.stdin.fileno()
    remaining = memoryview(packet)
    deadline = time.monotonic() + 8
    while remaining:
        wait = deadline - time.monotonic()
        if wait <= 0 or not select.select([], [fd], [], wait)[1]:
            raise TimeoutError("ORB input pipe stalled. Inspect the Viewer log.")
        try:
            remaining = remaining[os.write(fd, remaining):]
        except BlockingIOError:
            continue


def open_viewer(binary, vocabulary, settings, log):
    print("ORB-SLAM3 Viewer log:", log, flush=True)
    with log.open("xb") as output:
        child = subprocess.Popen([binary, vocabulary, settings], stdin=subprocess.PIPE, bufsize=0,
                                 stdout=output, stderr=subprocess.STDOUT, cwd=log.parent,
                                 start_new_session=True)
    try:
        live.wait_ready(child, log)
        os.set_blocking(child.stdin.fileno(), False)
        return child
    except BaseException:
        close_viewer(child)
        raise


def close_viewer(child):
    live.terminate(child)
    if not child.stdin.closed:
        child.stdin.close()


def replay(folder, binary, vocabulary):
    capture, frames = load_frames(folder)
    out = Path(tempfile.mkdtemp(prefix="stock-replay-", dir=folder))
    child = open_viewer(binary, vocabulary, folder / "settings.yaml", out / "orb.log")
    try:
        with (folder / "stream.bin").open("rb") as source:
            digest = hashlib.sha256()
            previous = frames[0].camera_ns
            target = time.monotonic()
            for frame in frames:
                packet = source.read(frame.size)
                check_packet(packet, frame)
                digest.update(packet)
                target += (frame.camera_ns - previous) / live.NS
                time.sleep(max(0, target - time.monotonic()))
                target = max(target, time.monotonic())
                send_packet(child, packet)
                previous = frame.camera_ns
            if source.read(1) or digest.hexdigest() != capture["sha256"]["stream.bin"]:
                raise ValueError("Recorded stream length or checksum changed.")
        child.stdin.close()
        code = child.wait(timeout=120)
        if code:
            raise RuntimeError(f"ORB exited with code {code}. See {out}/orb.log.")
    finally:
        close_viewer(child)
    print(f"Replayed {len(frames)} camera packets from {folder}.")


def follow_lines(table, recorder, child):
    while recorder.poll() is None:
        if child.poll() is not None:
            raise RuntimeError(f"ORB exited with code {child.returncode}.")
        offset = table.tell()
        line = table.readline()
        if line.endswith("\n"):
            yield line
        else:
            table.seek(offset)
            time.sleep(0.025)


def preview(folder, recorder, child):
    index = folder / "frames.csv"
    packets = folder / "stream.bin"
    while not (index.is_file() and packets.is_file()):
        if recorder.poll() is not None:
            return
        if child.poll() is not None:
            raise RuntimeError(f"ORB exited with code {child.returncode}.")
        time.sleep(0.05)
    with index.open(newline="") as table, packets.open("rb") as source:
        rows = csv.reader(follow_lines(table, recorder, child))
        fields = next(rows, None)
        if fields is None:
            return
        if fields != list(FRAME_FIELDS):
            raise ValueError("Unexpected recording frame index.")
        for row in rows:
            if len(row) != len(FRAME_FIELDS):
                raise ValueError("Unexpected recording frame index row.")
            frame = Frame(*map(int, row))
            if (frame.offset != source.tell() or not 1 <= frame.imu_count <= 1000 or
                    not live.PIXELS < frame.size <= live.PIXELS + 256 * (frame.imu_count + 1)):
                raise ValueError(f"Invalid recording index at frame {frame.frame_idx}.")
            while os.fstat(source.fileno()).st_size < frame.offset + frame.size:
                if recorder.poll() is not None:
                    raise ValueError(f"Recorder exited before writing frame {frame.frame_idx}.")
                if child.poll() is not None:
                    raise RuntimeError(f"ORB exited with code {child.returncode}.")
                time.sleep(0.025)
            packet = source.read(frame.size)
            check_packet(packet, frame)
            send_packet(child, packet)


def record_with_preview(folder, binary, vocabulary, camera):
    child = open_viewer(binary, vocabulary, folder / "settings.yaml", folder / "preview.log")
    recorder = None
    try:
        with (folder / "record.log").open("xb") as output:
            recorder = subprocess.Popen([sys.executable, "-u", TOOLS / "umi.py", "record", folder, camera],
                                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            preview(folder, recorder, child)
        except (OSError, ValueError, RuntimeError, csv.Error) as exc:
            close_viewer(child)
            print(f"Preview stopped ({exc}). Recording continues. See {folder}/preview.log.", flush=True)
        recorder.wait()
    except KeyboardInterrupt:
        pass
    finally:
        # Keep additional Ctrl+C presses from interrupting the recorder's fsync.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
        close_viewer(child)
        if recorder is not None:
            code = recorder.wait()
            if code:
                raise RuntimeError(f"Recording exited with code {code}. See {folder}/record.log.")
    if recorder is None:
        raise SystemExit("Interrupted before capture started.")
    print(f"Recording complete: {folder}")


def interrupted(_signal, _frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("mode", choices=("replay", "record-view"))
    for name in ("folder", "binary", "vocabulary"):
        ap.add_argument(name, type=lambda path: Path(path).expanduser().resolve())
    ap.add_argument("camera", nargs="?", type=lambda path: Path(path).expanduser().resolve())
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    if args.mode == "replay":
        replay(args.folder, args.binary, args.vocabulary)
    elif args.camera is not None:
        record_with_preview(args.folder, args.binary, args.vocabulary, args.camera)
    else:
        ap.error("record-view needs a camera helper.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("Stopped.")
    except (OSError, ValueError, RuntimeError, TypeError, KeyError, subprocess.SubprocessError) as exc:
        raise SystemExit(str(exc)) from exc
