#!/usr/bin/env python3
"""Replay a take in ORB-SLAM3, or supervise recording with a camera-only preview."""
import argparse
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
from camera_profile import load_snapshot
from umi_quality import load_frames


def check_packet(packet, frame):
    expected = f"{frame.camera_ns} {frame.imu_count}\n".encode()
    if len(packet) != frame.size or not packet.startswith(expected):
        raise ValueError(f"Damaged packet at frame {frame.frame_idx}.")


def send_packet(child, packet):
    # Preserve packet boundaries, but time out if the Viewer has stopped
    # consuming replay input.
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
    load_snapshot(settings.parent)
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


def record_with_preview(folder, viewer, camera):
    _, layout = load_snapshot(folder)
    preview_log = folder / "preview.log"
    stop_requested = False

    def request_stop(_signal, _frame):
        nonlocal stop_requested
        stop_requested = True

    handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    recorder = child = None
    try:
        if stop_requested:
            raise SystemExit("Interrupted before capture started.")
        with (folder / "record.log").open("xb") as output:
            recorder = subprocess.Popen([sys.executable, "-u", TOOLS / "umi.py", "record", folder, camera],
                                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        if not stop_requested:
            try:
                with preview_log.open("xb") as output:
                    child = subprocess.Popen([viewer, folder, str(layout["width"]), str(layout["height"])],
                                             stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                print(f"Camera preview failed to start ({exc}). Recording continues.", flush=True)

        while not stop_requested and recorder.poll() is None:
            if child is not None and child.poll() is not None:
                if child.returncode == 0:
                    print("Camera preview closed. Recording continues until Ctrl+C.", flush=True)
                else:
                    print(f"Camera preview stopped (code {child.returncode}). Recording continues. "
                          f"See {preview_log}.", flush=True)
                child = None
            time.sleep(0.05)
    finally:
        try:
            if recorder is not None and recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            try:
                live.terminate(child)
            finally:
                if recorder is not None:
                    code = recorder.wait()
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    if code:
        raise RuntimeError(f"Recording exited with code {code}. See {folder}/record.log.")
    print(f"Recording complete: {folder}", flush=True)


def interrupted(_signal, _frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    modes = ap.add_subparsers(dest="mode", required=True)

    def absolute(path):
        return Path(path).expanduser().resolve()

    play = modes.add_parser("replay")
    for name in ("folder", "binary", "vocabulary"):
        play.add_argument(name, type=absolute)

    record = modes.add_parser("record-view")
    for name in ("folder", "viewer", "camera"):
        record.add_argument(name, type=absolute)

    args = ap.parse_args()
    if args.mode == "replay":
        signal.signal(signal.SIGTERM, interrupted)
        replay(args.folder, args.binary, args.vocabulary)
    else:
        record_with_preview(args.folder, args.viewer, args.camera)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("Stopped.")
    except (OSError, ValueError, RuntimeError, TypeError, KeyError, subprocess.SubprocessError) as exc:
        raise SystemExit(str(exc)) from exc
