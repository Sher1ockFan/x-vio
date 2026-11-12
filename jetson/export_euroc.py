#!/usr/bin/env python3
"""Export a sealed take for the retained monocular-inertial EuRoC example."""
import argparse
import csv
import hashlib
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import zlib


TOOLS = Path(__file__).resolve().parent / "tools" / "orb_replay"
sys.dont_write_bytecode = True
sys.path.insert(0, str(TOOLS))
import live
from umi_quality import dump_json, load_frames

WIDTH, HEIGHT = 1920, 1440
FLOAT32_MAX = 3.4028234663852886e38
FRAME_FIELDS = ("frame_idx", "sequence", "camera_raw_ns", "camera_ns")


def png_chunk(kind, payload):
    checksum = zlib.crc32(payload, zlib.crc32(kind))
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def write_gray_png(path, pixels):
    # PNG filter 0 prefixes each unchanged source row with one zero byte.
    stride = WIDTH + 1
    scanlines = bytearray(stride * HEIGHT)
    image = memoryview(pixels)
    for row in range(HEIGHT):
        scanlines[row * stride + 1:(row + 1) * stride] = image[row * WIDTH:(row + 1) * WIDTH]
    with path.open("xb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(png_chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 0, 0, 0, 0)))
        f.write(png_chunk(b"IDAT", zlib.compress(scanlines, 1)))
        f.write(png_chunk(b"IEND", b""))


def native_seconds(stamp_ns):
    # The upstream EuRoC reader parses nanoseconds as double before scaling.
    return float(stamp_ns) / live.NS


def read_packet(source, frame, previous_imu, previous_camera):
    header = source.read(frame.size - live.PIXELS)
    lines = header.split(b"\n")
    if (len(lines) != frame.imu_count + 2 or lines[-1] or not header.isascii()
            or lines[0] != f"{frame.camera_ns} {frame.imu_count}".encode("ascii")
            or (frame.frame_idx == 0 and frame.imu_count != 1)):
        raise ValueError(f"Invalid packet header at frame {frame.frame_idx}.")

    imu = []
    for line in lines[1:-1]:
        fields = line.split()
        if len(fields) != 7:
            raise ValueError(f"Invalid IMU row at frame {frame.frame_idx}.")
        stamp = int(fields[0])
        values = tuple(map(float, fields[1:]))
        if (not max(previous_imu, previous_camera) < stamp <= frame.camera_ns
                or not all(math.isfinite(x) and abs(x) <= FLOAT32_MAX for x in values)
                or (previous_imu and not 1_498_000 <= stamp - previous_imu <= 2_502_000)):
            raise ValueError(f"Invalid IMU stamp, cadence or measurement at frame {frame.frame_idx}.")
        ax, ay, az, gx, gy, gz = values
        imu.append((stamp, gx, gy, gz, ax, ay, az))
        previous_imu = stamp
    if frame.camera_ns - previous_imu > 3_000_000:
        raise ValueError(f"IMU coverage ends too far before frame {frame.frame_idx}.")
    pixels = source.read(live.PIXELS)
    if len(pixels) != live.PIXELS:
        raise ValueError(f"Short image at frame {frame.frame_idx}.")
    return header, pixels, imu


def export_take(take, dest):
    capture_path = take / "capture.json"
    capture_digest = hashlib.sha256(capture_path.read_bytes()).hexdigest()
    cap, frames = load_frames(take)
    if (cap.get("image_format"), cap.get("width"), cap.get("height"), cap.get("clock")) != (
            "mono8", WIDTH, HEIGHT, "CLOCK_MONOTONIC_RAW"):
        raise ValueError("Expected a 1920x1440 mono8 take on the host RAW clock.")
    shift = live.camera_shift(take / "timing.json")
    if cap["camera_shift_ns"] != shift:
        raise ValueError("The take and timing.json disagree on the recorded camera shift.")

    if dest.exists() or dest.is_symlink():
        raise FileExistsError(f"Choose a new sequence directory: {dest}")
    with tempfile.TemporaryDirectory(prefix=".euroc-export-", dir=dest.parent) as temporary:
        stage = Path(temporary) / "sequence"
        image_dir = stage / "mav0/cam0/data"
        imu_dir = stage / "mav0/imu0"
        image_dir.mkdir(parents=True)
        imu_dir.mkdir(parents=True)
        for name in ("settings.yaml", "timing.json"):
            data = (take / name).read_bytes()
            if hashlib.sha256(data).hexdigest() != cap["sha256"][name]:
                raise ValueError(f"{name} changed during export.")
            (stage / name).write_bytes(data)

        digest = hashlib.sha256()
        previous_imu = previous_camera = first_imu = 0
        first_camera = last_camera = count = 0
        withheld = []
        with (take / "stream.bin").open("rb") as source, \
                (stage / "times.txt").open("w") as times, \
                (stage / "frame_index.csv").open("w", newline="") as index, \
                (imu_dir / "data.csv").open("w", newline="") as imu_file:
            source_stat = os.fstat(source.fileno())
            frame_rows = csv.writer(index, lineterminator="\n")
            imu_rows = csv.writer(imu_file, lineterminator="\n")
            frame_rows.writerow(FRAME_FIELDS)
            imu_rows.writerow(("#timestamp [ns]", "wx [rad/s]", "wy [rad/s]", "wz [rad/s]",
                               "ax [m/s^2]", "ay [m/s^2]", "az [m/s^2]"))
            for frame in frames:
                header, pixels, imu = read_packet(source, frame, previous_imu, previous_camera)
                digest.update(header)
                digest.update(pixels)
                if frame.frame_idx == 0:
                    first_imu = imu[0][0]
                previous_imu = imu[-1][0]
                previous_camera = frame.camera_ns
                imu_rows.writerows(imu)
                # The upstream EuRoC loop reads one IMU timestamp past every
                # image time. The last packet only provides samples up to its own image.
                if frame.frame_idx == len(frames) - 1:
                    withheld.append({"frame_idx": frame.frame_idx, "reason": "no later recorded IMU sample"})
                    continue
                if native_seconds(first_imu) >= native_seconds(frame.camera_ns):
                    withheld.append({"frame_idx": frame.frame_idx, "reason": "no earlier recorded IMU sample"})
                    continue
                write_gray_png(image_dir / f"{frame.camera_ns}.png", pixels)
                times.write(f"{frame.camera_ns}\n")
                frame_rows.writerow(frame[:4])
                first_camera = first_camera or frame.camera_ns
                last_camera = frame.camera_ns
                count += 1
                if count % 150 == 0:
                    print(f"Exported {count} images.", flush=True)
            if (source.read(1) or os.fstat(source.fileno()).st_mtime_ns != source_stat.st_mtime_ns
                    or digest.hexdigest() != cap["sha256"]["stream.bin"]):
                raise ValueError("Source stream length, content or checksum changed.")

        lower, first, last, upper = map(native_seconds, (first_imu, first_camera, last_camera, previous_imu))
        if count < 2 or not lower < first < last < upper:
            raise ValueError("Need at least two exported images strictly bracketed by recorded IMU samples.")
        if hashlib.sha256(capture_path.read_bytes()).hexdigest() != capture_digest:
            raise ValueError("The capture manifest changed during export.")
        dump_json(stage / "export.json", {
            "status": "EXPORT_PASS",
            "source_take": str(take),
            "source_capture_status": cap["status"],
            "source_capture_sha256": capture_digest,
            "source_hashes": cap["sha256"],
            "source_frames": len(frames),
            "source_driver_missing": cap["driver_missing"],
            "exported_frames": count,
            "imu_samples": cap["imu_samples"],
            "first_last_image_ns": [first_camera, last_camera],
            "first_last_imu_ns": [first_imu, previous_imu],
            "camera_shift_in_source_ns": shift,
            "additional_camera_shift_ns": 0,
            "withheld_frames": withheld,
        })
        if dest.exists() or dest.is_symlink():
            raise FileExistsError(f"Sequence appeared during export: {dest}")
        stage.rename(dest)
    print(f"EXPORT_PASS: {count} images, {cap['imu_samples']} IMU samples. {dest}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("take", type=Path)
    parser.add_argument("out", type=Path, help="new EuRoC sequence directory")
    args = parser.parse_args()
    export_take(args.take.expanduser().resolve(), args.out.expanduser().absolute())


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(str(error)) from error
