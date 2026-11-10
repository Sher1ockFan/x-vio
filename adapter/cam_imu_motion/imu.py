#!/usr/bin/env python3
"""Passive IMU receiver for a Jetson motion trial. Usage: imu.py OUTDIR SECONDS."""
import argparse
import bisect
import collections
import copy
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import stat
import statistics
import struct
import subprocess
import sys
import tempfile
import termios
import time
import zlib

DEVICE = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SYNC = b"\xaa\x55"
AHRS_HEADER = SYNC + struct.pack("<HH", 2, 44)
FRAME_BYTES = 54
STREAM_TIMEOUT = 3.0
SENSOR_FIELDS = ("ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s",
                 "gz_rad_s", "acc_norm_m_s2", "gyro_norm_rad_s", "temperature_c")
OUTPUT_NAMES = ("imu_raw.bin", "chunks.csv", "imu_si.csv", "imu_summary.json", "imu_ready")


def crc32_forsense(data):
    # Vendor algorithm: reflected polynomial 0xEDB88320, seed 1, no final XOR.
    # zlib complements its input/output state; undo both to match that algorithm.
    return (zlib.crc32(data, 0xFFFFFFFE) ^ 0xFFFFFFFF) & 0xFFFFFFFF


def check_owners(device, info, log):
    fuser = shutil.which("fuser")
    if fuser:
        try:
            r = subprocess.run([fuser, "-v", str(device)], capture_output=True,
                               text=True, timeout=5, check=False)
            log("fuser exit=%s stdout=%r stderr=%r" % (r.returncode, r.stdout.strip(), r.stderr.strip()))
            pids = [int(x) for x in r.stdout.split() if x.isdigit()]
            if pids or r.returncode == 0:
                raise RuntimeError("Serial device has a visible owner. Close other readers first: %s" % pids)
            if r.returncode == 1 and not r.stderr.strip():
                return
        except (OSError, subprocess.TimeoutExpired) as exc:
            log("fuser unavailable/inconclusive: " + str(exc))
    matches = set()
    inaccessible = 0
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            with os.scandir(process / "fd") as entries:
                for entry in entries:
                    try:
                        other = os.stat(entry.path)
                    except OSError:
                        continue
                    if stat.S_ISCHR(other.st_mode) and other.st_rdev == info.st_rdev:
                        matches.add(int(process.name))
                        break
        except PermissionError:
            inaccessible += 1
        except (FileNotFoundError, ProcessLookupError):
            continue
    log("Visible matching PIDs=%s; unreadable process fd directories=%d" % (sorted(matches), inaccessible))
    if matches:
        raise RuntimeError("Close other serial readers before running this probe.")


def host_settings(original, baud):
    speed = getattr(termios, "B%d" % baud, None)
    if speed is None:
        raise ValueError("This Python/Linux build does not expose B%d" % baud)
    a = copy.deepcopy(original)
    a[0] = 0
    a[1] = 0
    a[2] &= ~(termios.CSIZE | termios.PARENB | termios.PARODD | termios.CSTOPB | termios.HUPCL)
    a[2] &= ~(getattr(termios, "CRTSCTS", 0) | getattr(termios, "CMSPAR", 0))
    a[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = speed
    a[6][termios.VMIN] = a[6][termios.VTIME] = 0
    return a


def first_valid_sample(window):
    """Find a complete CRC-valid AHRS frame in recent bytes (including a partial previous read)."""
    pos = 0
    while True:
        pos = window.find(AHRS_HEADER, pos)
        if pos < 0 or pos + FRAME_BYTES > len(window):
            return None
        end = pos + FRAME_BYTES
        if crc32_forsense(window[pos:end-4]) == struct.unpack_from("<I", window, end-4)[0]:
            return pos
        pos += 1


def mark_ready(directory, offset, host_ns):
    # Link only after the complete marker has been written; never replace another file.
    with tempfile.NamedTemporaryFile(mode="w", dir=directory, delete=False) as marker:
        try:
            json.dump({"first_valid_offset": offset, "host_packet_complete_raw_ns": host_ns}, marker)
            marker.flush()
            os.link(marker.name, directory/"imu_ready")
        finally:
            os.unlink(marker.name)


def record(fd, directory, seconds, log):
    with (directory / "imu_raw.bin").open("xb") as binary, \
         (directory / "chunks.csv").open("x", newline="") as chunkfile:
        writer = csv.writer(chunkfile)
        writer.writerow(("end_byte_exclusive", "host_raw_after_read_ns"))
        start = time.monotonic()
        deadline = start + seconds
        next_update = start + 5
        last_valid = None
        suffix = b""
        offset = 0
        while True:
            now = time.monotonic()
            watch_deadline = (start if last_valid is None else last_valid) + STREAM_TIMEOUT
            if now >= watch_deadline and watch_deadline <= deadline:
                phase = "no first valid AHRS packet" if last_valid is None else "valid AHRS stream stalled"
                raise TimeoutError(f"{phase} for {STREAM_TIMEOUT:g} s; received {offset} bytes")
            if now >= deadline:
                break
            if now >= next_update:
                log(f"Elapsed {now-start:.1f} s, received {offset} bytes")
                next_update = now + 5
            if not select.select([fd], [], [], min(0.2, deadline-now, watch_deadline-now))[0]:
                continue
            try:
                block = os.read(fd, 65536)
            except BlockingIOError:
                continue
            host_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            if not block:
                raise OSError("Serial EOF/disconnect")
            binary.write(block)
            writer.writerow((offset + len(block), host_ns))
            window = suffix + block
            match = first_valid_sample(window)
            if match is not None:
                if last_valid is None:
                    mark_ready(directory, offset-len(suffix)+match, host_ns)
                    log("IMU ready: first CRC-valid AHRS frame received.")
                last_valid = time.monotonic()
            suffix = window[-(FRAME_BYTES-1):]
            offset += len(block)
        if last_valid is None:
            raise TimeoutError(f"No CRC-valid AHRS packet in {seconds:g} s; received {offset} bytes")
        return time.monotonic() - start


def capture(directory, seconds, log):
    path = Path(DEVICE).resolve(strict=True)
    info = path.stat()
    log(f"Port {path}, permissions {stat.filemode(info.st_mode)}, uid {info.st_uid}, gid {info.st_gid}")
    if not stat.S_ISCHR(info.st_mode):
        raise RuntimeError("The serial link does not resolve to a character device.")
    check_owners(path, info, log)
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    exclusive = False
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        requested = host_settings(original, 921600)
        termios.tcsetattr(fd, termios.TCSANOW, requested)
        if termios.tcgetattr(fd)[4:6] != requested[4:6]:
            raise RuntimeError("Host baud-rate readback failed.")
        log(f"Recording {seconds:g} s at 921600 baud, 8N1.")
        return record(fd, directory, seconds, log)
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
                log("Original host serial settings restored.")
        finally:
            try:
                if exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)


def describe(values):
    if not values:
        return {"n": 0}
    return {"n": len(values), "mean": statistics.fmean(values),
            "sample_std": statistics.stdev(values) if len(values) > 1 else None,
            "min": min(values), "max": max(values)}


def analyze(data, ends, stamps, outpath):
    if len(ends) != len(stamps) or (ends[-1] if ends else 0) != len(data):
        raise ValueError("Raw length and chunk index do not match.")
    if any(a >= b for a, b in zip([0] + ends, ends)):
        raise ValueError("Chunk offsets must strictly increase.")
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Host RAW read timestamps went backward.")
    failed_offsets = []
    counts = collections.Counter()
    columns = {key: [] for key in SENSOR_FIELDS}
    intervals = []
    segments = []
    pos = 0
    covered = 0
    first_start = last_end = last_timer = None
    previous_good = False
    wrap_base = epoch = 0
    with outpath.open("x", newline="") as output:
        w = csv.writer(output)
        w.writerow(("sample", "byte_offset", "timer_us_u32", "clock_epoch",
                    "timer_us_unwrapped_in_epoch", "host_packet_complete_raw_ns",
                    "segment", "flags", *SENSOR_FIELDS))
        while True:
            pos = data.find(SYNC, pos)
            if pos < 0 or len(data)-pos < 6:
                break
            mid, n = struct.unpack_from("<HH", data, pos+2)
            end = pos+10+n
            if end > len(data):
                counts["incomplete_sync_candidates"] += 1
                pos += 1
                continue
            if crc32_forsense(data[pos:end-4]) != struct.unpack_from("<I", data, end-4)[0]:
                counts["bad_crc_candidates"] += 1
                failed_offsets.append(pos)
                pos += 1
                continue
            covered += end-pos
            if mid != 2 or n != 44:
                counts["valid_other_frames"] += 1
                pos = end
                continue
            index = counts["valid_ahrs_frames"]
            counts["valid_ahrs_frames"] += 1
            timer, *v = struct.unpack_from("<I10f", data, pos+6)
            acc = [x*9.80665 for x in v[3:6]]
            gyro = [math.radians(x) for x in v[6:9]]
            values = (*acc, *gyro, math.hypot(*acc), math.hypot(*gyro), v[9])
            good = all(math.isfinite(x) for x in values)
            flags = []
            split = not previous_good
            if not good:
                counts["nonfinite_si_or_temperature"] += 1
                flags.append("nonfinite")
            if first_start is None:
                first_start = pos
            if last_end is not None and pos != last_end:
                flags.append("intervening_bytes")
                counts["byte_discontinuities"] += 1
                counts["bytes_between_valid_ahrs"] += pos-last_end
                split = True
            if last_timer is not None:
                dt = (timer-last_timer) & 0xffffffff
                if timer < last_timer:
                    if dt <= 1_000_000:
                        wrap_base += 1 << 32
                        counts["plausible_u32_wraps"] += 1
                        flags.append("u32_wrap")
                    else:
                        epoch += 1
                        wrap_base = 0
                        counts["backward_or_reset_events"] += 1
                        flags.append("backward_or_reset")
                        split = True
                if not 1500 <= dt <= 2500:
                    counts["irregular_timer_intervals"] += 1
                    if dt == 0:
                        counts["duplicate_timers"] += 1
                    elif 2500 < dt <= 1_000_000 and abs(dt - round(dt/2000)*2000) <= 500:
                        counts["estimated_missing_500hz_samples"] += round(dt/2000)-1
                    flags.append("irregular_dt")
                    split = True
                if good and not split:
                    intervals.append(dt)
            tick = wrap_base + timer
            segment_id = ""
            if good:
                if split:
                    segments.append({"first_frame": index, "count": 0, "duration_us": 0,
                                     "first_tick_us": tick})
                seg = segments[-1]
                seg["count"] += 1
                seg["duration_us"] = tick-seg["first_tick_us"]
                segment_id = len(segments)-1
                for field, x in zip(SENSOR_FIELDS, values):
                    columns[field].append(x)
            host_ns = stamps[bisect.bisect_left(ends, end)]
            w.writerow((index, pos, timer, epoch, tick, host_ns, segment_id,
                        "|".join(flags) or "regular", *values))
            previous_good = good
            last_timer, last_end, pos = timer, end, end
    nonempty = counts["valid_ahrs_frames"] > 0
    prefix_fail = sum(pos < first_start for pos in failed_offsets) if nonempty else 0
    interior_fail = sum(first_start <= pos < last_end for pos in failed_offsets) if nonempty else 0
    suffix_fail = sum(pos >= last_end for pos in failed_offsets) if nonempty else 0
    contiguous_rate = 1e6/statistics.fmean(intervals) if intervals else None
    return {
        "bytes": len(data), "read_chunks": len(ends),
        "first_ahrs_host_raw_ns": stamps[bisect.bisect_left(ends, first_start + FRAME_BYTES)] if nonempty else None,
        "last_ahrs_host_raw_ns": stamps[bisect.bisect_left(ends, last_end)] if nonempty else None,
        "prefix_crc_fail_candidates": prefix_fail, "interior_crc_fail_candidates": interior_fail,
        "suffix_crc_fail_candidates": suffix_fail, "crc_fail_offsets_first_16": failed_offsets[:16],
        "crc_valid_ahrs_frames": counts["valid_ahrs_frames"], "other_crc_valid_frames": counts["valid_other_frames"],
        "crc_fail_candidates": counts["bad_crc_candidates"], "incomplete_sync_candidates": counts["incomplete_sync_candidates"],
        "uncovered_bytes": len(data)-covered, "prefix_before_first_ahrs_bytes": first_start,
        "suffix_after_last_ahrs_bytes": len(data)-last_end if nonempty else None,
        "bytes_between_valid_ahrs": counts["bytes_between_valid_ahrs"], "byte_discontinuities": counts["byte_discontinuities"],
        "plausible_u32_wraps": counts["plausible_u32_wraps"], "reset_or_backward_events": counts["backward_or_reset_events"],
        "irregular_timer_intervals": counts["irregular_timer_intervals"], "duplicate_timers": counts["duplicate_timers"],
        "estimated_missing_500hz_samples": counts["estimated_missing_500hz_samples"],
        "nonfinite_si_or_temperature": counts["nonfinite_si_or_temperature"], "finite_samples": len(columns[SENSOR_FIELDS[0]]),
        "continuous_segments": len(segments), "longest_continuous_samples": max((s["count"] for s in segments), default=0),
        "longest_continuous_elapsed_s": max((s["duration_us"] for s in segments), default=0)/1e6,
        "continuous_device_interval_us": describe(intervals), "observed_hz_on_continuous_intervals": contiguous_rate,
        "signal_statistics": {key: describe(vals) for key, vals in columns.items()},
        "notes": ["The six axes retain the IMU configured output orientation.",
                  "Host packet completion uses CLOCK_MONOTONIC_RAW at the read containing the final CRC byte.",
                  "Segments break at nonfinite data, intervening bytes or device intervals outside 1500 to 2500 us.",
                  "Missing sample estimates assume a 2000 us nominal period. Each row identifies its segment and clock epoch.",
                  "Format: FORSENSE ID 2, 44-byte payload with uint32 timer_us and 10 floats."]
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("outdir", type=Path, help="existing new session directory")
    parser.add_argument("seconds", type=float, help="recording duration, 1 to 110 seconds")
    args = parser.parse_args(argv)
    if not 1 <= args.seconds <= 110:
        parser.error("duration must be between 1 and 110 seconds")
    if not sys.platform.startswith("linux") or os.uname().machine != "aarch64":
        parser.error("Run on the Linux ARM64 Jetson.")
    out = args.outdir.resolve(strict=True)
    if not out.is_dir():
        parser.error("outdir must be an existing session directory")
    occupied = [name for name in OUTPUT_NAMES if os.path.lexists(out/name)]
    if occupied:
        parser.error("output already exists: " + ", ".join(occupied))

    def stop(signum, stack):
        raise KeyboardInterrupt("signal %d" % signum)

    def log(message):
        print(message, flush=True)

    summary = {"device": DEVICE, "host_baud": 921600, "host_framing": "8N1",
               "requested_seconds": args.seconds, "capture_completed": False}
    status = 1
    # Exclusive summary reservation serializes probes using this output directory.
    with (out/"imu_summary.json").open("x") as report:
        old = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
        try:
            try:
                summary["elapsed_host_s"] = capture(out, args.seconds, log)
                summary["capture_completed"] = True
            except (KeyboardInterrupt, OSError, RuntimeError, ValueError, termios.error) as exc:
                summary["capture_error"] = str(exc) or type(exc).__name__
                log("Capture stopped: " + summary["capture_error"])
            if (out/"chunks.csv").is_file() and (out/"imu_raw.bin").is_file():
                with (out/"chunks.csv").open() as source:
                    pairs = [(int(x["end_byte_exclusive"]), int(x["host_raw_after_read_ns"]))
                             for x in csv.DictReader(source)]
                summary.update(analyze((out/"imu_raw.bin").read_bytes(), [p[0] for p in pairs],
                                       [p[1] for p in pairs], out/"imu_si.csv"))
            criteria = {
                "entire_requested_capture_completed": summary["capture_completed"],
                "finite_samples_present": summary.get("finite_samples", 0) > 0,
                "all_ahrs_si_finite": summary.get("nonfinite_si_or_temperature") == 0,
                "one_contiguous_sample_segment": summary.get("continuous_segments") == 1,
                "no_interior_crc_or_byte_gaps": (summary.get("interior_crc_fail_candidates") == 0
                                                 and summary.get("byte_discontinuities") == 0),
                "no_timer_discontinuity": summary.get("irregular_timer_intervals") == 0,
                "observed_device_rate_490_to_510_hz": 490 <= (summary.get("observed_hz_on_continuous_intervals") or 0) <= 510,
            }
            summary["checks"] = criteria
            summary["imu_stream_ok"] = all(criteria.values())
            log(f"CRC-valid AHRS frames: {summary.get('crc_valid_ahrs_frames', 0)}; checks {criteria}")
            if summary["imu_stream_ok"]:
                status = 0
            elif summary["capture_completed"] and summary.get("finite_samples", 0):
                status = 2
        except (KeyboardInterrupt, OSError, ValueError, RuntimeError) as exc:
            summary["analysis_error"] = str(exc) or type(exc).__name__
            log("Analysis stopped: " + summary["analysis_error"])
        finally:
            for s, h in old.items():
                signal.signal(s, h)
            summary["exit_status"] = status
            json.dump(summary, report, indent=2, allow_nan=False)
            report.write("\n")
    return status


if __name__ == "__main__":
    sys.exit(main())

