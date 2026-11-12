#!/usr/bin/env python3
"""Exercise packaged Linux ARM64 capture helpers with a fake V4L2 device.

Run inside the prepared ARM64 build container after packaging:
    python3 native_timing_test.py
No real camera is accessed. The test compiles only its LD_PRELOAD fake device,
extracts the two packaged capture executables into a temporary directory, and
does not change project source or installed files.
"""
import argparse
import csv
import io
import os
from pathlib import Path
import platform
import struct
import subprocess
import sys
import tarfile
import tempfile

MOCK_SOURCE = b'#define _GNU_SOURCE\n#include <dlfcn.h>\n#include <fcntl.h>\n#include <errno.h>\n#include <stdlib.h>\n#include <stdio.h>\n#include <linux/videodev2.h>\n#include <poll.h>\n#include <stdarg.h>\n#include <stdint.h>\n#include <string.h>\n#include <sys/ioctl.h>\n#include <sys/mman.h>\n#include <unistd.h>\n\nstatic int camera_fd = -1;\nstatic unsigned frame;\nstatic int channel_b;\nstatic int width = 960, height = 721, stride = 2048;\nstatic int mode_set, fps_set;\nstatic unsigned dq_count, stream_on_count, stream_off_count;\n__attribute__((destructor)) static void audit(void) {\n    const char *path = getenv("FAKE_IOCTL_AUDIT");\n    if (!path) return;\n    FILE *out = fopen(path, "w");\n    if (!out) abort();\n    fprintf(out, "%u %u %u\\n", dq_count, stream_on_count, stream_off_count);\n    fclose(out);\n}\nstatic size_t frame_bytes(void) { return (size_t)stride * height; }\nstatic void fmt(struct v4l2_pix_format *p) {\n    p->width = width;\n    p->height = height;\n    p->pixelformat = V4L2_PIX_FMT_UYVY;\n    p->bytesperline = stride;\n    p->sizeimage = frame_bytes();\n    p->field = V4L2_FIELD_NONE;\n}\n\nint open(const char *path, int flags, ...) {\n    int (*real_open)(const char *, int, ...) = dlsym(RTLD_NEXT, "open");\n    if (!strcmp(path, "/dev/video6") || !strcmp(path, "/dev/gmsl/B")) {\n        const char *expected = getenv("FAKE_EXPECT_PATH");\n        if (!expected || strcmp(expected, path)) abort();\n        channel_b = !strcmp(path, "/dev/gmsl/B");\n        camera_fd = real_open("/dev/null", flags);\n        return camera_fd;\n    }\n    if (flags & O_CREAT) {\n        va_list ap;\n        va_start(ap, flags);\n        int mode = va_arg(ap, int);\n        va_end(ap);\n        return real_open(path, flags, mode);\n    }\n    return real_open(path, flags);\n}\n\nvoid *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t offset) {\n    void *(*real_mmap)(void *, size_t, int, int, int, off_t) = dlsym(RTLD_NEXT, "mmap");\n    if (camera_fd >= 0 && fd == camera_fd) {\n        void *p = real_mmap(addr, len, prot, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);\n        if (p != MAP_FAILED) {\n            memset(p, 253, len);\n            for (int y = 0; y < height; ++y)\n                for (int x = 0; x < width; ++x)\n                    ((uint8_t *)p)[y * stride + x * 2 + 1] = (3 * y + x) % 251;\n        }\n        return p;\n    }\n    return real_mmap(addr, len, prot, flags, fd, offset);\n}\n\nint poll(struct pollfd *fds, nfds_t n, int timeout) {\n    int (*real_poll)(struct pollfd *, nfds_t, int) = dlsym(RTLD_NEXT, "poll");\n    if (n != 1 || fds[0].fd != camera_fd) return real_poll(fds, n, timeout);\n    const char *maximum = getenv("FAKE_MAX_FRAMES");\n    if (maximum && frame >= strtoul(maximum, NULL, 10)) {\n        fds[0].revents = 0;\n        return 0;\n    }\n    usleep(1000);\n    fds[0].revents = POLLIN;\n    return 1;\n}\n\nint ioctl(int fd, unsigned long request, ...) {\n    va_list ap;\n    va_start(ap, request);\n    void *arg = va_arg(ap, void *);\n    va_end(ap);\n    if (fd != camera_fd) {\n        int (*real_ioctl)(int, unsigned long, ...) = dlsym(RTLD_NEXT, "ioctl");\n        return real_ioctl(fd, request, arg);\n    }\n    if (request == VIDIOC_QUERYCAP) {\n        struct v4l2_capability *p = arg;\n        strcpy((char *)p->driver, "tegra-video");\n        int as_b = channel_b && !getenv("FAKE_WRONG_IDENTITY");\n        strcpy((char *)p->card, as_b ? "vi-output, pnd_gmsl_mc 10-003b" : "vi-output, pnd_gmsl_mc 10-003c");\n        strcpy((char *)p->bus_info, as_b ? "platform:tegra-capture-vi:0" : "platform:tegra-capture-vi:2");\n    } else if (request == VIDIOC_G_FMT) {\n        struct v4l2_pix_format *p = &((struct v4l2_format *)arg)->fmt.pix;\n        fmt(p);\n    } else if (request == VIDIOC_S_FMT) {\n        struct v4l2_pix_format *p = &((struct v4l2_format *)arg)->fmt.pix;\n        if (p->bytesperline || p->sizeimage || p->pixelformat != V4L2_PIX_FMT_UYVY ||\n            !((p->width == 1920 && p->height == 1441) || (p->width == 960 && p->height == 721))) abort();\n        if (getenv("FAKE_WRONG_IDENTITY") || (p->width == 960 && !channel_b)) abort();\n        width = p->width;\n        height = p->height;\n        stride = width == 960 ? 2048 : 3840;\n        fmt(p);\n        mode_set = 1;\n    } else if (request == VIDIOC_G_PARM) {\n        struct v4l2_captureparm *p = &((struct v4l2_streamparm *)arg)->parm.capture;\n        p->capability = V4L2_CAP_TIMEPERFRAME;\n        p->timeperframe.numerator = 1;\n        p->timeperframe.denominator = 5;\n    } else if (request == VIDIOC_S_PARM) {\n        struct v4l2_captureparm *p = &((struct v4l2_streamparm *)arg)->parm.capture;\n        if (p->timeperframe.numerator != 1 || p->timeperframe.denominator != 30) abort();\n        fps_set = 1;\n    } else if (request == VIDIOC_STREAMON) {\n        ++stream_on_count;\n        if (!mode_set || !fps_set) abort();\n    } else if (request == VIDIOC_STREAMOFF) {\n        ++stream_off_count;\n    } else if (request == VIDIOC_QUERYBUF) {\n        ((struct v4l2_buffer *)arg)->length = frame_bytes();\n    } else if (request == VIDIOC_DQBUF) {\n        ++dq_count;\n        struct v4l2_buffer *p = arg;\n        uint64_t tick, hz;\n        __asm__ __volatile__("mrs %0, cntvct_el0" : "=r"(tick));\n        __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(hz));\n        uint64_t us = tick/hz*1000000 + tick%hz*1000000/hz - 20000;\n        p->index = frame % 4;\n        p->sequence = frame++;\n        p->bytesused = getenv("FAKE_SHORT_IMAGE") ? (size_t)(height-1)*stride+width*2-1 : frame_bytes();\n        p->timestamp.tv_sec = us/1000000;\n        p->timestamp.tv_usec = us%1000000;\n    }\n    return 0;\n}\n'
WIDTH, HEIGHT = 1920, 1440
RAW_BYTES = 3840 * 1441
GRAY_BYTES = WIDTH * HEIGHT
FRAMES = 3
FIELDS = (
    "sequence,driver_timestamp_ns,camera_raw_ns,dqbuf_raw_ns,gray_start_raw_ns,"
    "gray_done_raw_ns,write_start_raw_ns,write_end_raw_ns,bytesused,flags"
).split(",")


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def read_packets(data, sequence, baseline):
    packet_size = 8 + (4 if sequence else 0) + GRAY_BYTES
    require(len(data) == FRAMES * packet_size,
            f"Expected {FRAMES} binary packets, got {len(data)} bytes")
    events = []
    for index in range(FRAMES):
        offset = index * packet_size
        event, = struct.unpack_from("<q", data, offset)
        require(event > 0 and (not events or event > events[-1]),
                f"Non-increasing camera timestamp in packet {index}")
        events.append(event)
        offset += 8
        if sequence:
            seq, = struct.unpack_from("<I", data, offset)
            require(seq == index, f"Wrong sequence in packet {index}: {seq}")
            offset += 4
        require(data[offset:offset + GRAY_BYTES] == baseline,
                f"Changed crop, stride, grayscale pixels, or wire layout in packet {index}")
    return events


def check_timing(path, events):
    reader = csv.DictReader(io.StringIO(path.read_text()))
    require(reader.fieldnames == FIELDS,
            f"Unexpected timing CSV column order: {reader.fieldnames}")
    rows = [{key: int(value) for key, value in row.items()} for row in reader]
    require(len(rows) == FRAMES, f"Expected {FRAMES} timing rows; got {len(rows)}")
    previous_end = 0
    for index, (row, event) in enumerate(zip(rows, events)):
        require(row["sequence"] == index, f"Timing sequence mismatch in row {index}")
        require(row["camera_raw_ns"] == event,
                f"Timing event differs from binary packet {index}")
        require(row["driver_timestamp_ns"] > 0 and row["driver_timestamp_ns"] % 1000 == 0,
                f"Invalid microsecond-resolution driver timestamp in row {index}")
        require(row["bytesused"] == RAW_BYTES and row["flags"] == 0,
                f"Incorrect driver byte/flag metadata in row {index}")
        ordered = [row[name] for name in (
            "camera_raw_ns", "dqbuf_raw_ns", "gray_start_raw_ns",
            "gray_done_raw_ns", "write_start_raw_ns", "write_end_raw_ns",
        )]
        require(all(a <= b for a, b in zip(ordered, ordered[1:])),
                f"Event/receipt/grayscale/write times out of order in row {index}: {ordered}")
        require(row["dqbuf_raw_ns"] >= previous_end,
                f"Next receipt precedes the preceding completed write in row {index}")
        require(0 < row["dqbuf_raw_ns"] - row["camera_raw_ns"] < 250_000_000,
                f"Fake driver's event-to-receipt lag not represented in row {index}")
        previous_end = row["write_end_raw_ns"]


def extract_helpers(archive, target):
    binary_dir = target / "mono_imu_vio" / "bin"
    binary_dir.mkdir(parents=True)
    with tarfile.open(archive, "r:gz") as source:
        for binary in ("camera_live", "camera_umi"):
            member = source.getmember(f"mono_imu_vio/bin/{binary}")
            require(member.isfile(), f"Packaged {binary} must be a regular executable")
            data = source.extractfile(member).read()
            require(data[:4] == b"\x7fELF" and data[4:6] == b"\x02\x01" and
                    struct.unpack_from("<H", data, 18)[0] == 183,
                    f"Packaged {binary} is not a little-endian ARM64 ELF")
            output = binary_dir / binary
            output.write_bytes(data)
            output.chmod(0o755)
    return binary_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=Path("mono_imu_vio.tar.gz"))
    parser.add_argument("--cc", default="gcc-11")
    args = parser.parse_args()
    require(platform.system() == "Linux" and platform.machine() in ("aarch64", "arm64"),
            "Run this test inside the Linux ARM64 build container")
    require(args.archive.is_file(), f"Package not found: {args.archive}")
    baseline = b"".join(bytes((3 * y + x) % 251 for x in range(WIDTH))
                        for y in range(1, HEIGHT + 1))
    with tempfile.TemporaryDirectory(prefix="name-test-native-timing-") as temp:
        work = Path(temp)
        mock = work / "mock.c"
        mock.write_bytes(MOCK_SOURCE)
        preload = work / "mock.so"
        subprocess.run([args.cc, "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror",
                        str(mock), "-ldl", "-o", str(preload)], check=True, cwd=work)
        binaries = extract_helpers(args.archive, work)
        environment = {**os.environ, "LD_PRELOAD": str(preload), "FAKE_MAX_FRAMES": str(FRAMES)}
        for name in tuple(environment):
            if name.startswith("FAKE_") and name != "FAKE_MAX_FRAMES":
                del environment[name]
        environment.pop("VIO_TIMING", None)
        for binary, sequence, expected_device in (
            ("camera_live", False, "/dev/gmsl/B"),
            ("camera_umi", True, "/dev/gmsl/B"),
        ):
            for timing in ("1", None, "0"):
                label = "unset" if timing is None else timing
                run = work / f"{binary}_timing_{label}"
                run.mkdir()
                env = {**environment, "FAKE_EXPECT_PATH": expected_device,
                       "FAKE_IOCTL_AUDIT": str(run / "audit.txt")}
                if timing is not None:
                    env["VIO_TIMING"] = timing
                capture = subprocess.run([str(binaries / binary)], cwd=run, env=env,
                                         capture_output=True, timeout=20)
                require(capture.returncode == 1 and b"Camera stalled, poll=0" in capture.stderr,
                        f"{binary}/{label}: fake finite stream did not end as expected: "
                        f"exit={capture.returncode} stderr={capture.stderr!r}")
                require(b"requested_fps=30" in capture.stderr,
                        f"{binary}/{label}: expected mode/30 fps request was lost")
                events = read_packets(capture.stdout, sequence, baseline)
                require((run / "audit.txt").read_text().strip() == f"{FRAMES} 1 1",
                        f"{binary}/{label}: unexpected DQBUF/STREAMON/STREAMOFF counts")
                csv_path = run / "camera_timing.csv"
                if timing == "1":
                    require(csv_path.is_file(), f"{binary}: timing CSV missing")
                    check_timing(csv_path, events)
                else:
                    require(not csv_path.exists(), f"{binary}: disabled timing created a CSV")
                print(f"CAPTURE_PACKET_AND_TIMING_PASS binary={binary} VIO_TIMING={label}")

            run = work / f"{binary}_short_frame"
            run.mkdir()
            short = subprocess.run(
                [str(binaries / binary)], cwd=run,
                env={**environment, "VIO_TIMING": "1", "FAKE_EXPECT_PATH": expected_device,
                     "FAKE_SHORT_IMAGE": "1", "FAKE_IOCTL_AUDIT": str(run / "audit.txt")},
                capture_output=True, timeout=10,
            )
            require(short.returncode == 1 and not short.stdout and
                    b"Invalid camera frame at sequence 0." in short.stderr,
                    f"{binary}: short frame no longer fails without emitting a packet: {short.stderr!r}")
            require((run / "audit.txt").read_text().strip() == "1 1 1",
                    f"{binary}: short frame retried or streaming cleanup changed")
            require((run / "camera_timing.csv").read_text().splitlines() == [",".join(FIELDS)],
                    f"{binary}: rejected short frame incorrectly received completed timing data")
            print(f"SHORT_FRAME_FAIL_WITHOUT_RETRY_PASS binary={binary}")
    print("NATIVE_ARM64_TIMING_TEST_PASS cases=8")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"NATIVE_ARM64_TIMING_TEST_FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
