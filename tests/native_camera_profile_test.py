#!/usr/bin/env python3
"""Native ARM64 profile/NEON/V4L2 regression. Uses temporary files and a fake device.

python3 tests/native_camera_profile_test.py [--root SOURCE_ROOT]
No real camera is opened. Run on Linux ARM64 (including the local build container).
--gray-only runs the pixel boundary harness on any native ARM64 host.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import tempfile

GRAY_TEST = r'''
#define _GNU_SOURCE
#include "gray.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
static void need(int ok) { if (!ok) { fputs("gray boundary test failed\n", stderr); exit(1); } }
static unsigned char *guarded(size_t size, void **mapping, size_t *length) {
    size_t page = (size_t)sysconf(_SC_PAGESIZE);
    size_t accessible = (size + page - 1) / page * page;
    *length = accessible + 2 * page;
    *mapping = mmap(NULL, *length, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    need(*mapping != MAP_FAILED);
    unsigned char *start = (unsigned char *)*mapping + page;
    need(!mprotect(start, accessible, PROT_READ | PROT_WRITE));
    return start + accessible - size;
}
static void run(size_t width, size_t height, size_t padding, size_t top, size_t bottom) {
    struct gray_layout g;
    need(!gray_layout_init(&g, width, height, width * 2 + padding, top, bottom));
    void *sm, *dm; size_t sl, dl;
    unsigned char *src = guarded(g.minimum_source_bytes, &sm, &sl);
    unsigned char *dst = guarded(g.pixels, &dm, &dl);
    memset(src, 253, g.minimum_source_bytes);
    for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < width; ++x) src[y * g.stride + 2 * x + 1] = (unsigned char)((x + y * 3) % 251);
    memset(dst, 0xa7, g.pixels);
    need(extract_gray(dst, g.pixels, src, g.minimum_source_bytes - 1, g.minimum_source_bytes, &g) == -1);
    need(extract_gray(dst, g.pixels, src, g.minimum_source_bytes, g.minimum_source_bytes - 1, &g) == -1);
    need(extract_gray(dst, g.pixels - 1, src, g.minimum_source_bytes, g.minimum_source_bytes, &g) == -1);
    for (size_t i = 0; i < g.pixels; ++i) need(dst[i] == 0xa7);
    need(!extract_gray(dst, g.pixels, src, g.minimum_source_bytes, g.minimum_source_bytes, &g));
    for (size_t y = 0; y < g.height; ++y)
        for (size_t x = 0; x < width; ++x) need(dst[y * width + x] == (x + (y + top) * 3) % 251);
    need(!munmap(sm, sl)); need(!munmap(dm, dl));
}
int main(void) {
    struct gray_layout g;
    need(gray_layout_init(&g, 0, 3, 10, 0, 0) == -1);
    need(gray_layout_init(&g, 17, 3, 34, 0, 0) == -1);
    need(gray_layout_init(&g, 18, 3, 35, 0, 0) == -1);
    need(gray_layout_init(&g, 18, 3, 36, 2, 1) == -1);
    need(gray_layout_init(&g, 18, 3, SIZE_MAX, 0, 0) == -1);
    for (size_t w = 2; w <= 66; w += 2)
        for (size_t pad = 0; pad < 4; ++pad) {
            run(w, 6, pad, 0, 0);
            run(w, 6, pad, 1, 2);
        }
    run(1920, 1536, 0, 0, 0);
    run(960, 720, 128, 0, 0);
    run(1920, 1441, 0, 1, 0);
    puts("GRAY_PROFILE_OK");
    return 0;
}
'''

FAKE_SOURCE = r'''
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
static int camera = -1, width, height, stride;
static unsigned opened, format_calls, fps_calls, on_calls, off_calls, frames, fps_control_calls;
static int64_t frame_rate;
#define FRAME_RATE_ID 0x009a200b
static int flag(const char *name) { return getenv(name) != NULL; }
static unsigned number(const char *name) { return (unsigned)strtoul(getenv(name), NULL, 10); }
static void set_layout(void) { width = number("VIO_CAMERA_WIDTH"); height = number("VIO_CAMERA_HEIGHT"); stride = width * 2 + 12; }
__attribute__((destructor)) static void audit(void) {
    const char *path = getenv("FAKE_AUDIT");
    if (!path) return;
    FILE *f = fopen(path, "w");
    if (!f) abort();
    fprintf(f, "{\"opened\":%u,\"format\":%u,\"fps\":%u,\"fps_control\":%u,\"on\":%u,\"off\":%u,\"frames\":%u}\n",
            opened, format_calls, fps_calls, fps_control_calls, on_calls, off_calls, frames);
    fclose(f);
}
int open(const char *path, int flags, ...) {
    int (*real_open)(const char *, int, ...) = dlsym(RTLD_NEXT, "open");
    if (!strcmp(path, "/dev/vio-profile-test")) {
        ++opened; set_layout();
        if (!flag("FAKE_MATCH")) { width = 2; height = 2; stride = 16; }
        camera = real_open("/dev/null", flags);
        return camera;
    }
    if (!strncmp(path, "/dev/video", 10) || !strncmp(path, "/dev/gmsl", 9)) abort();
    if (flags & O_CREAT) {
        va_list ap; va_start(ap, flags); int mode = va_arg(ap, int); va_end(ap);
        return real_open(path, flags, mode);
    }
    return real_open(path, flags);
}
void *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t offset) {
    void *(*real_mmap)(void *, size_t, int, int, int, off_t) = dlsym(RTLD_NEXT, "mmap");
    if (fd != camera || camera < 0) return real_mmap(addr, len, prot, flags, fd, offset);
    unsigned char *data = real_mmap(addr, len, prot, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (data != MAP_FAILED) {
        memset(data, 253, len);
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) data[y * stride + 2 * x + 1] = (x + y * 3) % 251;
    }
    return data;
}
int poll(struct pollfd *fds, nfds_t n, int timeout) {
    int (*real_poll)(struct pollfd *, nfds_t, int) = dlsym(RTLD_NEXT, "poll");
    if (n != 1 || fds[0].fd != camera) return real_poll(fds, n, timeout);
    if (frames >= 2) { fds[0].revents = 0; return 0; }
    usleep(20000);
    fds[0].revents = POLLIN;
    return 1;
}
static void fill_format(struct v4l2_pix_format *p) {
    p->width = width; p->height = height;
    p->pixelformat = flag("FAKE_WRONG_PIXELS") ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_UYVY;
    p->field = V4L2_FIELD_NONE;
    p->bytesperline = flag("FAKE_SHORT_STRIDE") ? width * 2 - 1 : stride;
    p->sizeimage = stride * height;
}
int ioctl(int fd, unsigned long op, ...) {
    va_list ap; va_start(ap, op); void *arg = va_arg(ap, void *); va_end(ap);
    if (fd != camera) {
        int (*real_ioctl)(int, unsigned long, ...) = dlsym(RTLD_NEXT, "ioctl");
        return real_ioctl(fd, op, arg);
    }
    if (op == VIDIOC_QUERYCAP) {
        struct v4l2_capability *p = arg;
        strcpy((char *)p->driver, "tegra-video");
        strcpy((char *)p->card, flag("FAKE_WRONG_CARD") ? "wrong-camera" : getenv("VIO_CAMERA_CARD"));
        strcpy((char *)p->bus_info, "platform:tegra-capture-vi:0");
        p->capabilities = V4L2_CAP_DEVICE_CAPS;
        p->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
    } else if (op == VIDIOC_G_FMT) {
        fill_format(&((struct v4l2_format *)arg)->fmt.pix);
    } else if (op == VIDIOC_S_FMT) {
        struct v4l2_pix_format *p = &((struct v4l2_format *)arg)->fmt.pix;
        if (p->bytesperline || p->sizeimage || p->width != number("VIO_CAMERA_WIDTH") ||
            p->height != number("VIO_CAMERA_HEIGHT") || p->pixelformat != V4L2_PIX_FMT_UYVY) abort();
        ++format_calls; set_layout();
        if (flag("FAKE_WRONG_SIZE")) --height;
        fill_format(p);
    } else if (op == VIDIOC_S_PARM) {
        struct v4l2_fract *p = &((struct v4l2_streamparm *)arg)->parm.capture.timeperframe;
        if (p->numerator != 1 || p->denominator != number("VIO_CAMERA_FPS")) abort();
        ++fps_calls;
        if (flag("FAKE_UNSUPPORTED_FPS")) { errno = EINVAL; return -1; }
    } else if (op == VIDIOC_G_PARM) {
        struct v4l2_captureparm *p = &((struct v4l2_streamparm *)arg)->parm.capture;
        p->capability = V4L2_CAP_TIMEPERFRAME;
        p->timeperframe = (struct v4l2_fract){1, flag("FAKE_UNSUPPORTED_FPS") ? 30 : number("VIO_CAMERA_FPS")};
    } else if (op == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *p = arg;
        if (!flag("VIO_CAMERA_FPS_CONTROL") || fps_calls != 1 || format_calls != 1 ||
                fps_control_calls++ || p->id != FRAME_RATE_ID) abort();
        strcpy(p->name, flag("FAKE_BAD_FPS_CONTROL") ? "Gain" : "Frame Rate");
        p->type = V4L2_CTRL_TYPE_INTEGER64;
        p->minimum = 30000000;
        p->maximum = 60000000;
        p->step = 1000000;
    } else if (op == VIDIOC_S_EXT_CTRLS || op == VIDIOC_G_EXT_CTRLS) {
        struct v4l2_ext_controls *p = arg;
        if (!flag("VIO_CAMERA_FPS_CONTROL") || p->count != 1 ||
                p->ctrl_class != V4L2_CTRL_ID2CLASS(FRAME_RATE_ID) ||
                p->controls[0].id != FRAME_RATE_ID ||
                fps_control_calls++ != (unsigned)(op == VIDIOC_S_EXT_CTRLS ? 1 : 2)) abort();
        if (op == VIDIOC_S_EXT_CTRLS) {
            frame_rate = p->controls[0].value64;
            if (frame_rate != (int64_t)number("VIO_CAMERA_FPS") * 1000000) abort();
        } else {
            p->controls[0].value64 = flag("FAKE_BAD_FPS_READBACK") ? 30000000 : frame_rate;
        }
    } else if (op == VIDIOC_REQBUFS) {
        ((struct v4l2_requestbuffers *)arg)->count = 4;
    } else if (op == VIDIOC_QUERYBUF) {
        ((struct v4l2_buffer *)arg)->length = stride * height - flag("FAKE_SHORT_MAP");
    } else if (op == VIDIOC_QBUF) {
    } else if (op == VIDIOC_STREAMON) {
        if (fps_calls != 1 || fps_control_calls != (flag("VIO_CAMERA_FPS_CONTROL") ? 3u : 0u)) abort();
        ++on_calls;
    } else if (op == VIDIOC_STREAMOFF) {
        ++off_calls;
    } else if (op == VIDIOC_DQBUF) {
        struct v4l2_buffer *p = arg;
        uint64_t ns;
        if (!strcmp(getenv("VIO_CAMERA_CLOCK"), "cntvct")) {
            uint64_t tick, hz;
            __asm__ __volatile__("mrs %0, cntvct_el0" : "=r"(tick));
            __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(hz));
            ns = tick / hz * 1000000000ULL + tick % hz * 1000000000ULL / hz;
        } else {
            struct timespec ts;
            clock_gettime(!strcmp(getenv("VIO_CAMERA_CLOCK"), "raw") ? CLOCK_MONOTONIC_RAW : CLOCK_MONOTONIC, &ts);
            ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        }
        ns -= 10000000;
        p->timestamp.tv_sec = ns / 1000000000ULL;
        p->timestamp.tv_usec = ns % 1000000000ULL / 1000;
        p->index = flag("FAKE_BAD_INDEX") ? 99 : frames % 4;
        p->sequence = frames++;
        p->bytesused = flag("FAKE_SHORT_FRAME") ? (height - 1) * stride + width * 2 - 1 :
                      flag("FAKE_MINIMUM_FRAME") ? (height - 1) * stride + width * 2 : height * stride;
        if (flag("FAKE_OVERSIZE_FRAME")) ++p->bytesused;
        if (flag("FAKE_FRAME_ERROR")) p->flags |= V4L2_BUF_FLAG_ERROR;
    } else abort();
    return 0;
}
'''

TIMING_FIELDS = (
    "sequence,driver_timestamp_ns,camera_raw_ns,dqbuf_raw_ns,gray_start_raw_ns,"
    "gray_done_raw_ns,write_start_raw_ns,write_end_raw_ns,bytesused,flags"
).split(",")


def require(ok, message):
    if not ok:
        raise AssertionError(message)


def compile_file(cc, source, output, includes, flags=()):
    subprocess.run([cc, "-O3", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(includes), *flags, str(source), "-o", str(output)], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--gray-only", action="store_true")
    args = parser.parse_args()
    require(platform.machine().lower() in ("aarch64", "arm64"), "Requires native ARM64 NEON")
    require(args.gray_only or platform.system() == "Linux", "Capture tests require Linux")
    cc = shutil.which("gcc-11") or shutil.which("cc")
    require(cc, "C compiler unavailable")
    include = args.root / "adapter/orb_replay"
    with tempfile.TemporaryDirectory(prefix="vio-camera-profile-") as name:
        temp = Path(name)
        gray_source = temp / "gray_test.c"
        gray_source.write_text(GRAY_TEST)
        compile_file(cc, gray_source, temp / "gray_test", include)
        subprocess.run([str(temp / "gray_test")], check=True)
        if args.gray_only:
            return
        for binary, flags in (("camera_live", ()), ("camera_umi", ("-DORB_CAMERA_SEQUENCE",))):
            compile_file(cc, include / "camera_live.c", temp / binary, include, flags)
        fake_source = temp / "fake.c"
        fake_source.write_text(FAKE_SOURCE)
        compile_file(cc, fake_source, temp / "fake.so", include, ("-shared", "-fPIC", "-ldl"))
        base = {key: value for key, value in os.environ.items()
                if not key.startswith(("VIO_CAMERA_", "FAKE_")) and key not in ("LD_PRELOAD", "VIO_TIMING")}
        base.update(VIO_CAMERA_DEVICE="/dev/vio-profile-test", VIO_CAMERA_CARD="vi-output, jl_gmsl 10-003a",
                    VIO_CAMERA_WIDTH="18", VIO_CAMERA_HEIGHT="6", VIO_CAMERA_CROP_TOP="0",
                    VIO_CAMERA_CROP_BOTTOM="0", VIO_CAMERA_FPS="60", VIO_CAMERA_CLOCK="raw")

        def invoke(binary="camera_live", extra=None, remove=(), describe=False):
            folder = temp / f"case_{len(list(temp.glob('case_*'))):03}"
            folder.mkdir()
            env = {**base, "LD_PRELOAD": str(temp / "fake.so"), "FAKE_AUDIT": str(folder / "audit.json"),
                   **(extra or {})}
            for key in remove:
                env.pop(key, None)
            result = subprocess.run([str(temp / binary), *(["--describe"] if describe else [])],
                                    env=env, cwd=folder, capture_output=True, timeout=10)
            audit = json.loads((folder / "audit.json").read_text())
            return result, audit, folder

        for clock in ("raw", "monotonic", "cntvct"):
            result, audit, _ = invoke(extra={"VIO_CAMERA_CLOCK": clock}, describe=True)
            require(result.returncode == 0, result.stderr.decode())
            profile = json.loads(result.stdout)
            require(profile["output_height"] == 6 and profile["requested_fps"] == 60 and
                    profile["clock_domain"] == clock and profile["bus"] is None, "describe contract")
            require(audit["opened"] == 0, "--describe touched camera")
        for key in [key for key in base if key.startswith("VIO_CAMERA_")]:
            result, audit, _ = invoke(remove=(key,), describe=True)
            require(result.returncode == 2 and not result.stdout and audit["opened"] == 0,
                    f"Missing {key} did not fail before hardware")
        for extra in ({"VIO_CAMERA_CROP_TOP": "6"}, {"VIO_CAMERA_CROP_BOTTOM": "6"},
                      {"VIO_CAMERA_CROP_TOP": "4", "VIO_CAMERA_CROP_BOTTOM": "2"},
                      {"VIO_CAMERA_WIDTH": "17"}, {"VIO_CAMERA_WIDTH": "99999999999999999999"},
                      {"VIO_CAMERA_FPS": "0"}, {"VIO_CAMERA_FPS": "-1"}, {"VIO_CAMERA_FPS": "241"},
                      {"VIO_CAMERA_CLOCK": "auto"}, {"VIO_CAMERA_WIDTH": " 18"}):
            result, audit, _ = invoke(extra=extra, describe=True)
            require(result.returncode == 2 and audit["opened"] == 0, f"Invalid profile accepted: {extra}")

        # All clock domains, both wire variants, matching/changing formats, and explicit crops.
        successful = 0
        for clock in ("raw", "monotonic", "cntvct"):
            for match in (False, True):
                for crop in ((0, 0), (1, 2)):
                    images = []
                    for binary in ("camera_live", "camera_umi"):
                        extra = {"VIO_CAMERA_CLOCK": clock, "VIO_CAMERA_CROP_TOP": str(crop[0]),
                                 "VIO_CAMERA_CROP_BOTTOM": str(crop[1]), "VIO_TIMING": "1"}
                        if match:
                            extra["FAKE_MATCH"] = "1"
                            extra["FAKE_MINIMUM_FRAME"] = "1"
                        result, audit, folder = invoke(binary, extra)
                        require(result.returncode == 1 and audit["frames"] == 2 and audit["on"] == audit["off"] == 1,
                                result.stderr.decode())
                        require(audit["fps"] == 1 and audit["format"] == int(not match), "format/fps calls")
                        expected = bytes((x + y * 3) % 251 for y in range(crop[0], 6-crop[1]) for x in range(18))
                        header = 12 if binary == "camera_umi" else 8
                        size = header + len(expected)
                        require(len(result.stdout) == 2 * size, "wire packet sizes")
                        stamps = []
                        for i in range(2):
                            stamp, = struct.unpack_from("<q", result.stdout, i * size)
                            stamps.append(stamp)
                            if header == 12:
                                require(struct.unpack_from("<I", result.stdout, i * size + 8)[0] == i, "sequence")
                            require(result.stdout[i*size+header:(i+1)*size] == expected, "crop/padding/NEON pixels")
                        require(0 < stamps[0] < stamps[1], "camera timestamp order")
                        with (folder / "camera_timing.csv").open() as file:
                            reader = csv.DictReader(file)
                            require(reader.fieldnames == TIMING_FIELDS, "timing CSV contract changed")
                            rows = list(reader)
                        require([int(r["camera_raw_ns"]) for r in rows] == stamps, "CSV/wire event mismatch")
                        for row in rows:
                            require(0 <= int(row["gray_done_raw_ns"]) - int(row["gray_start_raw_ns"]), "negative gray latency")
                            lag = int(row["dqbuf_raw_ns"]) - int(row["camera_raw_ns"])
                            require(5_000_000 <= lag < 100_000_000, "configured driver clock mapping")
                            if clock == "raw":
                                require(row["driver_timestamp_ns"] == row["camera_raw_ns"], "RAW mapping is not identity")
                        images.append(expected)
                        successful += 1
                    require(images[0] == images[1], "live/record pixels differ")

        result, audit, _ = invoke(extra={"FAKE_UNSUPPORTED_FPS": "1", "FAKE_MATCH": "1"})
        require(audit["fps"] == 1 and audit["frames"] == 2 and b"interval_request=unsupported" in result.stderr and
                b"requested_fps=60" in result.stderr and b"reported_fps=30." in result.stderr,
                "Unsupported interval must clearly report requested versus actual")
        tegra = {"VIO_CAMERA_FPS_CONTROL": "tegra_frame_rate_microhz", "FAKE_MATCH": "1",
                 "FAKE_UNSUPPORTED_FPS": "1"}
        for binary in ("camera_live", "camera_umi"):
            result, audit, _ = invoke(binary, tegra)
            require(audit["fps"] == audit["format"] == audit["on"] == 1 and audit["fps_control"] == 3 and
                    audit["frames"] == 2 and b"readback_microhz=60000000" in result.stderr,
                    "60 fps request was not written and read back before streaming")
        for failure, calls, message in (("FAKE_BAD_FPS_CONTROL", 1, b"Unexpected Tegra frame_rate control"),
                                        ("FAKE_BAD_FPS_READBACK", 3, b"Tegra frame_rate readback=30000000")):
            result, audit, _ = invoke(extra={**tegra, failure: "1"})
            require(result.returncode == 1 and not result.stdout and audit["on"] == 0 and
                    audit["fps_control"] == calls and message in result.stderr,
                    f"Wrong fps control did not prevent CPU acquisition: {failure}")
        for failure in ("FAKE_WRONG_CARD", "FAKE_WRONG_SIZE", "FAKE_WRONG_PIXELS",
                        "FAKE_SHORT_STRIDE", "FAKE_SHORT_MAP", "FAKE_SHORT_FRAME",
                        "FAKE_OVERSIZE_FRAME", "FAKE_BAD_INDEX", "FAKE_FRAME_ERROR"):
            result, audit, _ = invoke(extra={failure: "1"})
            require(result.returncode == 1 and not result.stdout, f"Invalid frame/setup escaped: {failure}")
            if failure == "FAKE_WRONG_CARD":
                require(audit["format"] == audit["fps"] == audit["on"] == 0, "Wrong identity mutated device")
        result, audit, _ = invoke(extra={"VIO_CAMERA_BUS": "wrong-bus"})
        require(result.returncode == 1 and audit["format"] == audit["fps"] == audit["on"] == 0,
                "Bus identity mismatch did not stop before configuration")
        print(f"CAMERA_PROFILE_OK valid_capture_cases={successful} invalid_profile_and_driver_cases=checked")


if __name__ == "__main__":
    main()
