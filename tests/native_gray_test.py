#!/usr/bin/env python3
"""Check the final package's actual NEON extraction on native Linux ARM64.

Run after packaging inside the prepared ARM64 GCC 11 build container:
    python3 native_gray_test.py
Only a temporary harness is compiled. No camera is opened and no project source
or installed binary is modified. This is a correctness test, not a benchmark.
"""
import argparse
from pathlib import Path
import platform
import subprocess
import sys
import tarfile
import tempfile


HARNESS_SOURCE = r'''#define CAMERA_PROBE_PIXEL_ONLY
#include "camera_under_test.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

_Static_assert(WIDTH == 1920u && HEIGHT == 1441u && STRIDE == 3840u,
               "Unexpected camera layout");
_Static_assert(FULL_SIZE == 5533440u && GRAY_SIZE == 2764800u,
               "Unexpected payload sizes");

/* Original implementation, retained as an independently scalar oracle. */
__attribute__((noinline, optimize("no-tree-vectorize")))
static int scalar_original(uint8_t *gray, const uint8_t *src, size_t used) {
    if (used != FULL_SIZE) return -1;
    for (size_t i = 0; i < GRAY_SIZE; ++i)
        gray[i] = src[STRIDE + 1u + 2u * i];
    return 0;
}

static void require(int ok, const char *message) {
    if (!ok) { fprintf(stderr, "GRAY_TEST_FAIL: %s\n", message); exit(1); }
}

struct guarded {
    uint8_t *mapping, *accessible, *pixels;
    size_t mapping_size, accessible_size, payload_size, tail_gap;
};

static struct guarded guarded_new(size_t size, size_t tail_gap, uint8_t canary) {
    long page_value = sysconf(_SC_PAGESIZE);
    require(page_value > 0, "Page size unavailable");
    size_t page = (size_t)page_value;
    size_t accessible_size = ((size + tail_gap + 64u + page - 1u) / page) * page;
    size_t mapping_size = accessible_size + 2u * page;
    uint8_t *mapping = mmap(NULL, mapping_size, PROT_NONE,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    require(mapping != MAP_FAILED, "Guarded mmap failed");
    uint8_t *accessible = mapping + page;
    require(mprotect(accessible, accessible_size, PROT_READ | PROT_WRITE) == 0,
            "mprotect failed");
    memset(accessible, canary, accessible_size);
    return (struct guarded){mapping, accessible,
        accessible + accessible_size - size - tail_gap,
        mapping_size, accessible_size, size, tail_gap};
}

static void guarded_check_canary(const struct guarded *buffer, uint8_t canary) {
    for (const uint8_t *p = buffer->accessible; p < buffer->pixels; ++p)
        require(*p == canary, "Output write before first pixel");
    for (size_t i = 0; i < buffer->tail_gap; ++i)
        require(buffer->pixels[buffer->payload_size + i] == canary,
                "Output write after last pixel");
}

static void guarded_free(struct guarded *buffer) {
    require(munmap(buffer->mapping, buffer->mapping_size) == 0, "munmap failed");
}

static uint32_t random_next(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *state = x;
}

static void fill_pattern(uint8_t *src, unsigned pattern) {
    uint32_t state = UINT32_C(0xa5ef7301) ^ (pattern * UINT32_C(0x9e3779b9));
    for (size_t i = 0; i < STRIDE; ++i) src[i] = (uint8_t)random_next(&state);
    for (size_t i = 0; i < GRAY_SIZE; ++i) {
        uint8_t y;
        switch (pattern) {
        case 0: y = 0; break;
        case 1: y = 255; break;
        case 2: y = (uint8_t)i; break;
        case 3: y = (uint8_t)((i / WIDTH) * 3u + i % WIDTH); break;
        case 4: y = (i & 1u) ? 0x55u : 0xaau; break;
        case 5: y = (uint8_t)(1u << (i & 7u)); break;
        default: y = (uint8_t)random_next(&state); break;
        }
        /* Alternating U/V always differs from its adjacent Y byte. */
        src[STRIDE + 2u * i] = y ^ ((i & 1u) ? 0xaau : 0x55u);
        src[STRIDE + 2u * i + 1u] = y;
    }
}

static void compare_pixels(const uint8_t *actual, const uint8_t *expected,
                           unsigned pattern, size_t source_gap, size_t dest_gap) {
    if (memcmp(actual, expected, GRAY_SIZE) == 0) return;
    size_t i = 0;
    while (i < GRAY_SIZE && actual[i] == expected[i]) ++i;
    fprintf(stderr, "GRAY_TEST_FAIL: pixel=%zu pattern=%u gaps=%zu/%zu "
            "actual=%u expected=%u\n", i, pattern, source_gap, dest_gap,
            (unsigned)actual[i], (unsigned)expected[i]);
    exit(1);
}

static void check_frame(unsigned pattern, size_t source_gap, size_t dest_gap,
                        uint8_t *expected) {
    struct guarded src = guarded_new(FULL_SIZE, source_gap, 0x39u);
    struct guarded dst = guarded_new(GRAY_SIZE, dest_gap, 0xc7u);
    fill_pattern(src.pixels, pattern);
    require(scalar_original(expected, src.pixels, FULL_SIZE) == 0,
            "Scalar oracle rejected full image");
    require(mprotect(src.accessible, src.accessible_size, PROT_READ) == 0,
            "Could not make input read-only");
    require(extract_gray(dst.pixels, src.pixels, FULL_SIZE) == 0,
            "Candidate rejected full image");
    compare_pixels(dst.pixels, expected, pattern, source_gap, dest_gap);
    guarded_check_canary(&dst, 0xc7u);

    /* Change every metadata byte and verify no output byte depends on row 0. */
    require(mprotect(src.accessible, src.accessible_size, PROT_READ | PROT_WRITE) == 0,
            "Could not update metadata row");
    for (size_t i = 0; i < STRIDE; ++i) src.pixels[i] ^= 0xffu;
    require(mprotect(src.accessible, src.accessible_size, PROT_READ) == 0,
            "Could not restore read-only input");
    memset(dst.pixels, 0x83u, GRAY_SIZE);
    require(extract_gray(dst.pixels, src.pixels, FULL_SIZE) == 0,
            "Candidate rejected changed metadata");
    compare_pixels(dst.pixels, expected, pattern, source_gap, dest_gap);
    guarded_check_canary(&dst, 0xc7u);
    guarded_free(&src);
    guarded_free(&dst);
}

static size_t check_invalid_lengths(void) {
    const size_t lengths[] = {0u, 1u, STRIDE, GRAY_SIZE, FULL_SIZE - STRIDE,
                             FULL_SIZE - 1u, FULL_SIZE + 1u, SIZE_MAX};
    long page_value = sysconf(_SC_PAGESIZE);
    require(page_value > 0, "Page size unavailable");
    size_t page = (size_t)page_value;
    uint8_t *blocked = mmap(NULL, 2u * page, PROT_NONE,
                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    require(blocked != MAP_FAILED, "Invalid-length guard mmap failed");
    for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
        /* Even reading either pointer before rejecting a length will fault. */
        require(extract_gray(blocked + page, blocked, lengths[i]) == -1,
                "Candidate accepted invalid byte count");
        require(scalar_original(blocked + page, blocked, lengths[i]) == -1,
                "Invalid length differs from original scalar behavior");
    }
    require(munmap(blocked, 2u * page) == 0, "Invalid-length munmap failed");
    return sizeof(lengths) / sizeof(lengths[0]);
}

int main(void) {
    (void)select_row;
    uint8_t *expected = malloc(GRAY_SIZE);
    require(expected != NULL, "Oracle allocation failed");
    size_t frames = 0;
    /* Zero gaps put both final legal bytes directly against PROT_NONE pages.
     * Other gaps give input/output different non-vector-aligned addresses. */
    const size_t gaps[][2] = {{0u, 0u}, {1u, 3u}, {15u, 7u}, {31u, 1u}};
    for (unsigned pattern = 0; pattern < 8u; ++pattern)
        for (size_t j = 0; j < sizeof(gaps) / sizeof(gaps[0]); ++j) {
            check_frame(pattern, gaps[j][0], gaps[j][1], expected);
            frames += 2u;
        }
    size_t invalid = check_invalid_lengths();
    free(expected);
    printf("NATIVE_ARM64_GRAY_TEST_PASS frames=%zu invalid_lengths=%zu "
           "input_bytes=%u output_bytes=%u byte_exact=1 metadata_excluded=1 "
           "unaligned=1 readonly_source=1 end_guard_pages=1 canaries=1\n",
           frames, invalid, FULL_SIZE, GRAY_SIZE);
    return 0;
}
'''


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=Path("mono_imu_vio.tar.gz"))
    parser.add_argument("--cc", default="gcc-11")
    args = parser.parse_args()
    require(platform.system() == "Linux" and platform.machine() in ("aarch64", "arm64"),
            "Run this test inside the Linux ARM64 build container")
    require(args.archive.is_file(), f"Package not found: {args.archive}")
    compiler_version = subprocess.check_output([args.cc, "-dumpfullversion"], text=True).strip()
    require(compiler_version.split(".")[0] == "11", f"Expected GCC 11, got {compiler_version}")
    with tempfile.TemporaryDirectory(prefix="name-test-native-gray-") as temp:
        work = Path(temp)
        with tarfile.open(args.archive, "r:gz") as archive:
            member = archive.getmember("mono_imu_vio/source/adapter/cam_imu_motion/camera.c")
            require(member.isfile(), "Packaged camera.c must be a regular source file")
            source = archive.extractfile(member).read()
            require(b"vld2q_u8" in source and b"vst1q_u8" in source,
                    "Packaged camera.c is missing expected explicit NEON intrinsics")
            (work / "camera_under_test.c").write_bytes(source)
        harness = work / "native_gray_test.c"
        harness.write_text(HARNESS_SOURCE)
        executable = work / "native_gray_test"
        subprocess.run([args.cc, "-O3", "-march=armv8-a", "-std=c11", "-Wall", "-Wextra",
                        "-Werror", str(harness), "-o", str(executable)], check=True, cwd=work)
        subprocess.run([str(executable)], check=True, cwd=work, timeout=60)
    print(f"PACKAGED_GRAY_SOURCE_PASS archive={args.archive} compiler={args.cc}-{compiler_version}")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"NATIVE_ARM64_GRAY_TEST_FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
