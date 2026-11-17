#!/usr/bin/env python3
"""Changed source bytes must rebuild even when an earlier object's mtime is newer.

Runs the actual source-sync Python block embedded in build.sh against a tiny
make graph. No Docker engine, ORB compilation, or production cache is touched.
"""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

PROJECT = Path(__file__).resolve().parents[1]
build = (PROJECT / "build.sh").read_text()
start = build.index("\nfrom pathlib import Path\n") + 1
end = build.index("\nPY\n", start)
sync_code = build[start:end]
compile(sync_code, "build.sh:source-sync", "exec")
make = shutil.which("make")
if not make:
    raise SystemExit("BUILD_SYNC_TEST: make is required")


def manifest(incoming):
    data = (incoming / "source/unit.txt").read_bytes()
    (incoming / "source.sha256").write_text(hashlib.sha256(data).hexdigest() + "  ./unit.txt\n")


def sync(incoming, cache):
    subprocess.run([sys.executable, "-c", sync_code, str(incoming), str(cache)], check=True)
    # The surrounding build.sh copies the new manifest after its hash check.
    shutil.copyfile(incoming / "source.sha256", cache / "source.sha256")


def run_make(cache):
    subprocess.run([make, "-s", "-f", "Makefile", "built.txt"], cwd=cache,
                   check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def build_count(cache):
    return len((cache / "rebuilds.log").read_text().splitlines())


with tempfile.TemporaryDirectory(prefix="mono-build-sync-") as temporary:
    root = Path(temporary)
    incoming, cache = root / "incoming", root / "cache"
    (incoming / "source").mkdir(parents=True)
    cache.mkdir()
    source = incoming / "source/unit.txt"
    source.write_text("version 1\n")
    source.chmod(0o640)
    manifest(incoming)
    (cache / "Makefile").write_text(
        "built.txt: source/unit.txt\n"
        "\t@cp source/unit.txt built.txt\n"
        "\t@printf 'build\\n' >> rebuilds.log\n"
    )
    sync(incoming, cache)
    run_make(cache)
    assert (cache / "built.txt").read_text() == "version 1\n"
    assert build_count(cache) == 1

    # Reproduce the race: the edit existed before a previous build finished.
    # Separate these timestamps by seconds so both nanosecond and older make
    # implementations exercise the same dependency-ordering failure.
    now = time.time_ns()
    os.utime(cache / "source/unit.txt", ns=(now - 20_000_000_000,) * 2)
    os.utime(cache / "built.txt", ns=(now - 10_000_000_000,) * 2)
    source.write_text("version 2, edited before the earlier build completed\n")
    os.utime(source, ns=(now - 30_000_000_000,) * 2)
    manifest(incoming)
    assert source.stat().st_mtime_ns < (cache / "built.txt").stat().st_mtime_ns

    # Negative control: the old copy2 behavior changes bytes but incorrectly
    # leaves the build result cached. This demonstrates the actual regression.
    control = root / "copy2_control"
    shutil.copytree(cache, control)
    shutil.copy2(source, control / "source/unit.txt")
    run_make(control)
    assert (control / "built.txt").read_text() == "version 1\n"
    assert build_count(control) == 1

    sync(incoming, cache)
    assert (cache / "source/unit.txt").stat().st_mtime_ns > (cache / "built.txt").stat().st_mtime_ns
    assert (cache / "source/unit.txt").stat().st_mode & 0o777 == 0o640
    run_make(cache)
    assert (cache / "built.txt").read_bytes() == source.read_bytes()
    assert build_count(cache) == 2

    # A no-change sync must retain the working cache, not touch every source.
    previous_source_time = (cache / "source/unit.txt").stat().st_mtime_ns
    previous_result_time = (cache / "built.txt").stat().st_mtime_ns
    sync(incoming, cache)
    run_make(cache)
    assert (cache / "source/unit.txt").stat().st_mtime_ns == previous_source_time
    assert (cache / "built.txt").stat().st_mtime_ns == previous_result_time
    assert build_count(cache) == 2

print("BUILD_SYNC_TEST_PASS old_copy2_misses_change=true changed_bytes_rebuilt=true unchanged_cache_preserved=true")
