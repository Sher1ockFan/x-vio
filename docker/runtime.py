"""Build, inspect and install the relocatable ORB CPU runtime, without camera access."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import re
import struct
import subprocess
import sys
import tarfile
import tempfile

BINS = "camera_live camera_umi mono_inertial_euroc mono_inertial_viewer orb_live_runner orb_live_viewer".split()
LIBS = "libDBoW2.so libg2o.so libORB_SLAM3.so libpangolin.so".split()
ELFS = ["bin/" + n for n in BINS] + ["lib/" + n for n in LIBS]
INPUTS = ["cam_imu_motion/camera.c", "learn/Jetson构建差异.patch", "orb_replay/camera_live.c",
          "orb_replay/orb_live_runner.cc", "orb_replay/replay_viewer.cc", "orb_replay/pose_trace.h",
          "docker/compile.sh", "docker/sources.py", "docker/targets.cmake", "docker/opencv.py", "docker/runtime.py"]
PROFILE = "orin-r36.5-jammy-opencv408-v1"
MAX_ARCHIVE = 64 * 1024 * 1024


def sha(data):
    return hashlib.sha256(data).hexdigest()


def input_hashes(kit):
    return {n: sha((kit/n).read_bytes()) for n in INPUTS}


def check_links(root, cv=None):
    env = {k: v for k, v in os.environ.items() if k not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "LD_AUDIT")}
    env["LC_ALL"] = "C"
    if cv:
        env["LD_LIBRARY_PATH"] = str(cv)
    for name in ELFS:
        path = root / name
        info = subprocess.check_output(["readelf", "-dW", path], env=env, text=True)
        for tag, entries in re.findall(r"\((RPATH|RUNPATH|NEEDED)\)[^\n]*?\[([^\]]+)\]", info):
            if tag == "NEEDED" and "/" in entries or tag != "NEEDED" and any(
                    part not in ("$ORIGIN", "$ORIGIN/../lib") for part in entries.split(":")):
                raise ValueError(f"{name}: non-relocatable {tag} {entries}")
        result = subprocess.run(["ldd", "-r", str(path)], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, env=env, text=True)
        if result.returncode or "not found" in result.stdout or "undefined symbol" in result.stdout:
            raise RuntimeError(f"{name}: dynamic dependencies or symbols failed:\n{result.stdout}")
        found = dict(re.findall(r"^\s*(\S+) => (/[^\n]+?) \(0x", result.stdout, re.M))
        for soname in LIBS:
            if soname in found and Path(found[soname]).resolve() != (root/"lib"/soname).resolve():
                raise ValueError(f"{name}: {soname} resolved outside the release to {found[soname]}")
        if any(n.startswith("libopencv_") and not n.endswith(".so.408") for n in found):
            raise ValueError(f"{name}: mixed OpenCV ABI")
        if name.startswith("bin/") and name.rsplit("/",1)[-1] not in ("camera_live", "camera_umi"):
            required = {"libopencv_core.so.408", *LIBS}
            if not required.issubset(found):
                raise ValueError(f"{name}: missing runtime links: {sorted(required - found.keys())}")


def unpack_release(data):
    if len(data) > MAX_ARCHIVE:
        raise ValueError("Runtime archive too large.")
    contents = {}
    total = 0
    with tarfile.open(fileobj=io.BytesIO(data), mode="r|gz") as tar:
        for m in tar:
            total += m.size
            if (m.name not in ELFS + ["build.json"] or not m.isfile() or m.name in contents or
                    m.size <= 0 or total > MAX_ARCHIVE):
                raise ValueError(f"Invalid runtime member {m.name}")
            contents[m.name] = tar.extractfile(m).read()
    if set(contents) != set(ELFS + ["build.json"]):
        raise ValueError("Incomplete runtime archive.")
    meta = json.loads(contents["build.json"])
    if meta.get("profile") != PROFILE or meta.get("files") != {n: sha(contents[n]) for n in ELFS}:
        raise ValueError("Runtime profile or binary checksums disagree.")
    for n in ELFS:
        b = contents[n]
        if len(b) < 64 or b[:6] != b"\x7fELF\x02\x01" or struct.unpack_from("<H", b, 18)[0] != 183:
            raise ValueError(f"{n}: expected a little-endian AArch64 ELF file")
    return contents, meta


def pack(root, kit, source, output):
    cv = Path("/build/orb-kit/opencv-4.8.0/usr/lib")
    check_links(root, cv)
    meta = {"profile": PROFILE, "inputs": input_hashes(kit), "source": json.loads(source.read_text()),
            "compiler": subprocess.check_output(["g++-11", "-dumpfullversion"], text=True).strip(),
            "cpu_target": "armv8-a", "opencv": "4.8.0-1-g6371ee1",
            "files": {n: sha((root/n).read_bytes()) for n in ELFS}}
    (root / "build.json").write_text(json.dumps(meta, sort_keys=True, indent=2) + "\n")
    # Stable gzip/tar headers make identical incrementals reuse the same release ID.
    import gzip
    with tempfile.NamedTemporaryFile(dir=output.parent, delete=False) as tmp:
        path = Path(tmp.name)
        try:
            with gzip.GzipFile(filename="", mode="wb", fileobj=tmp, mtime=0) as gz, tarfile.open(fileobj=gz, mode="w|") as tar:
                for n in sorted(ELFS + ["build.json"]):
                    data = (root/n).read_bytes()
                    member = tarfile.TarInfo(n)
                    member.mode = 0o755 if n in ELFS else 0o644
                    member.size = len(data)
                    tar.addfile(member, io.BytesIO(data))
            tmp.flush()
            unpack_release(path.read_bytes())
            os.replace(path, output)
        finally:
            path.unlink(missing_ok=True)


def install_release(data, release_id, home):
    if sha(data) != release_id:
        raise ValueError("Runtime archive hash mismatch.")
    contents, _ = unpack_release(data)
    if platform.system() != "Linux" or platform.machine() != "aarch64":
        raise ValueError("This runtime requires the ARM64 Jetson.")
    rel = Path("/etc/nv_tegra_release").read_text()
    if not re.search(r"^# R36\b[^\n]*REVISION:\s*5\.0\b", rel, re.M):
        raise ValueError("Expected Jetson Linux R36.5.0. Active kit preserved.")
    base = home / "orb_trial"
    releases = base / "runtime"
    if base.is_symlink() or releases.is_symlink():
        raise ValueError(f"Unexpected symbolic link in {releases}")
    releases.mkdir(parents=True, exist_ok=True)
    dest = releases / release_id
    if dest.is_symlink():
        raise ValueError("Runtime release path is a symlink.")
    if dest.exists():
        for n, b in contents.items():
            p = dest/n
            if (p.is_symlink() or p.parent.is_symlink() or not p.is_file() or sha(p.read_bytes()) != sha(b)
                    or n in ELFS and not os.access(p, os.X_OK)):
                raise ValueError(f"Installed immutable runtime differs: {p}")
        check_links(dest)
    else:
        with tempfile.TemporaryDirectory(prefix=".stage-", dir=releases) as tmp:
            stage = Path(tmp) / "release"
            for n, b in contents.items():
                p = stage / n
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b)
                p.chmod(0o755 if n in ELFS else 0o644)
            check_links(stage)
            stage.rename(dest)
    print("JETSON_RUNTIME_CHECK_OK", dest, flush=True)


def main():
    cmd = argparse.ArgumentParser()
    sub = cmd.add_subparsers(dest="action", required=True)
    p = sub.add_parser("pack")
    for n in ("root", "kit", "source", "output"):
        p.add_argument(n, type=Path)
    p = sub.add_parser("verify")
    p.add_argument("archive", type=Path)
    p.add_argument("kit", type=Path)
    a = cmd.parse_args()
    if a.action == "pack":
        pack(a.root, a.kit, a.source, a.output)
    else:
        _, meta = unpack_release(a.archive.read_bytes())
        if meta["inputs"] != input_hashes(a.kit):
            raise ValueError("The runtime and project sources differ. Re-run docker/build.sh.")
        print(f"Verified {len(BINS)} AArch64 programs and {len(LIBS)} libraries.")


if __name__ == "__main__":
    main()
