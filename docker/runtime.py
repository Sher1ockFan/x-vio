"""Verify ARM64 package runtime links inside the prepared Docker builder."""
import os
from pathlib import Path
import re
import subprocess

LIBS = "libDBoW2.so libg2o.so libORB_SLAM3.so libpangolin.so".split()
ELFS = []  # Set by package.sh from the produced package.

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

