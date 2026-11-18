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
    # Match start.sh: private package libraries precede host system libraries.
    env["LD_LIBRARY_PATH"] = str(root / "lib")
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
        for soname, filename in found.items():
            if (soname.startswith("libopencv_") or soname in ("libboost_serialization.so.1.74.0", "libGLU.so.1")) and Path(filename).resolve() != (root / "lib" / soname).resolve():
                raise ValueError(f"{name}: bundled {soname} resolved outside the package to {filename}")
        if any(n.startswith("libopencv_") and not n.endswith(".so.4.5d") for n in found):
            raise ValueError(f"{name}: mixed OpenCV ABI")
        if name == "bin/camera_preview":
            required = {"libopencv_core.so.4.5d", "libopencv_highgui.so.4.5d"}
            if set(LIBS) & found.keys():
                raise ValueError(f"{name}: camera preview depends on the SLAM runtime")
        elif name.startswith("bin/") and name.rsplit("/", 1)[-1] not in ("camera_live", "camera_umi"):
            required = {"libopencv_core.so.4.5d", *LIBS}
        else:
            continue
        if not required.issubset(found):
            raise ValueError(f"{name}: missing runtime links: {sorted(required - found.keys())}")



def bundle_libraries(root):
    """Bundle only the application's OpenCV/Boost/GLU libraries, never GPU or libc."""
    import shutil
    env = {k: v for k, v in os.environ.items() if k not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "LD_AUDIT")}
    env["LC_ALL"] = "C"
    selected = {}
    for path in sorted((root / "bin").iterdir()):
        if path.is_symlink():
            continue
        report = subprocess.check_output(["ldd", str(path)], env=env, text=True)
        if "not found" in report:
            raise RuntimeError("Unresolved build-time dependency: " + report)
        for soname, filename in re.findall(r"^\s*(\S+) => (/[^\n]+?) \(0x", report, re.M):
            if soname.startswith("libopencv_") or soname in ("libboost_serialization.so.1.74.0", "libGLU.so.1"):
                selected[soname] = Path(filename).resolve()
    licenses = root / "licenses" / "runtime"
    licenses.mkdir(parents=True)
    lines = []
    for soname, original in sorted(selected.items()):
        shutil.copy2(original, root / "lib" / soname)
        owner = subprocess.check_output(["dpkg-query", "-S", str(original)], text=True).splitlines()[0].split(": ",1)[0]
        package = owner.split(":", 1)[0]
        copyright = Path("/usr/share/doc") / package / "copyright"
        if not copyright.is_file():
            raise RuntimeError("Missing bundled library copyright: " + str(copyright))
        shutil.copy2(copyright, licenses / (package + ".copyright"))
        version = subprocess.check_output(["dpkg-query", "-W", "-f=${Version}", owner], text=True)
        lines.append(f"{soname}\t{owner}\t{version}")
    (root / "RUNTIME_LIBRARIES.txt").write_text("\n".join(lines) + "\n")
