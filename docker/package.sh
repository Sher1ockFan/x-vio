#!/usr/bin/env bash
# Create the relocatable Jetson package from verified upstream build outputs.
set -euo pipefail
root=${ORB_STOCK_BUILD_ROOT:-/build/orb-stock}
name=${ORB_STOCK_PACKAGE_NAME:-orb_stock}
archive=${ORB_STOCK_ARCHIVE_NAME:-orb-stock-orin-nx.tar.gz}
[[ $name =~ ^[A-Za-z0-9_-]+$ && $archive =~ ^[A-Za-z0-9_-]+\.tar\.gz$ ]]
src=$root/upstream
input=${ORB_STOCK_INPUT_ROOT:-$root/input}
cv=/build/orb-kit/opencv-4.8.0/usr/lib
pango=/build/orb-kit/prefix
cd "$src"
sha256sum --quiet -c "$root/upstream.sha256"
temp=$(mktemp -d "$root/package.XXXXXX")
trap 'rm -rf "$temp"' EXIT
pkg=$temp/$name
mkdir -p "$pkg"/{bin,lib,licenses,Vocabulary,params,tools/orb_replay,tools/cam_imu_motion}

install -m 755 "$src/Examples/Monocular-Inertial/mono_inertial_euroc" "$pkg/bin/"
install -m 755 "$src/lib/libORB_SLAM3.so" "$src/Thirdparty/DBoW2/lib/libDBoW2.so" \
    "$src/Thirdparty/g2o/lib/libg2o.so" "$pango/lib/libpangolin.so" "$pkg/lib/"

CCACHE_DIR=/ccache ccache g++-11 -std=c++17 -O2 -march=armv8-a -include unistd.h -include boost/serialization/library_version_type.hpp \
    -I"$src" -I"$src/include" -I"$src/include/CameraModels" -I"$src/Thirdparty/Sophus" \
    -isystem /usr/include/eigen3 -isystem "$cv/../include/opencv4" -isystem "$pango/include" \
    "$input/viewer.cc" -o "$pkg/bin/stock_viewer" -L"$pkg/lib" -L"$cv" \
    '-Wl,-rpath,$ORIGIN/../lib' "-Wl,-rpath-link,$cv" "-Wl,-rpath-link,$pkg/lib" \
    -lORB_SLAM3 -lpangolin -lopencv_core -lOpenGL -lGLX -lGLU -lGLEW -lEGL -lX11 -pthread
ln -s stock_viewer "$pkg/bin/orb_live_viewer"
camera_flags=()
if [[ ${ORB_STOCK_HUB_CAPTURE:-0} == 1 ]]; then
    camera_flags+=(-DORB_HUB_CAPTURE)
fi
gcc-11 -O2 -march=armv8-a "${camera_flags[@]}" "$input/orb_replay/camera_live.c" -o "$pkg/bin/camera_live"
gcc-11 -O2 -march=armv8-a "${camera_flags[@]}" -DORB_CAMERA_SEQUENCE "$input/orb_replay/camera_live.c" -o "$pkg/bin/camera_umi"

install -m 755 "$input/start.sh" "$input/check.sh" "$pkg/"
install -m 644 "$input/playback.py" "$input/README.md" "$input/viewer.cc" "$pkg/"
install -m 644 "$input/orb_replay/"{live.py,umi.py,umi_quality.py} "$pkg/tools/orb_replay/"
install -m 644 "$input/cam_imu_motion/"{imu.py,timing_core.py} "$pkg/tools/cam_imu_motion/"
install -m 644 "$input/params/"{orb_trial.yaml,timing.json} "$pkg/params/"
install -m 644 "$src/LICENSE" "$pkg/licenses/ORB_SLAM3.txt"
install -m 644 "$src/Thirdparty/DBoW2/README.txt" "$pkg/licenses/DBoW2.txt"
install -m 644 "$src/Thirdparty/g2o/license-bsd.txt" "$pkg/licenses/g2o.txt"
install -m 644 /build/orb-kit/src/pangolin/LICENCE "$pkg/licenses/Pangolin.txt"
tar -xzf "$src/Vocabulary/ORBvoc.txt.tar.gz" -C "$pkg/Vocabulary"
test -s "$pkg/Vocabulary/ORBvoc.txt"
{
    echo 'ORB-SLAM3 upstream commit: 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4'
    echo 'Official source archive SHA256: b69fe1809588a8610580db1d5e6b77c63b5e4b5b47abc16ed839e93fbc7e7ddf'
    echo "Original upstream files verified after compile: $(wc -l < "$root/upstream.sha256")"
    echo 'Target: Ubuntu 22.04 ARM64, Jetson Linux R36.5.0, ARMv8-A'
    echo 'NVIDIA OpenCV: 4.8.0-1-g6371ee1 (.so.408)'
    echo "Compiler: GCC $(g++-11 -dumpfullversion)"
} > "$pkg/BUILD_INFO.txt"
cp "$root/upstream.sha256" "$pkg/upstream.sha256"
if [[ -f "$input/extra.sh" ]]; then
    bash "$input/extra.sh" "$pkg" "$src" "$input"
fi
cd "$pkg"
find . -type f ! -name SHA256SUMS -print0 | LC_ALL=C sort -z | xargs -0 sha256sum > "$temp/SHA256SUMS"
mv "$temp/SHA256SUMS" "$pkg/SHA256SUMS"
PYTHONDONTWRITEBYTECODE=1 python3 - "$input/runtime.py" "$pkg" "$cv" <<'PY'
import importlib.util
from pathlib import Path
import sys
spec = importlib.util.spec_from_file_location("checks", sys.argv[1])
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)
pkg = Path(sys.argv[2])
checks.ELFS = [str(p.relative_to(pkg)) for p in (pkg / "bin").iterdir() if not p.is_symlink()]
checks.ELFS += [str(p.relative_to(pkg)) for p in (pkg / "lib").iterdir()]
checks.check_links(pkg, sys.argv[3])
print("RELOCATABLE_ARM64_LINKS_OK")
PY
tar -C "$temp" -cf - "$name" | gzip -1 > "$temp/$archive"
mv "$temp/$archive" "$root/$archive"
