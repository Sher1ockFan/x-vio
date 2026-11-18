#!/usr/bin/env bash
set -euo pipefail
pkg=$1
src=$2
input=$3

install -m 755 "$input/trajectory.sh" "$pkg/"
install -m 644 "$input/"{export_euroc.py,check_return.py,trajectory_view.html} "$pkg/"

# Ship the modified core code used for this runtime without example data or built objects.
mkdir -p "$pkg/source/ORB_SLAM3" "$pkg/source/adapter/"{orb_replay,cam_imu_motion}
awk '{
    file = substr($0, 67)
    if (file ~ /^\.\/(src|include|Thirdparty)\// ||
        file ~ /^\.\/(CMakeLists\.txt|LICENSE|README\.md)$/ ||
        file == "./Examples/Monocular-Inertial/mono_inertial_euroc.cc")
        print file
}' "$pkg/source.sha256" |
    tar -C "$src" -cf - -T - |
    tar -C "$pkg/source/ORB_SLAM3" -xf -
mv "$pkg/viewer.cc" "$pkg/source/adapter/"
install -m 644 "$input/camera_preview.cc" "$pkg/source/adapter/"
install -m 644 "$input/orb_replay/camera_live.c" "$input/orb_replay/gray.h" "$pkg/source/adapter/orb_replay/"

{
    echo 'Calibration: fit_03 Omni-radtan, full 1920x1536, residual camera shift +14502238 ns. Independent accuracy validation skipped.'
    echo 'Camera: jl_gmsl video0 with 60 fps Tegra frame-rate control and CNTVCT timestamp mapping from run_02.'
    echo 'Recording: camera and IMU at the selected capture format; record-view adds an independent latest-image window.'
    echo 'Modified source directory in the package is a selected subset for reading; see provenance.'
} >> "$pkg/BUILD_INFO.txt"

mkdir -p "$pkg/provenance"
cp "$input/provenance/"* "$pkg/provenance/"
