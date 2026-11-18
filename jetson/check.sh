#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
fail() { printf '%s\n' "$*" >&2; exit 1; }
[[ $# -le 1 && ( -z ${1:-} || $1 == --devices ) ]] || fail 'Usage: bash check.sh [--devices]'
[[ $(uname -sm) == 'Linux aarch64' ]] || fail 'Expected Jetson Linux ARM64.'
grep -Eq '^# R36\b.*REVISION:[[:space:]]*5\.0\b' /etc/nv_tegra_release ||
    fail 'This release requires Jetson Linux R36.5.0.'
# Active calibration is user-supplied data; its consistency is checked at startup.
# Preserve checksum checks for binaries, scripts, templates and legacy references.
awk '$2 != "./params/camera_profile.json" && $2 != "./params/orb_trial.yaml" && $2 != "./params/timing.json" {print}' SHA256SUMS | sha256sum --quiet -c -
[[ -L bin/orb_live_viewer && $(readlink bin/orb_live_viewer) == stock_viewer ]] ||
    fail 'Invalid native viewer entry point.'
[[ -L bin/orb_live_runner && $(readlink bin/orb_live_runner) == stock_viewer ]] ||
    fail 'Invalid headless entry point.'
unset LD_LIBRARY_PATH LD_PRELOAD LD_AUDIT
export LC_ALL=C
# Match the launcher for transitive OpenCV dependencies as well.
export LD_LIBRARY_PATH="$PWD/lib"
for file in bin/stock_viewer bin/mono_inertial_euroc bin/camera_live bin/camera_umi bin/camera_preview lib/*.so; do
    if ! output=$(ldd -r "$file" 2>&1) || grep -Eq 'not found|undefined symbol' <<< "$output"; then
        printf 'Runtime dependency check failed for %s\n%s\n' "$file" "$output" >&2
        exit 1
    fi
done
python3 -c 'import numpy' || fail 'Install the Jetson python3-numpy package.'

if [[ ${1:-} == --devices ]]; then
    profile=${VIO_CAMERA_PROFILE:-"$PWD/params/camera_profile.json"}
    camera=$(python3 "$PWD/camera_config.py" device "$profile")
    serial=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    [[ -c $camera && -r $camera && -w $camera ]] ||
        fail "Camera unavailable to $(id -un): $camera. Check the jl_gmsl driver and video group."
    [[ -c $serial && -r $serial ]] ||
        fail "IMU unavailable to $(id -un): $serial. Check the CP2102 connection and dialout group."
    echo 'Camera and IMU device paths are accessible.'
fi
echo 'MONO_VIO_60_RUNTIME_CHECK_OK (software only; new-camera calibration and streaming not verified)'
