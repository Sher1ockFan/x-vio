#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
fail() { printf '%s\n' "$*" >&2; exit 1; }
[[ $# -le 1 && ( -z ${1:-} || $1 == --devices ) ]] || fail 'Usage: bash check.sh [--devices]'
[[ $(uname -sm) == 'Linux aarch64' ]] || fail 'Expected Jetson Linux ARM64.'
grep -Eq '^# R36\b.*REVISION:[[:space:]]*5\.0\b' /etc/nv_tegra_release ||
    fail 'This release requires Jetson Linux R36.5.0.'
sha256sum --quiet -c SHA256SUMS
[[ -L bin/orb_live_viewer && $(readlink bin/orb_live_viewer) == stock_viewer ]] ||
    fail 'Invalid native viewer entry point.'
unset LD_LIBRARY_PATH LD_PRELOAD LD_AUDIT
export LC_ALL=C
for file in bin/stock_viewer bin/mono_inertial_euroc bin/camera_live bin/camera_umi lib/*.so; do
    if ! output=$(ldd -r "$file" 2>&1) || grep -Eq 'not found|undefined symbol' <<< "$output"; then
        printf 'Runtime dependency check failed for %s\n%s\n' "$file" "$output" >&2
        exit 1
    fi
done
python3 -c 'import numpy' || fail 'Install the Jetson python3-numpy package.'

if [[ ${1:-} == --devices ]]; then
    camera=/dev/gmsl/B
    serial=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    [[ -c $camera && -r $camera && -w $camera ]] ||
        fail "Camera unavailable to $(id -un): $camera. Check the GMSL driver and video group."
    [[ -c $serial && -r $serial ]] ||
        fail "IMU unavailable to $(id -un): $serial. Check the CP2102 connection and dialout group."
    echo 'Camera and IMU device paths are accessible.'
fi
echo 'ORB_HUB_CHECK_OK'
