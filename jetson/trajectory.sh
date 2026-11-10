#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 1 ]] || { echo "Usage: bash trajectory.sh TAKE" >&2; exit 2; }
[[ $(uname -sm) == 'Linux aarch64' && -n ${DISPLAY:-} ]] || {
    echo 'Run from the Jetson graphical desktop.' >&2
    exit 1
}
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
take=$(cd -- "$1" && pwd -P)
job=$(mktemp -d -- "$take/stock-trajectory-XXXXXX")
echo "Trajectory job: $job"
export LD_LIBRARY_PATH="$root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 "$root/export_euroc.py" "$take" "$job/euroc"
cd -- "$job"
"$root/bin/mono_inertial_euroc" "$root/Vocabulary/ORBvoc.txt" \
    "$job/euroc/settings.yaml" "$job/euroc" "$job/euroc/times.txt" 2>&1 | tee "$job/orb.log"
[[ -s CameraTrajectory.txt && -s KeyFrameTrajectory.txt ]] || {
    echo 'ORB-SLAM3 did not export a usable trajectory. Inspect orb.log.' >&2
    exit 1
}
echo "Native trajectory: $job/CameraTrajectory.txt"
if [[ -f "$take/marks.json" ]]; then
    python3 "$root/check_return.py" "$job"
fi
