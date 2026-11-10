#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash start.sh live [SECONDS]       B port, 1920x1440 capture and native viewer
                                    Omit SECONDS or use 0 to run until Ctrl+C
  bash start.sh record NEW_TAKE      Record calibrated camera + IMU packets
  bash start.sh record-view NEW_TAKE Record with independent native preview
  bash start.sh replay TAKE          Replay a complete take in the native viewer
  bash start.sh recover TAKE         Verify a previously interrupted take
  bash start.sh mark TAKE            Mark A/B/A holds while recording
  bash start.sh trajectory TAKE      Export native trajectory and check marked holds
EOF
}
fail() { printf '%s\n' "$*" >&2; exit 1; }
mode=${1:-}
case "$mode" in
    -h|--help|'') usage; exit 0 ;;
    live) [[ $# -le 2 && ${2:-0} =~ ^[0-9]+$ ]] || { usage >&2; exit 2; } ;;
    record|record-view|replay|recover|mark|trajectory)
        [[ $# -eq 2 && -n $2 ]] || { usage >&2; exit 2; }
        ;;
    *) usage >&2; exit 2 ;;
esac

[[ $(uname -sm) == 'Linux aarch64' ]] || fail 'Run this command on the Jetson Linux ARM64 host.'
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
tools="$root/tools/orb_replay"
bin="$root/bin"
cfg="$root/params/orb_trial.yaml"
timing="$root/params/timing.json"
export PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH="$root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

case "$mode" in
    live|replay|record-view)
        [[ -n ${DISPLAY:-} ]] || fail 'Run from the Jetson graphical desktop with DISPLAY set.'
        if command -v xdpyinfo >/dev/null; then
            timeout 8s xdpyinfo >/dev/null 2>&1 || fail "Cannot access X11 display $DISPLAY."
        fi
        [[ -x "$bin/stock_viewer" ]] || fail "Missing executable: $bin/stock_viewer"
        vocab=${ORB_VOCABULARY:-"$root/Vocabulary/ORBvoc.txt"}
        [[ -r $vocab && -s $vocab ]] || fail "Missing ORB vocabulary: $vocab. Set ORB_VOCABULARY to ORBvoc.txt."
        ;;
esac

case "$mode" in
    live)
        [[ -x "$bin/camera_live" && -x "$bin/orb_live_viewer" ]] || fail 'Missing live executables.'
        camera=/dev/gmsl/B
        [[ -c $camera && -r $camera && -w $camera ]] ||
            fail "Camera unavailable to $(id -un): $camera. Check the GMSL driver and video group."
        printf '%s\n' "$camera: 1920x1441 UYVY; crop top row to 1920x1440; ORB input resize to 960x720." \
            'Trial calibration: fit_05 and camera–IMU time shift inherited from channel C.'
        runs=${ORB_HUB_RUNS:-"$root/runs"}
        mkdir -p -- "$runs"
        run=$(mktemp -d -- "$runs/live_XXXXXX")
        cp -- "$cfg" "$run/settings.yaml"
        cp -- "$timing" "$run/timing.json"
        exec python3 -u "$tools/live.py" "$bin" "$vocab" "$run/settings.yaml" "$run/timing.json" \
            "$run/session" --viewer --seconds "${2:-0}"
        ;;
    record|record-view)
        [[ -x "$bin/camera_umi" ]] || fail "Missing executable: $bin/camera_umi"
        python3 -c 'import numpy' || fail 'The recorder requires python3-numpy on Jetson.'
        mkdir -p -- "$(dirname -- "$2")"
        mkdir -m 700 -- "$2" || fail 'Choose a new, unused take directory.'
        take=$(cd -- "$2" && pwd -P)
        cp -- "$cfg" "$take/settings.yaml"
        cp -- "$timing" "$take/timing.json"
        printf 'Recording to %s. Press Ctrl+C once to seal the recording. Progress: %s/record.log\n' "$take" "$take"
        if [[ $mode == record ]]; then
            exec python3 -u "$tools/umi.py" record "$take" "$bin/camera_umi" >"$take/record.log" 2>&1
        fi
        exec python3 -u "$root/playback.py" record-view "$take" "$bin/stock_viewer" "$vocab" "$bin/camera_umi"
        ;;
    replay)
        exec python3 -u "$root/playback.py" replay "$2" "$bin/stock_viewer" "$vocab"
        ;;
    recover|mark)
        exec python3 -u "$tools/umi.py" "$mode" "$2"
        ;;
    trajectory)
        exec bash "$root/trajectory.sh" "$2"
        ;;
esac
