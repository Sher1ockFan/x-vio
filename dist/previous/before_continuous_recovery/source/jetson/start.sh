#!/usr/bin/env bash
set -euo pipefail

# The live supervisor owns terminal output and forwards stop signals to this worker.
if [[ ${1:-} == --live-worker ]]; then
    shift
elif [[ ${1:-} == live ]]; then
    root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" 2>/dev/null && pwd -P) || {
        printf '%s\n' 'VIO_E_PATH: Cannot locate the directory containing start.sh.' >&2
        exit 1
    }
    if ! command -v python3 >/dev/null 2>&1; then
        printf '%s\n' 'VIO_E_DEPENDENCY: python3 is not installed or is not on PATH.' >&2
        exit 1
    fi
    if [[ ! -r "$root/live_console.py" ]]; then
        printf 'VIO_E_PATH: Missing or unreadable startup helper: %s/live_console.py\n' "$root" >&2
        exit 1
    fi
    exec python3 -u "$root/live_console.py" "$@"
fi

usage() {
    cat <<'EOF'
Usage:
  bash start.sh live [SECONDS]       B port, 1920x1440 capture and native viewer
                                    Omit SECONDS or use 0 to run until Ctrl+C
  bash start.sh record NEW_TAKE      Record B-port camera + IMU packets
  bash start.sh record-view NEW_TAKE Record with independent native preview
  bash start.sh replay TAKE          Replay a complete take in the native viewer
  bash start.sh recover TAKE         Verify a previously interrupted take
  bash start.sh mark TAKE            Mark A/B/A holds while recording
  bash start.sh trajectory TAKE      Export native trajectory and check marked holds

Live maps: VIO_MAP_MODE=auto|new|off (default auto).
           VIO_MAP_FILE defaults to PACKAGE/maps/workspace_v2.osa.
           Existing reference maps are never overwritten automatically.
           Record/replay are fresh sessions and do not use the live reference.
EOF
}
fail() { printf '%s\n' "$*" >&2; exit 1; }
fail_code() { local code=$1; shift; fail "$code: $*"; }
mode=${1:-}
case "$mode" in
    -h|--help|'') usage; exit 0 ;;
    live) [[ $# -le 2 && ${2:-0} =~ ^[0-9]+$ ]] || {
        printf '%s\n' 'VIO_E_ARGUMENT: Usage: bash start.sh live [SECONDS]; use a nonnegative integer, omit it or use 0 to run until Ctrl+C.' >&2
        exit 2
    } ;;
    record|record-view|replay|recover|mark|trajectory)
        [[ $# -eq 2 && -n $2 ]] || { usage >&2; exit 2; }
        ;;
    *) usage >&2; exit 2 ;;
esac

[[ $(uname -sm) == 'Linux aarch64' ]] || fail_code VIO_E_PLATFORM 'Run this command on the Jetson Linux ARM64 host.'
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
tools="$root/tools/orb_replay"
bin="$root/bin"
cfg="$root/params/orb_trial.yaml"
timing="$root/params/timing.json"
export PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH="$root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

case "$mode" in
    live|replay|record-view)
        if [[ -z ${DISPLAY:-} ]]; then
            if [[ -n ${DSIPLAY:-} ]]; then
                fail_code VIO_E_DISPLAY 'DISPLAY is unset; DSIPLAY is misspelled. Use DISPLAY=:0 (or the display of your Jetson desktop).'
            fi
            fail_code VIO_E_DISPLAY 'DISPLAY is unset. Run from the Jetson graphical desktop with DISPLAY=:0 (or your desktop display).'
        fi
        if command -v xdpyinfo >/dev/null; then
            timeout 8s xdpyinfo >/dev/null 2>&1 || fail_code VIO_E_DISPLAY "Cannot access X11 display $DISPLAY; check the desktop login and X11 access permissions."
        fi
        [[ -x "$bin/stock_viewer" ]] || fail_code VIO_E_PATH "Missing or non-executable file: $bin/stock_viewer"
        vocab=${ORB_VOCABULARY:-"$root/Vocabulary/ORBvoc.txt"}
        [[ -r $vocab && -s $vocab ]] || fail_code VIO_E_PATH "Missing, empty or unreadable ORB vocabulary: $vocab. Set ORB_VOCABULARY to ORBvoc.txt."
        ;;
esac

case "$mode" in
    live)
        for executable in "$bin/camera_live" "$bin/orb_live_viewer"; do
            [[ -x "$executable" ]] || fail_code VIO_E_PATH "Missing or non-executable file: $executable"
        done
        for parameter in "$cfg" "$timing"; do
            [[ -f "$parameter" && -r "$parameter" ]] || fail_code VIO_E_CONFIG "Missing or unreadable parameter file: $parameter"
        done
        camera=/dev/gmsl/B
        [[ -c $camera ]] || fail_code VIO_E_CAMERA "Camera device not found: $camera. Check the B-port connection, device link and GMSL driver."
        [[ -r $camera && -w $camera ]] ||
            fail_code VIO_E_PERMISSION "Camera is not readable/writable by $(id -un): $camera. Check the video group/device permissions."
        printf '%s\n' "$camera: 1920x1441 UYVY; crop top row to 1920x1440; ORB input resize to 960x720." \
            'Trial calibration: fit_05 and camera–IMU time shift inherited from channel C.'
        runs=${ORB_HUB_RUNS:-"$root/runs"}
        mkdir -p -- "$runs"
        run=$(mktemp -d -- "$runs/live_XXXXXX")
        cp -- "$cfg" "$run/settings.yaml"
        cp -- "$timing" "$run/timing.json"
        python3 "$root/map_session.py" prepare "$root" "$run"
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
