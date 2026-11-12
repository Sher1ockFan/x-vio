#!/usr/bin/env bash
# Run on the Mac with the prepared ARM64 Docker environment.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
adapter=$here/adapter
jetson=$here/jetson
builder=$here/docker
params=$here/params
upstream=$here/ORB_SLAM3
manifest=$here/ORB_SLAM3.sha256
container=${ORB_CONTAINER:-orin-r365-dev}
jobs=${ORB_HUB_JOBS:-3}
[[ $container != -* && -n $container && $jobs =~ ^[1-9][0-9]*$ ]]
docker=$(command -v docker || printf '%s\n' /Applications/Docker.app/Contents/Resources/bin/docker)
scratch=$(mktemp -d)
trap 'rm -r -- "$scratch"' EXIT

(cd "$upstream"; shasum -a 256 --quiet -c "$manifest")
mkdir -p "$scratch/input/"{orb_replay,cam_imu_motion,params}
cp "$manifest" "$scratch/input/upstream.sha256"
cp "$builder/"{compile.sh,package.sh,extra.sh,runtime.py} "$scratch/input/"
cp "$adapter/viewer.cc" "$scratch/input/"
cp "$jetson/"{start.sh,check.sh,README.md,playback.py,live_console.py,performance.py,trajectory.sh,export_euroc.py,check_return.py,trajectory_view.html} "$scratch/input/"
cp "$adapter/orb_replay/"{live.py,umi.py,umi_quality.py,camera_live.c} "$scratch/input/orb_replay/"
cp "$adapter/cam_imu_motion/"{camera.c,imu.py,timing_core.py} "$scratch/input/cam_imu_motion/"
cp "$params/"{camera,cam_imu,imu,target,orb_trial}.yaml "$params/timing.json" "$scratch/input/params/"

if [[ $("$docker" inspect --format '{{.State.Status}}' "$container") != running ]]; then
    "$docker" start "$container" >/dev/null
fi
# A verified, identical source tree already in Docker can reuse the build cache.
if ! "$docker" exec "$container" test -d /build/orb-hub/upstream; then
    mkdir "$scratch/input/upstream"
    cut -c67- "$manifest" |
        COPYFILE_DISABLE=1 tar --no-xattrs -C "$upstream" -cf - -T - |
        tar -C "$scratch/input/upstream" -xf -
fi
COPYFILE_DISABLE=1 tar --no-xattrs -cf - -C "$scratch/input" . |
    "$docker" exec -i -e "ORB_STOCK_JOBS=$jobs" "$container" bash -euo pipefail -c '
        root=/build/orb-hub
        mkdir -p "$root"
        exec 9>"$root/lock"
        flock -n 9 || { echo "Hub build is already running." >&2; exit 1; }
        input=$(mktemp -d "$root/input.XXXXXX")
        trap '\''rm -r -- "$input"'\'' EXIT
        tar -xf - -C "$input"

        printf "%s  %s\n" dd7ad17553b122cc69055ecfa9aefd9be05fbfbc7c154f6b1688c7d2d691a232 \
            "$input/upstream.sha256" | sha256sum --quiet -c -
        if [[ ! -d "$root/upstream" ]]; then
            [[ -d "$input/upstream" ]] || { echo "Docker source cache disappeared. Run the build again." >&2; exit 1; }
            (cd "$input/upstream"; sha256sum --quiet -c "$input/upstream.sha256")
            mv -- "$input/upstream" "$root/upstream"
        fi
        cp "$input/upstream.sha256" "$root/upstream.sha256"
        export ORB_STOCK_BUILD_ROOT="$root" ORB_STOCK_INPUT_ROOT="$input"
        export ORB_STOCK_PACKAGE_NAME=name_test ORB_STOCK_ARCHIVE_NAME=name_test.tar.gz
        export ORB_STOCK_HUB_CAPTURE=1
        bash "$input/compile.sh"
        bash "$input/package.sh"
        sha256sum "$root/name_test.tar.gz"
    '
mkdir -p "$here/dist"
"$docker" cp "$container:/build/orb-hub/name_test.tar.gz" "$scratch/name_test.tar.gz"
mv -- "$scratch/name_test.tar.gz" "$here/dist/name_test.tar.gz"
printf 'ORB_HUB_BUILD_OK %s\n' "$here/dist/name_test.tar.gz"
