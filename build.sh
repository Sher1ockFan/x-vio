#!/usr/bin/env bash
# Build the modified mono+IMU-only core inside the prepared ARM64 Docker container.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
container=${ORB_CONTAINER:-orin-r365-dev}
jobs=${ORB_HUB_JOBS:-3}
[[ $container != -* && -n $container && $jobs =~ ^[1-9][0-9]*$ ]]
docker=$(command -v docker || printf '%s\n' /Applications/Docker.app/Contents/Resources/bin/docker)
scratch=$(mktemp -d)
trap 'rm -r -- "$scratch"' EXIT
manifest=$here/source.sha256
(cd "$here/ORB_SLAM3"; shasum -a 256 --quiet -c "$manifest")
root=/build/mono-vio-60/work
mkdir -p "$scratch/input/"{orb_replay,cam_imu_motion,params,source,provenance}
cp "$manifest" "$scratch/input/source.sha256"
cp "$here/provenance/"* "$scratch/input/provenance/"
cp "$here/docker/"{compile.sh,package.sh,extra.sh,runtime.py} "$scratch/input/"
cp "$here/adapter/"{viewer.cc,camera_preview.cc} "$scratch/input/"
cp "$here/jetson/"{start.sh,check.sh,README.md,playback.py,live_console.py,map_session.py,performance.py,status.py,trajectory.sh,export_euroc.py,check_return.py,trajectory_view.html,camera_config.py} "$scratch/input/"
cp "$here/adapter/orb_replay/"{live.py,umi.py,umi_quality.py,camera_live.c,camera_profile.py,gray.h} "$scratch/input/orb_replay/"
cp "$here/adapter/cam_imu_motion/"{imu.py,timing_core.py} "$scratch/input/cam_imu_motion/"
cp -R "$here/params/." "$scratch/input/params/"
cut -c67- "$manifest" |
    COPYFILE_DISABLE=1 tar --no-xattrs -C "$here/ORB_SLAM3" -cf - -T - |
    tar -C "$scratch/input/source" -xf -
if [[ $("$docker" inspect --format '{{.State.Status}}' "$container") != running ]]; then
    "$docker" start "$container" >/dev/null
fi
COPYFILE_DISABLE=1 tar --no-xattrs -cf - -C "$scratch/input" . |
    "$docker" exec -i -e "ORB_STOCK_JOBS=$jobs" -e "ORB_STOCK_BUILD_ROOT=$root" "$container" bash -euo pipefail -c '
        root=$ORB_STOCK_BUILD_ROOT
        mkdir -p "$root"
        exec 9>/build/mono-vio-60/lock
        flock -n 9 || { echo "mono_vio_60 build is already running." >&2; exit 1; }
        input=$(mktemp -d "$root/input.XXXXXX")
        trap '\''rm -r -- "$input"'\'' EXIT
        tar -xf - -C "$input"
        (cd "$input/source"; sha256sum --quiet -c "$input/source.sha256")
        # Update changed files only; keep generated objects and remove sources
        # dropped from the previous manifest. The original cache is untouched.
        python3 - "$input" "$root" <<'\''PY'\''
from pathlib import Path
import hashlib, os, shutil, sys
incoming, root = map(Path, sys.argv[1:])
def entries(path):
    return {line[66:]: line[:64] for line in path.read_text().splitlines()} if path.exists() else {}
new = entries(incoming / "source.sha256")
old = entries(root / "source.sha256")
for rel in old.keys() - new.keys():
    p = root / "source" / rel
    if p.is_file(): p.unlink()
for rel, digest in new.items():
    src, dst = incoming / "source" / rel, root / "source" / rel
    if not dst.is_file() or hashlib.sha256(dst.read_bytes()).hexdigest() != digest:
        dst.parent.mkdir(parents=True, exist_ok=True)
        # A staging edit can predate an object produced by an earlier concurrent
        # build. Preserve permissions, not that mtime: changed bytes must make
        # Ninja rebuild even when the previous object is newer than the edit.
        shutil.copyfile(src, dst)
        shutil.copymode(src, dst)
        os.utime(dst, None)
PY
        (cd "$root/source"; sha256sum --quiet -c "$input/source.sha256")
        cp "$input/source.sha256" "$root/source.sha256"
        export ORB_STOCK_INPUT_ROOT="$input"
        export ORB_STOCK_PACKAGE_NAME=mono_vio_60 ORB_STOCK_ARCHIVE_NAME=mono_vio_60.tar.gz
        export ORB_STOCK_HUB_CAPTURE=1
        bash "$input/compile.sh"
        bash "$input/package.sh"
        sha256sum "$root/mono_vio_60.tar.gz"
    '
mkdir -p "$here/dist"
"$docker" cp "$container:$root/mono_vio_60.tar.gz" "$scratch/mono_vio_60.tar.gz"
mv -- "$scratch/mono_vio_60.tar.gz" "$here/dist/mono_vio_60.tar.gz"
(cd "$here/dist"; shasum -a 256 mono_vio_60.tar.gz > mono_vio_60.tar.gz.sha256)
printf 'MONO_VIO_60_BUILD_OK %s\n' "$here/dist/mono_vio_60.tar.gz"
