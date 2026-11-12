#!/usr/bin/env bash
# Run in the existing Jammy ARM64 builder. All output is in a private volume path.
set -euo pipefail
root=${ORB_STOCK_BUILD_ROOT:-/build/orb-stock}
src=$root/source
cv=/build/orb-kit/opencv-4.8.0/usr/lib
pango=/build/orb-kit/prefix
jobs=${ORB_STOCK_JOBS:-2}
[[ $jobs =~ ^[1-9][0-9]*$ ]]
[[ $(uname -sm) == 'Linux aarch64' && $(g++-11 -dumpmachine) == aarch64-linux-gnu ]]
. /etc/os-release
[[ $VERSION_ID == 22.04 ]]
[[ -f $cv/cmake/opencv4/OpenCVConfig.cmake && -f $pango/lib/cmake/Pangolin/PangolinConfig.cmake ]] || {
    echo 'Missing the prepared Jetson OpenCV 4.8.0 or Pangolin prefix in /build/orb-kit.' >&2
    exit 1
}
cd "$src"
sha256sum --quiet -c "$root/source.sha256"
export CCACHE_DIR=/ccache
export LD_LIBRARY_PATH="$cv"

# Translate the upstream host-detection flag at the compiler boundary.
cat > "$root/compiler-launcher.new" <<'EOF'
#!/usr/bin/env bash
set -eu
args=()
for arg; do
    if [[ $arg == -march=native ]]; then arg=-march=armv8-a; fi
    args+=("$arg")
done
exec ccache "${args[@]}"
EOF
chmod 755 "$root/compiler-launcher.new"
if ! cmp -s "$root/compiler-launcher.new" "$root/compiler-launcher"; then
    mv "$root/compiler-launcher.new" "$root/compiler-launcher"
else
    rm "$root/compiler-launcher.new"
fi

args=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11
      "-DCMAKE_C_COMPILER_LAUNCHER=$root/compiler-launcher"
      "-DCMAKE_CXX_COMPILER_LAUNCHER=$root/compiler-launcher"
      -DCMAKE_C_FLAGS=-march=armv8-a
      -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON '-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib;$ORIGIN'
      -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF "-DOpenCV_DIR=$cv/cmake/opencv4"
      "-DCMAKE_EXE_LINKER_FLAGS=-Wl,-rpath-link,$cv"
      "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-rpath-link,$cv")

echo '[1/2] Building upstream DBoW2'
cmake -S "$src/Thirdparty/DBoW2" -B "$root/dbow-build" "${args[@]}" -DCMAKE_CXX_FLAGS=-march=armv8-a
cmake --build "$root/dbow-build" --target DBoW2 --parallel "$jobs"

echo '[2/2] Building mono_imu_vio core and the EuRoC example'
cmake -S "$src" -B "$root/orb-build" "${args[@]}" "-DPangolin_DIR=$pango/lib/cmake/Pangolin" \
    '-DCMAKE_CXX_FLAGS=-march=armv8-a -include unistd.h -include boost/serialization/library_version_type.hpp'
if ! cmake --build "$root/orb-build" --target mono_inertial_euroc --parallel "$jobs" 2>&1 | tee "$root/compile.log"; then
    if (( jobs == 1 )) || ! grep -q 'Killed signal terminated program cc1plus' "$root/compile.log"; then
        exit 1
    fi
    echo 'Compiler ran out of memory. Retrying unfinished targets with one job.'
    cmake --build "$root/orb-build" --target mono_inertial_euroc --parallel 1
fi
sha256sum --quiet -c "$root/source.sha256"
echo 'FORK_SOURCE_FILES_VERIFIED'
