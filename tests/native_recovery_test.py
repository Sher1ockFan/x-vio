#!/usr/bin/env python3
"""Execute analytic IMU recovery cases against the actual ARM64 shared core.
Uses known synthetic visual/IMU truth; this is not real-camera accuracy testing.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

build=Path(os.environ.get("MONO_IMU_BUILD_ROOT", "/build/mono-imu-vio/work"))
source=build/"source"
cv=Path("/build/orb-kit/opencv-4.8.0/usr/lib")
pango=Path("/build/orb-kit/prefix")
env={**os.environ, "LD_LIBRARY_PATH":str(source/"lib")+":"+str(cv)+":"+str(pango/"lib")+":"+str(source/"Thirdparty/DBoW2/lib")+":"+str(source/"Thirdparty/g2o/lib")}
with tempfile.TemporaryDirectory(prefix="imu-recovery-test-") as tmp:
    binary=Path(tmp)/"recovery_test"
    subprocess.run(["g++-11", "-std=c++11", "-O1", "-I"+str(source), "-I"+str(source/"include"),
        "-I"+str(source/"Thirdparty/Sophus"), "-isystem", "/usr/include/eigen3", "-isystem", str(cv.parent/"include/opencv4"),
        str(Path(__file__).with_name("imu_recovery_test.cc")), "-o", str(binary),
        "-L"+str(source/"lib"), "-L"+str(cv), "-Wl,-rpath-link,"+str(cv), "-Wl,-rpath-link,"+str(pango/"lib"),
        "-lORB_SLAM3", "-lopencv_core", "-pthread"], check=True, env=env)
    subprocess.run([str(binary)], check=True, env=env, timeout=60)
