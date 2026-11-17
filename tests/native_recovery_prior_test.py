#!/usr/bin/env python3
"""Check real LocalInertialBA correction and prior persistence in an ARM64 package.

The stationary synthetic fixture has exact truth. Passing does not establish
sensor accuracy or static detection on physical hardware.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

archive=Path(sys.argv[1]).resolve()
build=Path(os.environ.get("MONO_IMU_BUILD_ROOT","/build/mono-imu-vio/work"))
cv=Path("/build/orb-kit/opencv-4.8.0/usr/lib")
pango=Path("/build/orb-kit/prefix")
with tempfile.TemporaryDirectory(prefix="recovery-prior-") as tmp:
    root=Path(tmp)
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            assert not Path(member.name).is_absolute() and ".." not in Path(member.name).parts
        tar.extractall(root)
    package=root/"mono_imu_vio"
    source=package/"source/ORB_SLAM3"
    shutil.copy2(build/"source/Thirdparty/g2o/config.h",source/"Thirdparty/g2o/config.h")
    env={**os.environ,"LD_LIBRARY_PATH":f"{package}/lib:{cv}"}
    binary=root/"prior_test"
    command=["g++-11","-std=c++11","-O1","-g","-include","unistd.h",
        "-include","boost/serialization/library_version_type.hpp",
        "-I"+str(source),"-I"+str(source/"include"),"-I"+str(source/"include/CameraModels"),
        "-I"+str(source/"Thirdparty/Sophus"),"-isystem","/usr/include/eigen3",
        "-isystem",str(cv.parent/"include/opencv4"),"-isystem",str(pango/"include"),
        str(Path(__file__).with_name("recovery_prior_test.cc")),"-o",str(binary),
        "-L"+str(package/"lib"),"-L"+str(cv),"-Wl,-rpath-link,"+str(cv),
        "-Wl,-rpath-link,"+str(package/"lib"),"-lORB_SLAM3","-lDBoW2","-lpangolin",
        "-lopencv_core","-lopencv_imgproc","-lboost_serialization","-lcrypto",
        "-lOpenGL","-lGLX","-lGLU","-lGLEW","-lEGL","-lX11","-pthread"]
    subprocess.run(command,check=True,env=env)
    subprocess.run([str(binary),str(package/"Vocabulary/ORBvoc.txt")],check=True,env=env,timeout=120)
