#!/usr/bin/env python3
"""Run actual ARM64 Atlas save/load, guard, atomic-failure and recovery tests.

Synthetic geometry tests software invariants, not real-camera accuracy.
Usage inside prepared builder: python3 native_atlas_test.py PACKAGE.tar.gz
The companion atlas_persistence_test.cc must be beside this file.
"""
import contextlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

archive = Path(sys.argv[1]).resolve()
cv = Path('/build/orb-kit/opencv-4.8.0/usr/lib')
pango = Path('/build/orb-kit/prefix')
build = Path(os.environ.get('MONO_IMU_BUILD_ROOT', '/build/mono-imu-vio/work'))
keep = os.environ.get('KEEP_NATIVE_ATLAS_QA') == '1'
context = contextlib.nullcontext(tempfile.mkdtemp(prefix='mono-imu-atlas-')) if keep else tempfile.TemporaryDirectory(prefix='mono-imu-atlas-')
with context as td:
    root = Path(td)
    print('NATIVE_ATLAS_QA_DIRECTORY', root, flush=True)
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            assert not Path(member.name).is_absolute() and '..' not in Path(member.name).parts
        tar.extractall(root)
    package = root / 'mono_imu_vio'
    source = package / 'source/ORB_SLAM3'
    shutil.copy2(build / 'source/Thirdparty/g2o/config.h', source / 'Thirdparty/g2o/config.h')
    env = {**os.environ, 'LD_LIBRARY_PATH': f'{package}/lib:{cv}'}
    binary = root / 'atlas_test'
    flags = ['g++-11', '-std=c++11', '-O1', '-g', '-rdynamic', '-include', 'unistd.h',
             '-include', 'boost/serialization/library_version_type.hpp',
             '-I'+str(source), '-I'+str(source/'include'), '-I'+str(source/'include/CameraModels'),
             '-I'+str(source/'Thirdparty/Sophus'), '-isystem', '/usr/include/eigen3',
             '-isystem', str(cv.parent/'include/opencv4'), '-isystem', str(pango/'include'),
             str(Path(__file__).with_name('atlas_persistence_test.cc')), '-o', str(binary),
             '-L'+str(package/'lib'), '-L'+str(cv), '-Wl,-rpath-link,'+str(cv),
             '-Wl,-rpath-link,'+str(package/'lib'), '-lORB_SLAM3', '-lDBoW2', '-lpangolin',
             '-lopencv_core', '-lopencv_imgproc', '-lboost_serialization', '-lcrypto',
             '-lOpenGL', '-lGLX', '-lGLU', '-lGLEW', '-lEGL', '-lX11', '-pthread']
    subprocess.run(flags, check=True, env=env)
    yaml = '''%YAML:1.0
File.version: "1.0"
Camera.type: "KannalaBrandt8"
Camera1.fx: 149.4304286787079
Camera1.fy: 149.2890620791193
Camera1.cx: 320.3523723662162
Camera1.cy: 239.1705796196921
Camera1.k1: 0.21836372495045944
Camera1.k2: -0.05933403743095892
Camera1.k3: 0.008485453727516598
Camera1.k4: -0.0008888521529517888
Camera.width: 640
Camera.height: 480
Camera.fps: 30
Camera.RGB: 0
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [-0.999741270430994,0.00796214066832993,-0.0213071939260198,0.00158744070191347,
          0.0212122426228454,-0.0118253547448793,-0.999705057378459,-0.0146710347704421,
          -0.00821175742048125,-0.999898377487,0.011653400316289,0.0239608057868008, 0,0,0,1]
IMU.NoiseGyro: 0.001
IMU.NoiseAcc: 0.01
IMU.GyroWalk: 0.00001
IMU.AccWalk: 0.0001
IMU.Frequency: 500.0
ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
MapReuse.Enabled: 1
loopClosing: 1
'''
    vocabulary = package / 'Vocabulary/ORBvoc.txt'
    base = root / 'base.yaml'
    base.write_text(yaml)
    fixture = root / 'fixture.osa'
    def run(mode, settings, output):
        completed = subprocess.run([str(binary), mode, str(vocabulary), str(settings), str(output)],
                                   env=env, cwd=root, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, timeout=240)
        print(completed.stdout, end='')
        assert completed.returncode == 0, (mode, completed.returncode)
    run('atomic', base, root/'atomic.osa')
    assert not list(root.glob('*.tmp.*'))
    run('fixture', base, fixture)
    initial = fixture.read_bytes()
    focus = os.environ.get('NATIVE_ATLAS_FOCUS')
    if focus in ('stationary', 'biasstep', 'mapjump'):
        focused_settings = root / (focus+'.yaml')
        focused_output = root / (focus+'.osa')
        focused_settings.write_text(yaml+f'MapReuse.LoadPath: "{fixture}"\nMapReuse.SavePath: "{focused_output}"\n')
        run(focus, focused_settings, focused_output)
        print('NATIVE_ATLAS_FOCUSED_PASS', focus)
        raise SystemExit(0)
    save = root/'cycle.osa'
    cycle = root/'cycle.yaml'
    cycle.write_text(yaml+f'MapReuse.LoadPath: "{fixture}"\nMapReuse.SavePath: "{save}"\n')
    run('loadsave', cycle, save)
    report = json.loads((root/'map_save.json').read_text())
    assert report['status'] == 'SAVED' and report['keyframes'] == 12
    assert save.is_file() and fixture.read_bytes() == initial
    requested = root/'requested.yaml'
    requested.write_text(yaml+f'MapReuse.LoadPath: "{fixture}"\nMapReuse.SavePath: "{root}/requested.osa"\n')
    run('requested', requested, root/'requested.osa')
    assert json.loads((root/'map_save.json').read_text())['status'] == 'SAVED'
    # Verify the saved output is accepted by the actual System loader too.
    recovered = root/'recovered.osa'
    recovery = root/'recovery.yaml'
    recovery.write_text(yaml+f'MapReuse.LoadPath: "{save}"\nMapReuse.SavePath: "{recovered}"\n')
    run('recover', recovery, recovered)
    assert json.loads((root/'map_save.json').read_text())['status'] == 'SAVED'
    run('inspect', recovery, recovered)
    for mode in ('stationary', 'biasstep', 'mapjump'):
        mode_output = root / (mode+'.osa')
        mode_settings = root / (mode+'.yaml')
        mode_settings.write_text(yaml+f'MapReuse.LoadPath: "{fixture}"\nMapReuse.SavePath: "{mode_output}"\n')
        run(mode, mode_settings, mode_output)
    # Never reinterpret v1 serialization after recovered-root prior fields were added.
    old_format = root/'old_format.osa'
    old_format.write_bytes(initial.replace(b'MONO_IMU_ATLAS_2', b'MONO_IMU_ATLAS_1', 1))
    old_settings = root/'old_format.yaml'
    old_settings.write_text(yaml+f'MapReuse.LoadPath: "{old_format}"\n')
    run('expect_error', old_settings, save)
    disabled = root/'disabled_loop.yaml'
    disabled_output = root/'disabled_loop.osa'
    disabled.write_text(cycle.read_text().replace('loopClosing: 1', 'loopClosing: 0').replace(str(save), str(disabled_output)))
    run('recover', disabled, disabled_output)
    timeout_settings = root/'timeout.yaml'
    timeout_output = root/'timeout.osa'
    timeout_settings.write_text(cycle.read_text().replace(str(save), str(timeout_output))+'MapReuse.TimeoutSeconds: 3.0\n')
    run('timeout', timeout_settings, timeout_output)
    run('inspect', timeout_settings, timeout_output)
    second_save = root/'second.osa'
    second_settings = root/'second.yaml'
    second_settings.write_text(yaml+f'MapReuse.LoadPath: "{recovered}"\nMapReuse.SavePath: "{second_save}"\n')
    run('loadsave', second_settings, second_save)
    run('inspect', second_settings, second_save)
    unwritable = root/'unwritable.yaml'
    unwritable.write_text(yaml+f'MapReuse.LoadPath: "{fixture}"\nMapReuse.SavePath: "{root}/no-such-directory/fail.osa"\n')
    run('expect_save_error', unwritable, save)
    # No mature map: do not overwrite an existing known-good destination.
    empty = root/'empty.yaml'
    empty.write_text(yaml+f'MapReuse.SavePath: "{fixture}"\n')
    run('empty', empty, fixture)
    assert fixture.read_bytes() == initial
    assert json.loads((root/'map_save.json').read_text())['status'] == 'SKIPPED'
    changed = root/'changed.yaml'
    changed.write_text(cycle.read_text().replace('Camera1.fx: 149.4304286787079', 'Camera1.fx: 401.0'))
    run('expect_error', changed, save)
    truncated = root/'truncated.osa'
    truncated.write_bytes(fixture.read_bytes()[:50])
    damaged = root/'damaged.yaml'
    damaged.write_text(yaml+f'MapReuse.LoadPath: "{truncated}"\n')
    run('expect_error', damaged, save)
    assert not list(root.glob('*.tmp.*'))
    print('NATIVE_ATLAS_PERSISTENCE_AND_RECOVERY_PASS')
