#!/usr/bin/env python3
"""Replay logged health-gate inputs, not camera/IMU data or trajectories.

Default: compile the current standalone production TrackingHealth.cc.
--package DIRECTORY_OR_TAR_GZ: link the packaged native core instead (ARM64
Linux package requires the matching ARM64 builder/runtime dependencies).
--expect legacy: negative control against the previous package/source.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SCALARS = {
    'dynamics_available', 'map_updated', 'output_pose_baseline', 'stationary_candidate',
    'visual_inliers', 'visual_cells', 'visual_rms_px', 'fused_rms_px',
    'visual_translation_error_m', 'visual_rotation_error_rad',
    'pose_step_m', 'pose_step_dt_s', 'output_pose_step_m', 'output_pose_step_dt_s',
    'stationary_duration_s', 'stationary_displacement_m',
}
VECTORS = {'velocity_w', 'gyro_bias', 'accel_bias'}


def literal(value):
    if value is None:
        return 'HealthUnknown()'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Unexpected nonfinite or nonnumeric fixture value')
    return repr(value)


def package_root(path, temporary):
    if path.is_dir():
        return path
    dest = temporary / 'package'
    dest.mkdir()
    # Only extract the one standard-C++ header and regular runtime libraries.
    # No links, device entries, paths outside the archive root, or executable
    # archive content are extracted or run by this helper.
    with tarfile.open(path, 'r|gz') as tar:
        for member in tar:
            p = PurePosixPath(member.name)
            if p.is_absolute() or '..' in p.parts or not p.parts or p.parts[0] != 'mono_imu_vio':
                raise ValueError('Unsafe package path: ' + member.name)
            rel = p.relative_to('mono_imu_vio')
            wanted = str(rel) == 'source/ORB_SLAM3/include/TrackingHealth.h' or (
                len(rel.parts) == 2 and rel.parts[0] == 'lib' and rel.name.endswith('.so'))
            if wanted:
                if not member.isfile():
                    raise ValueError('Expected regular package file: ' + member.name)
                target = dest / str(rel)
                target.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as src, target.open('wb') as dst:
                    shutil.copyfileobj(src, dst)
    return dest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--package', type=Path)
    ap.add_argument('--expect', choices=('diagnostic', 'legacy'), default='diagnostic')
    args = ap.parse_args()
    fixture = json.loads(Path(__file__).with_name('reprojection_fixture.json').read_text())
    rows = fixture['rows']
    expected_frames = [245, 1529, 1624, 1787, 2001, 2139, 2224]
    assert [r['source_frame'] for r in rows] == expected_frames
    source = ['#include "TrackingHealth.h"', '#include <iostream>', '#include <stdexcept>',
              'using namespace ORB_SLAM3;', 'int main(){try{']
    for row in rows:
        d = row['diagnostics']
        assert set(d) == SCALARS | VECTORS
        assert row['finite_pose'] is True and 8 < d['fused_rms_px'] < 10
        source.append('{ TrackingHealthDiagnostics d;')
        for key in sorted(SCALARS):
            source.append('d.' + key + '=' + literal(d[key]) + ';')
        for key in sorted(VECTORS):
            assert len(d[key]) == 3
            source.append('d.' + key + '={{' + ','.join(map(literal, d[key])) + '}};')
        source.append('const bool accepted=EvaluateTrackingHealth(d,true);')
        if args.expect == 'diagnostic':
            condition = '!accepted || !d.evaluated || !d.accepted || d.hard_fault'
        else:
            condition = 'accepted || !d.evaluated || d.accepted || d.hard_fault || d.reason!="health: excessive/nonfinite reprojection residual"'
        source.append('if(' + condition + ') throw std::runtime_error("frame ' + str(row['source_frame']) + ': "+d.reason);')
        source.append('std::cout<<"frame=' + str(row['source_frame']) + ' accepted="<<accepted<<" reason="<<d.reason<<"\\n"; }')
    source += ['return 0;}catch(const std::exception& e){std::cerr<<e.what()<<"\\n";return 1;}}']
    compiler = os.environ.get('CXX') or shutil.which('g++-11') or shutil.which('c++') or shutil.which('g++')
    if not compiler:
        raise SystemExit('A C++11 compiler is required')
    with tempfile.TemporaryDirectory(prefix='vio-rms-gate-') as temp:
        temporary = Path(temp)
        test = temporary / 'fixture.cc'
        binary = temporary / 'fixture'
        test.write_text('\n'.join(source) + '\n')
        env = dict(os.environ)
        core_digest = None
        if args.package:
            package = package_root(args.package.resolve(), temporary)
            core = package / 'lib/libORB_SLAM3.so'
            core_digest = hashlib.sha256(core.read_bytes()).hexdigest()
            cv = Path(os.environ.get('MONO_IMU_OPENCV_LIB', '/build/orb-kit/opencv-4.8.0/usr/lib'))
            env['LD_LIBRARY_PATH'] = str(package / 'lib') + ':' + str(cv) + ':' + env.get('LD_LIBRARY_PATH', '')
            command = [compiler, '-std=c++11', '-O2', '-I' + str(package / 'source/ORB_SLAM3/include'),
                       str(test), '-o', str(binary), '-L' + str(package / 'lib'),
                       '-Wl,-rpath,' + str(package / 'lib'), '-Wl,-rpath-link,' + str(package / 'lib'),
                       '-Wl,-rpath-link,' + str(cv), '-lORB_SLAM3', '-pthread']
        else:
            command = [compiler, '-std=c++11', '-O2', '-I' + str(ROOT / 'vio_src/include'),
                       str(ROOT / 'vio_src/src/TrackingHealth.cc'), str(test), '-o', str(binary)]
        subprocess.run(command, check=True, env=env)
        subprocess.run([str(binary)], check=True, env=env, timeout=30)
    print(json.dumps({'result': 'REPROJECTION_LOGGED_GATE_PASS', 'expected_policy': args.expect,
                      'cases': len(rows), 'backend': 'packaged_native_core' if args.package else 'production_source',
                      'core_sha256': core_digest, 'raw_image_imu_replay': False,
                      'hardware_accuracy_validated': False}, indent=2))


if __name__ == '__main__':
    main()
