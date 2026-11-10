#!/usr/bin/env python3
"""Measure camera repeatability from a native EuRoC export and physical A/B/A holds."""
import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / 'tools' / 'orb_replay'))
from umi_quality import (
    angle, camera_pose, center, checked_holds, dump_json, euroc_poses,
    extrinsic, file_sha256, load_frames,
)


def measure(job):
    take = job.parent
    capture, frames = load_frames(take)
    holds = checked_holds(take, frames)
    euroc = job/'euroc'
    export = json.loads((euroc/'export.json').read_text())
    if (export['status'] != 'EXPORT_PASS'
            or export['source_capture_sha256'] != file_sha256(take/'capture.json')
            or export['source_hashes'] != capture['sha256']
            or export['camera_shift_in_source_ns'] != capture['camera_shift_ns']
            or export['additional_camera_shift_ns'] != 0
            or file_sha256(euroc/'settings.yaml') != capture['sha256']['settings.yaml']):
        raise ValueError('EuRoC input does not match the sealed recording.')
    input_stamps = [int(line) for line in (euroc/'times.txt').read_text().splitlines()]
    withheld = {item['frame_idx'] for item in export['withheld_frames']}
    if input_stamps != [f.camera_ns for f in frames if f.frame_idx not in withheld]:
        raise ValueError('EuRoC timestamps do not match the recorded frame index.')
    lookup = {round((float(ns)/1e9)*1e9): ns for ns in input_stamps}
    if len(lookup) != len(input_stamps):
        raise ValueError('Native timestamps alias.')
    body = euroc_poses(job/'CameraTrajectory.txt', lookup)
    Rbc, tbc = extrinsic(take/'settings.yaml')
    in_test = [f for f in frames if holds['A'][0] <= f.camera_raw_ns <= holds['return_A'][1]]
    report = {
        'status': 'REVIEW_REQUIRED',
        'scope': 'Native ORB largest-map trajectory, no physical ground-truth pose',
        'camera_shift_ns': capture['camera_shift_ns'],
        'recorded_frames': len(frames), 'solver_input_frames': len(input_stamps),
        'exported_pose_frames': sum(pose is not None for pose in body.values()),
        'test_frames': len(in_test),
        'test_frames_with_pose': sum(body.get(f.camera_ns) is not None for f in in_test),
    }
    log = (job/'orb.log').read_text(errors='replace')
    finalized_maps = re.findall(r'^  Map \d+ has (\d+) KFs$', log, re.M)
    completed_viba = log.rfind('end VIBA 2')
    try:
        if completed_viba < 0 or 'Reseting active map' in log[completed_viba:]:
            raise ValueError('Could not verify a mature inertial map at the end of replay.')
        if sum(int(n)>0 for n in finalized_maps) != 1:
            raise ValueError('Native replay did not finish with one nonempty map.')
        if not in_test or report['test_frames_with_pose'] != len(in_test):
            raise ValueError('Part of A→B→A lacks an exported pose.')
        if any((b.sequence-a.sequence) % 2**32 != 1 for a,b in zip(in_test,in_test[1:])):
            raise ValueError('Driver sequence gap during A→B→A.')
        gaps = np.diff([f.camera_raw_ns for f in frames])
        max_gap = max(100_000_000, 3*float(np.median(gaps)))
        if any(b.camera_raw_ns-a.camera_raw_ns > max_gap for a,b in zip(in_test,in_test[1:])):
            raise ValueError('Long image timestamp gap during A→B→A.')
        poses, summary = {}, {}
        for name,(start,end) in holds.items():
            selected = [f for f in in_test if start <= f.camera_raw_ns <= end]
            if len(selected)<3 or selected[0].camera_raw_ns-start>max_gap or end-selected[-1].camera_raw_ns>max_gap:
                raise ValueError(f'{name} has insufficient hold coverage.')
            samples = np.array([camera_pose(body[f.camera_ns], Rbc, tbc) for f in selected])
            p,q,summary[name] = center(samples)
            poses[name] = p,q
        p0,q0 = poses['A']
        for name,key in (('B','A_to_B'),('return_A','return_to_A')):
            p,q = poses[name]
            report[key] = {'translation_mm':float(1000*np.linalg.norm(p-p0)), 'rotation_deg':float(angle(q0,q))}
        report['holds'] = summary
        noise_mm = sum(summary[name]['translation_p95_mm'] for name in ('A','B'))
        noise_deg = sum(summary[name]['rotation_p95_deg'] for name in ('A','B'))
        if (report['A_to_B']['translation_mm']<=max(1e-6,noise_mm) and
                report['A_to_B']['rotation_deg']<=max(1e-5,noise_deg)):
            raise ValueError('The different physical pose B was unresolved above the static scatter.')
        report['status'] = 'REPEATABILITY_MEASURED'
    except ValueError as error:
        report['reason'] = str(error)
    path = job/'accuracy.json'
    dump_json(path, report)
    print(report['status'], path, flush=True)
    return 0 if report['status']=='REPEATABILITY_MEASURED' else 3


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('job', type=Path)
    try:
        sys.exit(measure(parser.parse_args().job.resolve(strict=True)))
    except (ValueError, KeyError, OSError) as error:
        sys.exit(str(error))
