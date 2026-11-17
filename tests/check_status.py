#!/usr/bin/env python3
"""Read-only status regression fixtures, including live-process identity safety."""
import csv
from contextlib import contextmanager
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('status_under_test', ROOT / 'jetson/status.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
POSE_HEADER = 'frame_idx,camera_ns,map_id,tracking_state,visual_ready,inertial_ready,valid,recovery_id,phase,tx,ty,tz,qx,qy,qz,qw'.split(',')
NOW = 20_000_000_000
RAW = 10_100_000_000
passed = 0


def check(name):
    global passed
    passed += 1
    print('PASS', name)


def write_json(path, value):
    path.write_text(json.dumps(value))


def pose(frame, timestamp, valid='1', phase='TRACKING', map_id='7', recovery='3'):
    return [frame, timestamp, map_id, '2', '1', valid, valid, recovery, phase, 0, 0, 0, 0, 0, 0, 1]


def write_poses(session, rows, suffix=''):
    with (session / 'poses.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(POSE_HEADER)
        writer.writerows(rows)
        stream.write(suffix)
    os.utime(session / 'poses.csv', ns=(NOW, NOW))


def runtime(state='RUNNING', **changes):
    value = {'schema': 1, 'state': state, 'supervisor_pid': 12345,
             'supervisor_start_ticks': '42', 'boot_id': 'same-boot', 'machine_id': 'same-machine',
             'mode': 'live-headless', 'started_unix_ns': NOW - 5_000_000_000,
             'updated_unix_ns': NOW, 'exit_status': None}
    value.update(changes)
    return value


@contextmanager
def identity(actual_ticks='42', process='S', boot='same-boot'):
    with patch.object(module, 'read_identity_text', side_effect=lambda p: boot if str(p).endswith('boot_id') else 'same-machine'), \
         patch.object(module, 'linux_start_ticks', return_value=(actual_ticks, process)), \
         patch.object(module.time, 'CLOCK_MONOTONIC_RAW', 4, create=True), \
         patch.object(module.time, 'clock_gettime_ns', return_value=RAW, create=True):
        yield


with tempfile.TemporaryDirectory(prefix='vio-status-test-') as temporary:
    root = Path(temporary).resolve()
    run = root / 'runs/live_current'
    session = run / 'session'
    session.mkdir(parents=True)
    write_json(session / 'runtime.json', runtime())
    rows = [pose(i, 7_000_000_000 + i * 100_000_000) for i in range(31)]
    write_poses(session, rows)
    write_json(run / 'map_plan.json', {'mode': 'auto', 'load': '/maps/old.osa', 'reference': '/maps/old.osa'})
    (session / 'orb.log').write_text('MAP_REFERENCE_LOADED id=7 keyframes=120; current velocity/bias still require recovery\n')
    (session / 'state_diagnostics.csv').write_text('frame_idx,camera_ns,health_reason,hard_fault\n30,10000000000,"quoted, residual \\\"text\\\"",0\n'.replace('\\\"', '""'))
    (session / 'recovery_timing.csv').write_text('frame_idx,camera_ns,recovery_method,recovery_search_ms,recovery_global_attempted,recovery_window_resets\n30,10000000000,LOCAL,0.5,0,1\n')
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['ready'] and status['stable_ready'], status
    assert status['program']['state'] == 'RUNNING' and status['tracking']['consecutive_valid_s'] == 3
    assert status['map_load']['state'] == 'LOADED' and status['recovery']['recovery_method'] == 'LOCAL'
    assert status['health']['health_reason'] == 'quoted, residual "text"', status['health']
    assert status['accuracy_verified'] is False
    check('running_fresh_valid_with_quoted_diagnostics')

    # Ignore only the unfinished last row. Never parse it as a complete invalid
    # or valid sample, and do not let a fresh mtime hide an old completed stamp.
    write_poses(session, rows, '31,10100000000,7,2,')
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['ready'] and status['tracking']['frame_idx'] == '30', status
    check('partial_csv_row_ignored')
    write_poses(session, rows[:-1], '30,10000000000,7,2,')
    with identity(), patch.object(module.time, 'clock_gettime_ns', return_value=20_000_000_000):
        status = module.snapshot(root, session, now_ns=NOW)
    assert not status['ready'] and status['program']['state'] == 'STALE', status
    check('partial_row_mtime_cannot_hide_old_completed_stamp')

    write_poses(session, rows, '31,broken\n')
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert not status['ready'] and status['data']['error'] and status['tracking'] is None
    check('complete_malformed_last_row_not_replaced_with_prior_valid')
    write_poses(session, rows + [pose(31, 9_900_000_000)])
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert not status['ready'] and 'not increasing' in status['data']['error']
    check('backward_timestamp_refuses_ready')

    write_poses(session, rows)
    with identity():
        status = module.snapshot(root, session, now_ns=NOW + 3_000_000_000)
    assert not status['ready'] and status['program']['state'] == 'STALE'
    check('alive_process_stale_file_not_ready')
    with identity(actual_ticks='different-pid-generation'):
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'STOPPED' and not status['ready']
    check('reused_pid_not_live')
    with identity(process='Z'):
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'STOPPED' and not status['ready']
    check('zombie_not_live')
    with identity(boot='another-boot'):
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'UNKNOWN' and not status['ready']
    check('foreign_or_previous_boot_never_ready')
    write_json(session / 'runtime.json', runtime(supervisor_pid=2147483640))
    with identity(actual_ticks=None):
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'STOPPED' and not status['ready']
    check('vanished_process_never_ready')

    write_json(session / 'runtime.json', runtime('FINISHED', exit_status=0))
    status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'STOPPED' and not status['ready']
    check('cleanly_finished_valid_tail_not_current_ready')
    write_json(session / 'runtime.json', runtime('FINALIZING'))
    write_json(session / 'live.json', {'status': 'STREAM_COMPLETED', 'orb_exit_code': 0})
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'FINALIZING' and not status['ready']
    check('final_report_during_finalizer_not_running_ready')
    (session / 'runtime.json').unlink()
    status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'STOPPED' and not status['ready']
    (session / 'live.json').unlink()
    status = module.snapshot(root, session, now_ns=NOW)
    assert status['program']['state'] == 'UNKNOWN' and not status['ready']
    check('legacy_logs_honest_liveness_unknown_or_finished')

    write_json(session / 'runtime.json', runtime())
    write_poses(session, rows + [pose(31, 10_100_000_000, valid='0', phase='VALIDATING_IMU')])
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['map_load']['state'] == 'LOADED' and not status['ready'] and not status['stable_ready']
    assert status['tracking']['phase'] == 'VALIDATING_IMU'
    check('loaded_map_does_not_certify_current_imu_ready')
    write_poses(session, rows[-3:])
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['ready'] and not status['stable_ready'], status
    check('single_valid_burst_not_stable_ready')
    write_poses(session, rows[:-1] + [pose(30, 10_000_000_000, map_id='8')])
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['ready'] and status['tracking']['consecutive_valid_frames'] == 1 and not status['stable_ready']
    check('map_switch_breaks_consecutive_stability')

    write_poses(session, rows)
    (session / 'orb.log').write_text('MAP_REFERENCE_LOADED id=7 keyframes=120; current velocity/bias still require recovery\n' + 'ordinary log line\n' * 10000)
    with identity():
        status = module.snapshot(root, session, now_ns=NOW)
    assert status['map_load']['state'] == 'LOADED'
    check('old_map_load_confirmation_survives_long_log')
    bounded = module.csv_tail(session / 'poses.csv', limit=300)
    assert bounded['truncated'] and len(bounded['rows']) < len(rows)
    assert bounded['rows'][-1]['frame_idx'] == '30'
    check('bounded_tail_retains_latest_complete_csv_record')

    # Explicit run and session paths both work; no platform/device checks.
    assert module.resolve_target(root, run)[0] == session
    assert module.resolve_target(root, session)[0] == session
    no_runs = root / 'empty'
    no_runs.mkdir()
    assert module.snapshot(no_runs)['program']['state'] == 'NO_SESSION'
    check('explicit_paths_and_no_session')
    startup = root / 'runs/startup_failed.log'
    startup.write_text('MAP_E_FORMAT: old Atlas format\n')
    os.utime(startup, ns=(time.time_ns() + 1_000_000, time.time_ns() + 1_000_000))
    status = module.snapshot(root)
    assert status['session'] is None and status['startup_log'] == str(startup) and 'MAP_E_FORMAT' in status['startup_detail']
    check('latest_failed_startup_is_not_hidden_by_old_run')

    before = {str(p.relative_to(root)): (p.stat().st_size, p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
    result = subprocess.run([sys.executable, str(ROOT / 'jetson/status.py'), '--root', str(root), str(session), '--json'],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    decoded = json.loads(result.stdout)
    assert decoded['session'] == str(session) and decoded['ready'] is False  # test identity is not real
    after = {str(p.relative_to(root)): (p.stat().st_size, p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
    assert before == after
    check('json_cli_reads_without_modifying_run')
    process = subprocess.Popen([sys.executable, str(ROOT / 'jetson/status.py'), '--root', str(root), str(session), '--watch', '--json', '--interval', '0.2'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    first = process.stdout.readline()
    assert json.loads(first)['session'] == str(session)
    process.send_signal(signal.SIGINT)
    out, err = process.communicate(timeout=5)
    assert process.returncode == 0 and err == '', (process.returncode, out, err)
    check('watch_sigint_only_ends_query')
    invalid = subprocess.run([sys.executable, str(ROOT / 'jetson/status.py'), str(root / 'missing')], capture_output=True, text=True)
    assert invalid.returncode == 2 and 'VIO_E_PATH' in invalid.stderr
    check('missing_explicit_path_reports_error')
print(f'STATUS_TEST_PASS cases={passed}')
