#!/usr/bin/env python3
"""ARM64 integration smoke: real bridge/parser/clock-fit/viewer, simulated sensors.

Run inside the existing build container with Python 3 and numpy. No hardware is
used. The camera timing rows describe fixture work, not V4L2/C-helper timing.
Artifacts are retained under /tmp/vio-bridge-timing-* for inspection on failure.
"""
import contextlib
import csv
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types


ARCHIVE = Path(os.environ.get('MONO_IMU_ARCHIVE', 'mono_imu_vio.tar.gz'))
CV_LIB = '/build/orb-kit/opencv-4.8.0/usr/lib'
SECONDS = 4


def raw_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def read_csv(path):
    with path.open(newline='') as stream:
        return [{k: int(v, 0) if v.startswith('0x') else int(v)
                 for k, v in row.items()} for row in csv.DictReader(stream)]


CAMERA_FIXTURE = r'''#!/usr/bin/env python3
import csv
import os
import select
import signal
import struct
import sys
import time
import numpy as np

assert sys.argv[1:] == []
assert os.environ.get('VIO_TIMING') == '1'
stop = False
def stopped(_signum, _frame):
    global stop
    stop = True
signal.signal(signal.SIGTERM, stopped)
signal.signal(signal.SIGINT, stopped)
def raw_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
pixels = np.random.default_rng(39).integers(0, 256, (1440, 1920), dtype=np.uint8).tobytes()
os.set_blocking(1, False)
print('SIMULATED_CAMERA: static random texture; event=RAW now-25ms; no V4L2, exposure, DQBUF, or C gray extraction measured', file=sys.stderr, flush=True)
with open('camera_timing.csv', 'x', newline='', buffering=1) as stream:
    writer = csv.writer(stream)
    writer.writerow('sequence,driver_timestamp_ns,camera_raw_ns,dqbuf_raw_ns,gray_start_raw_ns,gray_done_raw_ns,write_start_raw_ns,write_end_raw_ns,bytesused,flags'.split(','))
    sequence = 0
    next_frame = raw_ns()
    while not stop:
        delay = (next_frame - raw_ns()) / 1e9
        if delay > 0:
            time.sleep(delay)
        if stop:
            break
        # These named stages intentionally describe the Python fixture only.
        fixture_receipt = raw_ns()
        event = fixture_receipt - 25_000_000
        gray_start = raw_ns()
        frame = pixels
        gray_done = raw_ns()
        write_start = raw_ns()
        complete = True
        for part in (struct.pack('<q', event), frame):
            pending = memoryview(part)
            while pending:
                if stop:
                    complete = False
                    break
                if not select.select([], [1], [], 0.1)[1]:
                    continue
                try:
                    n = os.write(1, pending)
                    pending = pending[n:]
                except BlockingIOError:
                    continue
            if not complete:
                break
        if not complete:
            break
        write_end = raw_ns()
        writer.writerow((sequence, event, event, fixture_receipt, gray_start,
                         gray_done, write_start, write_end, len(frame), 0))
        sequence += 1
        # 30 Hz target, with real scheduling/pipe delays retained in timestamps.
        next_frame += 1_000_000_000 // 30
        if next_frame < raw_ns() - 1_000_000_000 // 30:
            next_frame = raw_ns()
'''


def x11_api():
    lib = C.CDLL('libX11.so.6')
    lib.XOpenDisplay.argtypes = [C.c_char_p]
    lib.XOpenDisplay.restype = C.c_void_p
    lib.XDefaultRootWindow.argtypes = [C.c_void_p]
    lib.XDefaultRootWindow.restype = C.c_ulong
    lib.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_char_p)]
    lib.XQueryTree.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong),
                              C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)),
                              C.POINTER(C.c_uint)]
    lib.XFree.argtypes = [C.c_void_p]
    lib.XCloseDisplay.argtypes = [C.c_void_p]
    return lib


def windows(lib, display, win):
    name = C.c_char_p()
    if lib.XFetchName(display, win, C.byref(name)) and name.value:
        yield name.value.decode(errors='replace')
        lib.XFree(name)
    root, parent, count = C.c_ulong(), C.c_ulong(), C.c_uint()
    children = C.POINTER(C.c_ulong)()
    if lib.XQueryTree(display, win, C.byref(root), C.byref(parent),
                      C.byref(children), C.byref(count)):
        try:
            for i in range(count.value):
                yield from windows(lib, display, children[i])
        finally:
            if children:
                lib.XFree(children)


def main():
    assert platform.machine() == 'aarch64', platform.machine()
    tmp = Path(tempfile.mkdtemp(prefix='vio-bridge-timing-'))
    print('BRIDGE_TIMING_ARTIFACTS=' + str(tmp), flush=True)
    subprocess.run(['tar', '-xzf', str(ARCHIVE), '-C', str(tmp)], check=True)
    pkg = tmp / 'mono_imu_vio'
    subprocess.run(['sha256sum', '--quiet', '-c', 'SHA256SUMS'], cwd=pkg, check=True)
    assert (pkg / 'bin/orb_live_viewer').readlink() == Path('stock_viewer')
    core_hash = digest(pkg / 'lib/libORB_SLAM3.so')
    sys.path.insert(0, str(pkg / 'tools/orb_replay'))
    import live
    assert live.WARMUP_US == 10_000_000
    live_path = Path(live.__file__).resolve()
    assert live_path == (pkg / 'tools/orb_replay/live.py').resolve()
    original_read_imu = live.read_imu
    original_serial_port = live.serial_port
    fixture_stats = {'samples_written': 0, 'max_writer_lateness_ms': 0.0,
                     'writer_errors': [], 'serial_open_raw_ns': None,
                     'serial_close_raw_ns': None}

    @contextlib.contextmanager
    def fixture_serial_port():
        read_fd, write_fd = os.pipe()
        os.set_blocking(read_fd, False)
        os.set_blocking(write_fd, False)
        stop = threading.Event()
        base_raw = raw_ns()
        base_tick = base_raw // 1000
        fixture_stats['serial_open_raw_ns'] = base_raw

        def produce():
            count = 0
            try:
                while not stop.is_set():
                    due_raw = base_raw + count * 2_000_000
                    delay = (due_raw - raw_ns()) / 1e9
                    if delay > 0 and stop.wait(delay):
                        break
                    now = raw_ns()
                    fixture_stats['max_writer_lateness_ms'] = max(
                        fixture_stats['max_writer_lateness_ms'], (now - due_raw) / 1e6)
                    # Preserve every nominal 500-Hz sample across scheduling gaps.
                    due_count = max(count + 1, (now - base_raw) // 2_000_000 + 1)
                    encoded = bytearray()
                    for index in range(count, due_count):
                        timer = (base_tick + index * 2000) & 0xffffffff
                        body = live.imu.AHRS_HEADER + struct.pack(
                            '<I10f', timer, 0, 0, 0, 0, 0, 1, 0, 0, 0, 25)
                        encoded += body + struct.pack('<I', live.imu.crc32_forsense(body))
                    pending = memoryview(encoded)
                    while pending and not stop.is_set():
                        if not select.select([], [write_fd], [], 0.05)[1]:
                            continue
                        try:
                            written = os.write(write_fd, pending)
                            pending = pending[written:]
                        except BlockingIOError:
                            continue
                    if pending:
                        break
                    fixture_stats['samples_written'] += due_count - count
                    count = due_count
            except Exception as exc:
                fixture_stats['writer_errors'].append(repr(exc))

        writer = threading.Thread(target=produce, name='fixture-ahrs-500hz', daemon=True)
        writer.start()
        try:
            yield read_fd
        finally:
            stop.set()
            writer.join(timeout=2)
            assert not writer.is_alive(), 'Fixture serial writer did not stop'
            fixture_stats['serial_close_raw_ns'] = raw_ns()
            os.close(write_fd)
            os.close(read_fd)

    live.serial_port = fixture_serial_port
    session = tmp / 'session'
    session.mkdir()
    bindir = tmp / 'fixture-bin'
    bindir.mkdir()
    (bindir / 'orb_live_viewer').symlink_to(pkg / 'bin/stock_viewer')
    camera = bindir / 'camera_live'
    camera.write_text(CAMERA_FIXTURE)
    camera.chmod(0o755)
    provenance = {
        'test_scope': 'integration smoke with simulated sensors, not hardware performance',
        'real_components': ['packaged live.run', 'read_imu', 'AHRS parser and CRC',
                            '0.5 s stale drain', '10 s clock fit and RollingClock',
                            'native stock_viewer and unmodified ORB core',
                            'native tracking_timing.csv', 'packaged performance.py'],
        'simulated_components': {
            'serial': 'pipe carrying CRC-valid stationary AHRS at nominal 500 Hz; device tick advances against CLOCK_MONOTONIC_RAW',
            'camera': 'Python helper sending static random 1920x1440 gray texture at target 30 Hz; event RAW=fixture receipt-25 ms',
            'camera_timing.csv': 'MOCKED CAMERA STAGES: timestamps measure Python fixture receipt, buffer selection and pipe writes; no native C helper, hardware driver, exposure or DQBUF measurements',
        },
        'hardware_accuracy_validated': False,
        'performance_interpretation': 'native call timing under synthetic input only; no claimed stable tracking, sensor latency, or deployed-device throughput',
    }
    (session / 'fixture_provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    (tmp / 'home').mkdir()
    os.environ.update(VIO_TIMING='1', LIBGL_ALWAYS_SOFTWARE='1',
                      LD_LIBRARY_PATH=str(pkg / 'lib') + ':' + CV_LIB,
                      HOME=str(tmp / 'home'), PYTHONDONTWRITEBYTECODE='1')
    os.environ.pop('ORB_VOCABULARY', None)
    read_fd, write_fd = os.pipe()
    xvfb_log = (tmp / 'xvfb.log').open('wb')
    server = subprocess.Popen(['Xvfb', '-displayfd', str(write_fd), '-screen', '0',
                               '1280x1024x24', '-nolisten', 'tcp'], pass_fds=(write_fd,),
                              stdout=xvfb_log, stderr=xvfb_log)
    os.close(write_fd)
    observer = None
    observer_stop = threading.Event()
    observer_errors, seen = [], set()
    try:
        assert select.select([read_fd], [], [], 10)[0], (tmp / 'xvfb.log').read_text()
        display = b':' + os.read(read_fd, 40).strip()
        os.environ['DISPLAY'] = display.decode()
        lib = x11_api()

        def observe():
            handle = lib.XOpenDisplay(display)
            if not handle:
                observer_errors.append('Cannot open Xvfb display')
                return
            try:
                root = lib.XDefaultRootWindow(handle)
                while not observer_stop.wait(.03):
                    seen.update(windows(lib, handle, root))
            except Exception as exc:
                observer_errors.append(repr(exc))
            finally:
                lib.XCloseDisplay(handle)

        observer = threading.Thread(target=observe, name='real-x11-window-observer')
        observer.start()
        args = types.SimpleNamespace(bindir=bindir, vocabulary=pkg / 'Vocabulary/ORBvoc.txt',
                                     yaml=pkg / 'params/orb_trial.yaml', viewer=True, seconds=SECONDS)
        started = time.monotonic()
        try:
            live.run(args, session, live.camera_shift(pkg / 'params/timing.json'))
        finally:
            fixture_stats['run_wall_seconds'] = time.monotonic() - started
            (session / 'fixture_stats.json').write_text(json.dumps(fixture_stats, indent=2) + '\n')
            for name in ('live.json', 'orb.log', 'camera.log'):
                path = session / name
                if path.exists():
                    print(name + ':\n' + path.read_text()[-6000:], flush=True)
    finally:
        observer_stop.set()
        if observer is not None:
            observer.join(timeout=3)
        server.terminate()
        server.wait(timeout=10)
        os.close(read_fd)
        xvfb_log.close()
        live.serial_port = original_serial_port

    assert live.read_imu is original_read_imu, 'Real read_imu must not be replaced'
    assert not fixture_stats['writer_errors'], fixture_stats
    assert not observer_errors, observer_errors
    assert {'ORB-SLAM3: Map Viewer', 'ORB-SLAM3: Current Frame'} <= seen, seen
    assert digest(pkg / 'lib/libORB_SLAM3.so') == core_hash
    subprocess.run(['sha256sum', '--quiet', '-c', 'SHA256SUMS'], cwd=pkg, check=True)
    report = json.loads((session / 'live.json').read_text())
    assert report['status'] == 'STREAM_COMPLETED', report
    assert report['orb_exit_code'] == 0 and report['performance_logging'] is True
    assert report['input_image_size'] == [1920, 1440]
    assert report['imu_clock_fit'] is not None
    live.check_clock_fit(report['imu_clock_fit'])
    data = {name: read_csv(session / (name + '.csv')) for name in
            ('camera_timing', 'imu_timing', 'camera_read', 'input_timing', 'tracking_timing')}
    assert all(data.values()), {k: len(v) for k, v in data.items()}
    camera_rows, imu_rows = data['camera_timing'], data['imu_timing']
    reads, inputs, tracking = data['camera_read'], data['input_timing'], data['tracking_timing']
    n = len(tracking)
    assert n >= 10, n
    assert n == len(inputs) == report['streams']['frames_sent'], (n, len(inputs), report)
    assert len(reads) == report['streams']['frames_read']
    assert len(camera_rows) >= n and len(reads) >= n
    assert all(len({row[key] for row in rows}) == len(rows)
               for rows, key in ((camera_rows, 'camera_raw_ns'), (reads, 'camera_raw_ns'),
                                 (inputs, 'camera_ns'), (tracking, 'camera_ns')))
    assert [r['read_index'] for r in imu_rows] == list(range(len(imu_rows)))
    assert all(r['samples'] > 0 for r in imu_rows)
    assert sum(r['samples'] for r in imu_rows) == report['streams']['imu_read']
    assert sum(r['samples'] for r in imu_rows) >= 5001, 'Full 10 s warmup must be logged'
    assert imu_rows[0]['receipt_raw_ns'] - fixture_stats['serial_open_raw_ns'] >= 500_000_000
    assert all(r['device_last_us'] - r['device_first_us'] == (r['samples'] - 1) * 2000
               for r in imu_rows)
    assert all(b['device_first_us'] - a['device_last_us'] == 2000
               for a, b in zip(imu_rows, imu_rows[1:]))
    assert all(b['receipt_raw_ns'] > a['receipt_raw_ns'] for a, b in zip(imu_rows, imu_rows[1:]))
    # No timing order is asserted between a concurrent pipe writer's return and reader's return.
    cb = {r['camera_raw_ns']: r for r in camera_rows}
    rb = {r['camera_raw_ns']: r for r in reads}
    ib = {r['camera_ns']: r for r in inputs}
    shift = report['camera_shift_ns']
    for index, row in enumerate(tracking):
        assert row['frame_idx'] == index
        raw = row['camera_ns'] - shift
        cam, read, inp = cb[raw], rb[raw], ib[row['camera_ns']]
        assert row['imu_count'] == inp['imu_count'] > 0
        assert (row['imu_first_ns'], row['imu_last_ns']) == (inp['imu_first_ns'], inp['imu_last_ns'])
        assert row['imu_first_ns'] <= row['imu_last_ns'] <= row['camera_ns']
        assert 0 <= row['camera_ns'] - row['imu_last_ns'] <= 3_000_000
        assert row['packet_read_end_raw_ns'] <= row['track_start_raw_ns'] < row['track_end_raw_ns']
        assert raw <= cam['dqbuf_raw_ns'] <= cam['gray_start_raw_ns'] <= cam['gray_done_raw_ns'] <= cam['write_start_raw_ns'] <= cam['write_end_raw_ns']
        assert cam['dqbuf_raw_ns'] <= read['read_done_raw_ns'] <= inp['selected_raw_ns'] <= inp['imu_ready_raw_ns'] <= inp['send_start_raw_ns'] <= inp['send_end_raw_ns']
        assert row['imu_initialized_proxy'] in (0, 1)
    assert sum(r['imu_count'] for r in tracking) == report['streams']['imu_sent']
    assert f'STOCK_STREAM_COMPLETE frames={n}' in (session / 'orb.log').read_text()
    # Exercise the packaged analyzer CLI and validate its actual report, not a reimplementation.
    subprocess.run([sys.executable, str(pkg / 'performance.py'), str(session)], check=True)
    measured = json.loads((session / 'performance.json').read_text())
    text_report = (session / 'performance.txt').read_text()
    assert measured['hardware_accuracy_validated'] is False
    assert measured['run_status'] == report['status'] and measured['run_requires_review'] is False
    assert measured['overall']['frame_count'] == n
    assert measured['overall']['matches']['all_three'] == n
    assert measured['overall']['matches']['imu_count_mismatches'] == 0
    assert all(meta['present'] and meta['invalid_rows'] == 0 and meta['valid_rows'] > 0
               for meta in measured['files'].values())
    assert all(value == 0 for value in measured['duplicate_match_keys'].values())
    assert math.isclose(measured['imu_capture']['source_density_hz'], 500, rel_tol=1e-12)
    assert measured['imu_capture']['active_tracking_host_window']['sample_count'] > 0
    assert measured['overall']['latency_ms']['track_call_ms']['count'] == n
    assert measured['overall']['latency_ms']['track_call_ms']['min'] > 0
    for field in ('latest_imu_to_track_start_ms', 'latest_imu_to_track_return_ms', 'imu_camera_tail_ms'):
        assert measured['overall']['latency_ms'][field]['observed_pairs'] == n
    ready_count = sum(r['tracking_state'] == 2 and r['imu_initialized_proxy'] == 1 for r in tracking)
    ready = measured['initialized_tracking']
    assert ready['frame_count'] == ready_count and ready['available'] is bool(ready_count)
    if not ready_count:
        assert ready['latency_ms']['track_call_ms']['count'] == 0
        assert ready['completed_call_throughput']['hz'] is None
        assert ready['latency_ms']['track_call_ms']['median'] is None
        assert '没有已初始化成功跟踪帧' in text_report
    summary = {
        'result': 'SIMULATED_SENSOR_BRIDGE_TIMING_SMOKE_OK',
        'artifact_directory': str(tmp), 'session': str(session),
        'csv_rows': {name: len(rows) for name, rows in data.items()},
        'native_x11_windows': sorted(seen), 'full_matches': n,
        'tracking_state_counts': measured['tracking_state_counts'],
        'initialized_tracking_rows': ready_count,
        'core_sha256': core_hash, 'hardware_performance_validated': False,
        'camera_csv_is_fixture': True, 'fixture': fixture_stats,
        'analyzer_status': measured['status'], 'analyzer_warnings': measured['warnings'],
    }
    (tmp / 'smoke_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
