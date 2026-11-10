#!/usr/bin/env python3
"""Inspect the 15 s simultaneous camera/IMU host-receipt trial. No time correction."""
import argparse
import bisect
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct
import zlib

NS = 1_000_000_000
FRAME_BYTES = 3840 * 1441
PIXELS = 1920 * 1440
CAM_FIELDS = 'frame_index sequence index flags driver_ts_us raw_before_ns raw_after_ns mono_after_ns bytesused metadata_offset'.split()
IMU_FIELDS = 'sample byte_offset timer_us_u32 clock_epoch timer_us_unwrapped_in_epoch host_packet_complete_raw_ns segment flags'.split()
AXES = 'ax_m_s2 ay_m_s2 az_m_s2 gx_rad_s gy_rad_s gz_rad_s'.split()
SENSORS = AXES + ['acc_norm_m_s2', 'gyro_norm_rad_s', 'temperature_c']
CLOCK_FIELDS = {
    'CNTVCT': ('raw_before_ns', 'counter_ns', 'raw_after_ns'),
    'MONOTONIC': ('mono_raw_before_ns', 'mono_ns', 'mono_raw_after_ns'),
    'REALTIME': ('realtime_raw_before_ns', 'realtime_ns', 'realtime_raw_after_ns')}


def bounded(path, limit):
    if path.is_symlink() or not path.is_file() or path.stat().st_size > limit:
        raise ValueError(f'{path.name}: missing, symlink, or larger than {limit} bytes')
    return path


def load_csv(path, fields, limit, max_rows):
    with bounded(path, limit).open(newline='', encoding='ascii') as f:
        reader = csv.DictReader(f)
        if not set(fields).issubset(reader.fieldnames or ()):
            raise ValueError(f'{path.name}: missing required fields')
        rows = []
        for row in reader:
            if len(rows) >= max_rows or None in row:
                raise ValueError(f'{path.name}: excess rows or malformed columns')
            rows.append(row)
        if not rows:
            raise ValueError(f'{path.name}: no rows')
        return rows


def distribution(values):
    if not values:
        return {'n': 0}
    values = sorted(values)
    def quantile(q):
        position = q*(len(values)-1)
        a, b = math.floor(position), math.ceil(position)
        return values[a] + (position-a)*(values[b]-values[a])
    return {'n': len(values), 'min': values[0], 'p05': quantile(.05), 'median': quantile(.5),
            'p95': quantile(.95), 'max': values[-1]}


def differences(values):
    return [b-a for a, b in zip(values, values[1:])]


def rate(values, unit):
    if len(values) < 2 or values[-1] <= values[0]:
        return 0.0
    return (len(values)-1)*unit/(values[-1]-values[0])


def read_camera(directory, fail):
    info = json.loads(bounded(directory/'capture_info.json', 64*1024).read_text())
    expected = {'device': '/dev/video6', 'driver': 'tegra-video', 'card': 'vi-output, pnd_gmsl_mc 10-003c',
                'bus': 'platform:tegra-capture-vi:2', 'pixel_format': 'UYVY', 'width': 1920, 'height': 1441,
                'bytesperline': 3840, 'sizeimage': FRAME_BYTES, 'active_first_row': 1, 'active_height': 1440,
                'metadata_record_bytes': 600, 'requested_seconds': 15}
    for key, value in expected.items():
        if info.get(key) != value:
            fail(f'camera capture_info {key}={info.get(key)!r}; expected {value!r}')
    rows = load_csv(directory/'camera_frames.csv', CAM_FIELDS+['sample'], 2_000_000, 1200)
    for row in rows:
        for key in CAM_FIELDS:
            row[key] = int(row[key])
    meta = bounded(directory/'camera_metadata.bin', 720000).read_bytes()
    valid = [r for r in rows if r['bytesused'] == FRAME_BYTES and not r['flags'] & 0x40]
    rejected = len(rows)-len(valid)
    if rejected:
        fail(f'camera rejected {rejected} frame(s): wrong payload length or V4L2 ERROR flag')
    if len(meta) != 600*len(valid):
        fail(f'camera_metadata.bin has {len(meta)} bytes for {len(valid)} valid frames')
    metadata_errors = collections.Counter()
    samples = []
    for n, row in enumerate(valid):
        start = n*600
        if row['metadata_offset'] != start:
            metadata_errors['offset'] += 1
        record = meta[start:start+600]
        if len(record) != 600:
            metadata_errors['short'] += 1
            continue
        if int.from_bytes(record[:2], 'little') != 588:
            metadata_errors['count'] += 1
        for kind, payload, checksum in [('header_crc', record[:4], record[4:8]),
                                        ('payload_crc', record[8:596], record[596:600])]:
            if zlib.crc32(payload) != int.from_bytes(checksum, 'little'):
                metadata_errors[kind] += 1
        if row['sample']:
            name = row['sample']
            if name != f"sample_{row['frame_index']:06d}.pgm" or not re.fullmatch(r'sample_[0-9]{6}\.pgm', name):
                raise ValueError(f'invalid PGM name at frame {row["frame_index"]}')
            if len(samples) >= 16:
                raise ValueError('more than 16 sample PGMs')
            pgm = bounded(directory/name, PIXELS+64).read_bytes()
            header = b'P5\n1920 1440\n255\n'
            if not pgm.startswith(header) or len(pgm) != len(header)+PIXELS:
                raise ValueError(f'{name}: PGM header, dimensions, or payload length incorrect')
            pixels = pgm[len(header):]
            low, high = min(pixels), max(pixels)
            if low == high:
                fail(f'{name}: uniform grayscale value {low}')
            samples.append({'name': name, 'dimensions': [1920,1440], 'pixels': len(pixels), 'min': low, 'max': high,
                            'pixel_sha256': hashlib.sha256(pixels).hexdigest()})
    if not samples:
        fail('camera saved no valid grayscale sample')
    if metadata_errors:
        fail('camera metadata checks failed: '+str(dict(metadata_errors)))
    if any(r['metadata_offset'] != -1 or r['sample'] for r in rows if r['bytesused'] != FRAME_BYTES or r['flags'] & 0x40):
        fail('a rejected camera row claims a metadata record or grayscale sample')
    if any(r['frame_index'] != i or not 0 <= r['index'] < info['mmap_buffer_count'] for i, r in enumerate(rows)):
        fail('camera CSV frame index order or MMAP buffer index invalid')
    seq_steps = [(b['sequence']-a['sequence']) % 2**32 for a, b in zip(rows, rows[1:])]
    seq_errors = sum(s != 1 for s in seq_steps)
    if seq_errors:
        fail(f'camera nonconsecutive sequence transitions: {seq_errors}')
    driver = [r['driver_ts_us'] for r in rows]
    receipts = [r['raw_after_ns'] for r in rows]
    if any(r['raw_after_ns'] < r['raw_before_ns'] or r['raw_before_ns'] <= 0 for r in rows):
        fail('camera DQBUF has reversed/invalid RAW brackets')
    if any(a['raw_after_ns'] > b['raw_before_ns'] for a,b in zip(rows,rows[1:])):
        fail('camera successive DQBUF RAW brackets overlap or run backward')
    if min(driver) <= 0 or any(x <= 0 for x in differences(driver)):
        fail('camera driver timestamps are zero, duplicated or out of order')
    if any(x <= 0 for x in differences(receipts)) or any(x <= 0 for x in differences([r['mono_after_ns'] for r in rows])):
        fail('camera host receipt timestamps are nonmonotonic')
    if not 25 <= rate(driver, 1_000_000) <= 35 or not 25 <= rate(receipts, NS) <= 35:
        fail('camera observed driver or receipt rate outside 25–35 Hz trial range')
    if any(x < 10000 or x > 70000 for x in differences(driver)):
        fail('camera driver frame interval outside 10–70 ms')
    if any(x > 250_000_000 for x in differences(receipts)):
        fail('camera host receipt gap exceeds 250 ms')
    clock_flags = collections.Counter(hex(r['flags'] & 0xe000) for r in rows)
    event_flags = collections.Counter(hex(r['flags'] & 0x70000) for r in rows)
    if len(clock_flags) != 1 or len(event_flags) != 1:
        fail('V4L2 timestamp type or source flag changes during trial')
    if any(r['flags'] & 0xe000 not in (0, 0x2000) for r in rows):
        fail('V4L2 timestamp type COPY or unrecognized during trial')
    result = {'rows': len(rows), 'valid_payloads': len(valid), 'rejected_payloads': rejected,
              'sequence_transition_errors': seq_errors, 'driver_hz': rate(driver, 1_000_000),
              'receipt_hz': rate(receipts, NS), 'driver_dt_ms': distribution([x/1000 for x in differences(driver)]),
              'receipt_dt_ms': distribution([x/1e6 for x in differences(receipts)]),
              'dqbuf_bracket_ms': distribution([(r['raw_after_ns']-r['raw_before_ns'])/1e6 for r in rows]),
              'v4l2_flags': dict(collections.Counter(hex(r['flags']) for r in rows)),
              'v4l2_timestamp_type_mask_0xe000': dict(clock_flags), 'v4l2_event_source_mask_0x70000': dict(event_flags),
              'metadata_records': len(meta)//600, 'metadata_errors': dict(metadata_errors), 'samples': samples}
    return result, valid, info


def read_clocks(directory, fail, notes):
    fields = [f for group in CLOCK_FIELDS.values() for f in group]+['cntfrq_hz']
    rows = load_csv(directory/'clock_samples.csv', fields, 500_000, 1000)
    for row in rows:
        for field in fields:
            row[field] = int(row[field])
    if len(rows) < 3:
        raise ValueError('fewer than 3 independent clock samples')
    frequencies = sorted({r['cntfrq_hz'] for r in rows})
    if len(frequencies) != 1 or min(frequencies) <= 0:
        fail(f'CNTFRQ invalid or unstable: {frequencies}')
    result = {'rows': len(rows), 'cntfrq_hz_values': frequencies, 'brackets': {}}
    anchors = {}
    for name, (before, field, after) in CLOCK_FIELDS.items():
        widths = [r[after]-r[before] for r in rows]
        domains = [r[field] for r in rows]
        mid = [(r[after]+r[before])//2 for r in rows]
        if min(widths) < 0 or max(widths) > 1_000_000 or min(domains) <= 0 or any(x <= 0 for x in differences(domains)) or any(x <= 0 for x in differences(mid)):
            message = f'{name} cross-clock samples invalid, not increasing, or RAW bracket wider than 1 ms'
            (notes.append if name == 'REALTIME' else fail)(message)
            result['brackets'][name] = {'excluded': message}
            continue
        offsets = [a-b for a,b in zip(mid, domains)]
        gross_range = max(offsets)-min(offsets)
        if gross_range > 2_000_000:
            notes.append(f'{name} sampled RAW-minus-domain offset range is {gross_range/1e6:.3f} ms; a smooth slew is represented by interpolation.')
        innovations = [offsets[i] - (offsets[i-1] + (offsets[i+1]-offsets[i-1]) *
                       (mid[i]-mid[i-1])/(mid[i+1]-mid[i-1])) for i in range(1,len(mid)-1)]
        discontinuous = max(abs(v) for v in innovations) > 2_000_000
        if discontinuous:
            notes.append(f'{name} has a >2 ms departure from interpolation between neighboring clock samples; exclude this candidate from plausible mappings.')
        result['brackets'][name] = {'raw_bracket_width_us': distribution([w/1000 for w in widths]),
                                   'raw_minus_domain_ns': distribution(offsets), 'offset_range_ms': gross_range/1e6,
                                   'neighbor_interpolation_residual_ms': distribution([v/1e6 for v in innovations]),
                                   'smooth_over_trial': not discontinuous,
                                   'raw_coverage_ns': [mid[0],mid[-1]], 'domain_coverage_ns': [domains[0],domains[-1]]}
        anchors[name] = (domains, offsets)
    return result, anchors


def mapped(t, anchor):
    domains, offsets = anchor
    pos = bisect.bisect_left(domains, t)
    if pos == 0 or pos == len(domains):
        i = 0 if pos == 0 else -1
        return t+offsets[i], abs(t-domains[i])
    a, b = pos-1, pos
    off = offsets[a]+(offsets[b]-offsets[a])*(t-domains[a])/(domains[b]-domains[a])
    return t+round(off), 0


def camera_clocks(rows, clocks, anchors, fail, notes):
    if not rows or 'MONOTONIC' not in anchors or 'CNTVCT' not in anchors:
        raise ValueError('no camera frames or missing usable MONOTONIC/CNTVCT brackets')
    valid = {}
    results = {}
    raw_range = clocks['brackets']['CNTVCT']['raw_coverage_ns']
    mono_anchor = anchors['MONOTONIC']
    host_checks = [(mapped(r['mono_after_ns'], mono_anchor)[0]-r['raw_after_ns'])/1e6 for r in rows]
    if max(abs(x) for x in host_checks) > 5:
        fail('camera MONOTONIC-after-DQ versus RAW-after-DQ cross-check differs by more than 5 ms')
    if rows[0]['raw_before_ns'] < raw_range[0]-1_000_000 or rows[-1]['raw_after_ns'] > raw_range[1]+1_000_000:
        fail('clock samples do not bracket the observed camera receipt interval')
    for name in ('RAW','MONOTONIC','CNTVCT'):
        lag = []
        outside = []
        mapped_times = []
        for r in rows:
            d = r['driver_ts_us']*1000
            if name == 'RAW':
                target = d
                distance = max(raw_range[0]-d, d-raw_range[1], 0)
            else:
                target, distance = mapped(d, anchors[name])
            mapped_times.append(target)
            outside.append(distance/1e6)
            lag.append((r['raw_after_ns']-target)/1e6)
        stat = distribution(lag)
        stable = name == 'RAW' or clocks['brackets'][name]['smooth_over_trial']
        plausible = (stable and max(outside) <= 250 and stat['min'] >= -5 and stat['p05'] >= -2 and stat['p95'] <= 250 and stat['max'] <= 500)
        results[name] = {'plausible_for_this_trial': plausible, 'sampled_clock_mapping_stable': stable, 'receipt_minus_mapped_driver_ms': stat,
                         'outside_sampled_domain_frames': sum(x > 0 for x in outside),
                         'maximum_outside_sampled_domain_ms': max(outside)}
        if plausible:
            valid[name] = mapped_times
    spread = max((max(v[i] for v in valid.values())-min(v[i] for v in valid.values()) for i in range(len(rows))), default=0)/1e6 if valid else None
    if not valid:
        fail('no RAW/MONOTONIC/CNTVCT timestamp hypothesis fits the observed receipts and sampled clock horizon')
    elif len(valid) > 1:
        notes.append('Several candidate camera clock mappings fit receipts; clock identity needs independent evidence.')
        if spread > 2:
            fail(f'plausible camera clock mappings differ by up to {spread:.3f} ms, above 2 ms trial agreement tolerance')
    if valid and 'MONOTONIC' not in valid and any(r['flags'] & 0xe000 == 0x2000 for r in rows):
        notes.append('V4L2 reports MONOTONIC but the sampled MONOTONIC candidate does not fit; check driver timestamp implementation.')
    return {'hypotheses': results, 'plausible_hypotheses': list(valid), 'maximum_plausible_mapping_spread_ms': spread,
            'mapped_host_mono_after_minus_raw_after_ms': distribution(host_checks),
            'method': 'Independent RAW brackets around each domain read. Piecewise-linear RAW-minus-domain offset within measured span, nearest endpoint offset outside. RAW candidate uses identity. No correction is applied to the original timestamps.'}


def imu_mapping(ticks, host):
    x = [(t-ticks[0])*1000 for t in ticks]
    y = [h-host[0] for h in host]
    span = x[-1]-x[0]
    if len(x) < 4 or span <= 0:
        raise ValueError('insufficient increasing IMU timer span for fit')
    groups = min(32, len(x))
    anchors = []
    for i in range(groups):
        a, b = i*len(x)//groups, (i+1)*len(x)//groups
        anchors.append((statistics.median(x[a:b]), statistics.median(y[a:b])))
    slopes = [(y2-y1)/(x2-x1) for i,(x1,y1) in enumerate(anchors) for x2,y2 in anchors[i+1:] if x2-x1 >= span/4]
    slope = statistics.median(slopes)
    intercept = statistics.median([v-slope*t for t,v in zip(x,y)])
    residual = [(v-(intercept+slope*t))/1e6 for t,v in zip(x,y)]
    return {'device_reference_us': ticks[0], 'host_reference_raw_ns': host[0],
            'host_relative_intercept_ns': intercept, 'host_ns_per_device_ns': slope,
            'apparent_rate_difference_ppm_over_trial': (slope-1)*1e6, 'receipt_residual_ms': distribution(residual),
            'method': 'Centered relative times. Median of secant slopes between 32 or fewer block medians separated by >=1/4 span, then median intercept. Residuals include serial/USB/read scheduling latency.'}


def read_imu(directory, fail, notes):
    summary = json.loads(bounded(directory/'imu_summary.json', 256_000).read_text())
    raw = bounded(directory/'imu_raw.bin', 4_000_000).read_bytes()
    chunks = load_csv(directory/'chunks.csv', ['end_byte_exclusive','host_raw_after_read_ns'], 9_000_000, 160_000)
    ends = [int(c['end_byte_exclusive']) for c in chunks]
    stamps = [int(c['host_raw_after_read_ns']) for c in chunks]
    if min(stamps) <= 0 or ends[0] <= 0 or ends[-1] != len(raw) or any(x <= 0 for x in differences(ends)) or any(x < 0 for x in differences(stamps)):
        raise ValueError('IMU chunk offsets/RAW timestamps are out of order or raw file has inconsistent length')
    rows = load_csv(directory/'imu_si.csv', IMU_FIELDS+SENSORS, 12_000_000, 18_000)
    for key, expected in [('bytes', len(raw)), ('crc_valid_ahrs_frames', len(rows)), ('finite_samples', len(rows)),
                          ('continuous_segments', 1), ('byte_discontinuities', 0), ('irregular_timer_intervals', 0),
                          ('reset_or_backward_events', 0), ('nonfinite_si_or_temperature', 0)]:
        if summary.get(key) != expected:
            fail(f'IMU summary {key}={summary.get(key)!r}; expected {expected}')
    if 'capture_completed' in summary and summary['capture_completed'] is not True:
        fail('IMU capture did not complete')
    if summary.get('imu_stream_ok') is False:
        fail('IMU capture worker checks failed: '+', '.join(k for k,v in summary.get('checks',{}).items() if not v))
    if summary.get('interior_crc_fail_candidates',0):
        fail(f"IMU interior CRC-failure candidates: {summary['interior_crc_fail_candidates']}")
    offsets = []
    ticks = []
    host = []
    values = {key: [] for key in SENSORS}
    bad = collections.Counter()
    segments = set()
    epochs = set()
    for n, row in enumerate(rows):
        off = int(row['byte_offset'])
        frame = raw[off:off+54] if off >= 0 else b''
        if len(frame)!=54 or frame[:6] != b'\xaa\x55\x02\x00\x2c\x00' or ((zlib.crc32(frame[:-4],0xfffffffe)^0xffffffff)&0xffffffff) != int.from_bytes(frame[-4:],'little'):
            bad['raw_frame_crc_or_layout'] += 1
            continue
        timer, *v = struct.unpack_from('<I10f', frame, 6)
        acc = [x*9.80665 for x in v[3:6]]
        gyro = [math.radians(x) for x in v[6:9]]
        expected = (*acc, *gyro, math.hypot(*acc), math.hypot(*gyro), v[9])
        tick = int(row['timer_us_unwrapped_in_epoch'])
        h = int(row['host_packet_complete_raw_ns'])
        j = bisect.bisect_left(ends, off+54)
        if j == len(ends) or stamps[j] != h:
            bad['packet_completion_chunk_stamp'] += 1
        if timer != int(row['timer_us_u32']) or tick < 0 or tick & 0xffffffff != timer:
            bad['timer_disagreement'] += 1
        if int(row['sample']) != n or row['flags'] not in ('regular','u32_wrap'):
            bad['sample_index_or_parser_flags'] += 1
        segments.add(row['segment']); epochs.add(int(row['clock_epoch']))
        offsets.append(off); ticks.append(tick); host.append(h)
        for key,want in zip(SENSORS,expected):
            actual = float(row[key])
            if not math.isfinite(actual) or not math.isclose(actual,want,rel_tol=1e-9,abs_tol=1e-10):
                bad['nonfinite_or_raw_si_mismatch'] += 1
            else:
                values[key].append(actual)
    if bad:
        fail('IMU raw/CSV/read-index checks failed: '+str(dict(bad)))
    if len(host)<2:
        raise ValueError('fewer than two independently CRC-valid IMU samples')
    if any(x != 54 for x in differences(offsets)) or len(segments)!=1 or '' in segments or len(epochs)!=1:
        fail('IMU packets are discontiguous or multiple segments/epochs present')
    dt = differences(ticks)
    hdt = differences(host)
    if any(x < 1500 or x > 2500 for x in dt):
        fail('IMU device timer increments outside 1500–2500 us or out of order')
    if any(x < 0 or x > 50_000_000 for x in hdt):
        fail('IMU packet receipt timestamps decrease or have >50 ms gap')
    dhz, hhz = rate(ticks,1e6), rate(host,NS)
    if not 480 <= dhz <= 520 or not 480 <= hhz <= 520:
        fail('IMU observed device or host rate outside 480–520 Hz trial range')
    mapping = None
    if len(epochs) == 1 and all(x > 0 for x in dt):
        mapping = imu_mapping(ticks,host)
        q = mapping['receipt_residual_ms']
        if abs(mapping['host_ns_per_device_ns']-1) > .01 or q['p95']-q['p05'] > 10 or max(abs(q['min']), abs(q['max'])) > 50:
            fail('IMU device-to-receipt diagnostic exceeds 1% rate, 10 ms central span, or 50 ms absolute residual tolerance')
    crc_candidates = summary.get('crc_fail_candidates')
    if crc_candidates:
        notes.append(f'IMU parser reported {crc_candidates} CRC-failed candidates. Inspect raw boundary bytes and internal byte gaps; an incomplete stream boundary is possible.')
    result = {'crc_verified_rows': len(host), 'read_chunks': len(chunks), 'raw_bytes': len(raw),
              'raw_sha256': hashlib.sha256(raw).hexdigest(), 'parser_crc_failed_candidates': crc_candidates,
              'parser_crc_failed_by_region': {k: summary.get(k+'_crc_fail_candidates') for k in ('prefix','interior','suffix')},
              'prefix_bytes': offsets[0], 'suffix_bytes': len(raw)-offsets[-1]-54, 'segments': sorted(segments),
              'epochs': sorted(epochs), 'device_hz': dhz, 'host_receipt_hz': hhz,
              'device_intervals_us': distribution(dt), 'host_receipt_intervals_ms': distribution([x/1e6 for x in hdt]),
              'equal_consecutive_host_stamp_count': hdt.count(0), 'raw_crosscheck_errors': dict(bad),
              'device_to_host_receipt_diagnostic': mapping, 'observed_si': {}}
    for key, vals in values.items():
        if vals:
            result['observed_si'][key] = {'n': len(vals), 'mean': statistics.fmean(vals), 'sample_std': statistics.stdev(vals) if len(vals)>1 else None,
                                         'min': min(vals), 'max': max(vals)}
    return result, host


def analyze(directory):
    failures, notes = [], []
    result = {'schema_version': 1, 'scope': '15-second simultaneous host receipt and timestamp diagnostic',
              'status': 'REVIEW_REQUIRED', 'failure_reasons': failures, 'notes': notes,
              'trial_gates': {'common_host_receipt_seconds_min': 12, 'camera_hz_range': [25,35], 'imu_hz_range':[480,520],
                              'camera_hypothesis_horizon_ms': 250, 'camera_hypothesis_receipt_lag_ms': {'min_floor':-5,'p05_floor':-2,'p95_ceiling':250,'max_ceiling':500},
                              'maximum_plausible_mapping_spread_ms': 2},
              'interpretation': 'These are engineering gates for proceeding to a dynamic timing trial. Neither physical synchronization, timestamp event semantics, camera–IMU delay nor calibration accuracy is established by still data. A 15-second rate fit does not measure long-term drift.'}
    rows, hosts, info, clocks, anchors = [], [], {}, None, {}
    errors = (OSError, ValueError, KeyError, TypeError, csv.Error, OverflowError, statistics.StatisticsError)
    try:
        result['camera'], rows, info = read_camera(directory, failures.append)
    except errors as exc:
        failures.append('camera: '+str(exc))
    try:
        clocks, anchors = read_clocks(directory, failures.append, notes)
        result['clock_samples'] = clocks
        if info and clocks['cntfrq_hz_values'] != [info.get('counter_frequency_hz')]:
            failures.append('capture_info counter frequency and clock sample frequencies differ')
        result['camera_clock_diagnostic'] = camera_clocks(rows, clocks, anchors, failures.append, notes)
    except errors as exc:
        failures.append('clock diagnostic: '+str(exc))
    try:
        result['imu'], hosts = read_imu(directory, failures.append, notes)
    except errors as exc:
        failures.append('IMU: '+str(exc))
    if rows and hosts:
        cam = [r['raw_after_ns'] for r in rows]
        start, end = max(cam[0],hosts[0]), min(cam[-1],hosts[-1])
        overlap = max(0,end-start)/NS
        result['host_receipt_overlap'] = {'camera_first_last_raw_ns':[cam[0],cam[-1]],'imu_first_last_raw_ns':[hosts[0],hosts[-1]],
                                          'seconds': overlap,'camera_rows_in_common_span': sum(start <= c <= end for c in cam),
                                          'imu_rows_in_common_span': sum(start <= h <= end for h in hosts)}
        if overlap < 12:
            failures.append(f'common host RAW receipt span {overlap:.6f} s is less than 12 s')
    else:
        failures.append('common host receipt span unavailable')
    if not failures:
        result['status'] = 'OK_FOR_DYNAMIC_TRIAL'
    return result


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('outdir',type=Path)
    args = cli.parse_args()
    directory = args.outdir.resolve(strict=True)
    if not directory.is_dir():
        cli.error('OUTDIR must be a directory')
    result = analyze(directory)
    target = directory/'timing.json'
    temporary = directory/'timing.json.tmp'
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(target)
    print(result['status'], target)
    for msg in result['failure_reasons']:
        print('Review:',msg)
    return int(result['status'] != 'OK_FOR_DYNAMIC_TRIAL')


if __name__ == '__main__':
    raise SystemExit(main())
