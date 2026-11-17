#!/usr/bin/env python3
"""Read-only live VIO status, independent of sensors, graphics and NumPy.

Read only bounded tails of changing CSV files. A saved map or a last valid row
cannot certify a currently running tracker: process identity and freshness are
checked separately. --watch stops this query only, never the VIO process.
"""
import argparse
import csv
from datetime import datetime
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import time

TAIL_BYTES = 512 * 1024
FRESH_SECONDS = 2.0


def clean(value):
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", str(value))
    return " ".join("".join(c for c in text if c.isprintable() or c.isspace()).split())[:700]


def json_file(path, limit=2 * 1024 * 1024):
    try:
        with Path(path).open("rb") as stream:
            data = stream.read(limit + 1)
        if len(data) > limit:
            return None
        value = json.loads(data)
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def text_tail(path, limit=65536):
    try:
        with Path(path).open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            start = max(0, size - limit)
            stream.seek(start)
            data = stream.read(limit)
        if start:
            data = data.partition(b"\n")[2]
        # Only completed records; writers may be in the middle of a line.
        data = data.rsplit(b"\n", 1)[0] if b"\n" in data else b""
        return data.decode("utf-8", errors="replace").splitlines()
    except OSError:
        return []


def text_head(path, limit=65536):
    try:
        with Path(path).open("rb") as stream:
            data = stream.read(limit)
        data = data.rsplit(b"\n", 1)[0] if b"\n" in data else b""
        return data.decode("utf-8", errors="replace").splitlines()
    except OSError:
        return []


def csv_tail(path, limit=TAIL_BYTES):
    """Return the completed tail, preserving malformed rows as barriers."""
    result = {"rows": [], "error": None, "truncated": False, "mtime_ns": None}
    try:
        with Path(path).open("rb") as stream:
            header_bytes = stream.readline(16384)
            if not header_bytes.endswith(b"\n"):
                result["error"] = "CSV header is missing or incomplete"
                return result
            header = next(csv.reader([header_bytes.decode("utf-8").rstrip("\r\n")], strict=True))
            if not header or len(header) != len(set(header)):
                result["error"] = "CSV header has missing/duplicate columns"
                return result
            first = stream.tell()
            stat = os.fstat(stream.fileno())
            result["mtime_ns"] = stat.st_mtime_ns
            start = max(first, stat.st_size - limit)
            stream.seek(start)
            data = stream.read(limit)
        if start > first:
            data = data.partition(b"\n")[2]
            result["truncated"] = True
        if b"\n" not in data:
            return result
        data = data.rsplit(b"\n", 1)[0]
        for line in data.decode("utf-8", errors="replace").splitlines():
            if not line:
                continue
            try:
                fields = next(csv.reader([line], strict=True))
                if len(fields) != len(header):
                    raise ValueError("wrong CSV column count")
                result["rows"].append(dict(zip(header, fields)))
            except (ValueError, csv.Error):
                result["rows"].append(None)
        return result
    except (OSError, UnicodeError, csv.Error, ValueError) as error:
        result["error"] = clean(error)
        return result


def integer(value):
    try:
        return int(value)
    except (ValueError, TypeError):
        return None


def finite(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError):
        return None


def read_identity_text(path):
    try:
        return Path(path).read_text().strip() or None
    except OSError:
        return None


def linux_start_ticks(pid):
    """The process name in stat can contain spaces and parentheses."""
    try:
        text = Path('/proc') / str(pid) / 'stat'
        fields = text.read_text().rsplit(')', 1)[1].split()
        return fields[19], fields[0]  # field 22 / process state
    except (OSError, ValueError, IndexError):
        return None, None


def process_state(runtime, final_report):
    if runtime is None:
        if final_report is not None:
            return {"state": "STOPPED", "reason": "已有本次最终 live.json；未提供监督进程记录", "identity_verified": False}
        return {"state": "UNKNOWN", "reason": "没有 runtime.json，不能仅凭日志确认进程存活", "identity_verified": False}
    lifecycle = runtime.get('state')
    if lifecycle == 'FINISHED':
        return {"state": "STOPPED", "reason": "监督进程已完成收尾", "identity_verified": False,
                "exit_status": runtime.get('exit_status')}
    boot = read_identity_text('/proc/sys/kernel/random/boot_id')
    if not boot or not runtime.get('boot_id') or boot != runtime.get('boot_id'):
        return {"state": "UNKNOWN", "reason": "运行记录不属于当前主机启动周期，或无法核对进程身份", "identity_verified": False}
    machine = read_identity_text('/etc/machine-id')
    if runtime.get('machine_id') and machine and runtime['machine_id'] != machine:
        return {"state": "UNKNOWN", "reason": "运行记录来自另一台主机", "identity_verified": False}
    pid = integer(runtime.get('supervisor_pid'))
    expected_ticks = runtime.get('supervisor_start_ticks')
    if pid is None or pid <= 0 or expected_ticks is None:
        return {"state": "UNKNOWN", "reason": "监督进程身份记录不完整", "identity_verified": False}
    actual_ticks, proc_state = linux_start_ticks(pid)
    if actual_ticks is None:
        # Distinguish a vanished process from /proc access restrictions.
        if not Path('/proc', str(pid)).exists():
            return {"state": "STOPPED", "reason": "监督进程已不存在（可能异常退出）", "identity_verified": False}
        return {"state": "UNKNOWN", "reason": "无法读取监督进程身份", "identity_verified": False}
    if str(expected_ticks) != actual_ticks or proc_state in ('Z', 'X'):
        return {"state": "STOPPED", "reason": "原监督进程已结束，PID 被复用或进程已退出", "identity_verified": False}
    if lifecycle not in ('STARTING', 'RUNNING', 'FINALIZING'):
        return {"state": "UNKNOWN", "reason": "未知监督进程状态", "identity_verified": True}
    return {"state": lifecycle, "reason": "监督进程 PID、启动时刻和系统启动标识一致", "identity_verified": True}


def resolve_target(root, requested=None):
    root = Path(root).expanduser().resolve()
    if requested:
        selected = Path(requested).expanduser().resolve()
        if not selected.is_dir():
            raise ValueError('VIO_E_PATH: Run/session directory does not exist: ' + str(selected))
        if selected.name == 'session' or any((selected / f).exists() for f in ('poses.csv', 'live.json', 'runtime.json')):
            return selected, None
        if (selected / 'session').is_dir() or (selected / 'settings.yaml').exists() or (selected / 'map_plan.json').exists():
            return selected / 'session', None
        raise ValueError('VIO_E_PATH: Expected a VIO run or session directory: ' + str(selected))
    runs = Path(os.environ.get('ORB_HUB_RUNS', str(root / 'runs'))).expanduser().resolve()
    candidates = []
    try:
        for path in runs.iterdir():
            if (path.is_dir() and path.name.startswith('live_')) or (path.is_file() and path.name.startswith('startup_') and path.suffix == '.log'):
                try:
                    candidates.append((path.stat().st_mtime_ns, path))
                except OSError:
                    pass
    except OSError:
        pass
    if not candidates:
        return None, None
    selected = max(candidates, key=lambda item: (item[0], str(item[1])))[1]
    return (None, selected) if selected.is_file() else (selected / 'session', None)


def map_load_status(session, plan, console_lines, orb_lines):
    combined = console_lines + orb_lines
    # Genuine native load marker takes priority over the earlier planned status.
    for line in reversed(combined):
        match = re.fullmatch(r'MAP_REFERENCE_LOADED id=(\d+) keyframes=(\d+); current velocity/bias still require recovery', line)
        echoed = re.match(r'Map load: SUCCESS id=(\d+) keyframes=(\d+);', line)
        if match or echoed:
            evidence = match or echoed
            return {"state": "LOADED", "map_id": int(evidence[1]), "keyframes": int(evidence[2]),
                    "path": plan.get('load') if plan else None,
                    "reason": "旧地图已实际加载；当前定位/惯性恢复状态需另看 ready"}
    for line in reversed(combined):
        match = re.search(r'\b(MAP_E_(?:LOAD|FORMAT|CHECKSUM|MANIFEST|CALIBRATION|HEALTH)):\s*(.+)', line)
        if match:
            return {"state": "FAILED", "code": match[1], "reason": clean(match[2])}
    if plan:
        if plan.get('mode') == 'off':
            return {"state": "DISABLED", "reason": "本次关闭旧地图加载"}
        if plan.get('load'):
            return {"state": "LOADING_OR_UNCONFIRMED", "path": plan['load'], "reason": "有加载计划，但有界日志中尚无成功确认"}
        return {"state": "NO_REFERENCE", "path": plan.get('reference'), "reason": "本次从新地图开始"}
    return {"state": "UNKNOWN", "reason": "缺少本次地图计划/加载记录"}


def pose_valid(row):
    if not row or row.get('phase') != 'TRACKING' or row.get('tracking_state') != '2':
        return False
    if any(row.get(key) != '1' for key in ('visual_ready', 'inertial_ready', 'valid')):
        return False
    if any(integer(row.get(key)) is None for key in ('frame_idx', 'camera_ns', 'map_id')):
        return False
    return all(finite(row.get(key)) is not None for key in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'))


def recent_valid_span(rows):
    if not rows or not pose_valid(rows[-1]):
        return 0.0, 0, False
    latest = rows[-1]
    newest_ns = previous_ns = int(latest['camera_ns'])
    previous_frame = int(latest['frame_idx'])
    count = 1
    for row in reversed(rows[:-1]):
        if not pose_valid(row) or row.get('map_id') != latest.get('map_id') or row.get('recovery_id') != latest.get('recovery_id'):
            break
        stamp, frame = int(row['camera_ns']), int(row['frame_idx'])
        if frame != previous_frame - 1 or not 0 < previous_ns - stamp <= 250_000_000:
            break
        previous_ns, previous_frame = stamp, frame
        count += 1
    return (newest_ns - previous_ns) / 1e9, count, count == len(rows)


def snapshot(root, requested=None, now_ns=None):
    now_ns = time.time_ns() if now_ns is None else now_ns
    session, startup = resolve_target(root, requested)
    result = {"schema": 1, "observed_unix_ns": now_ns, "session": str(session) if session else None,
              "ready": False, "stable_ready": False, "accuracy_verified": False}
    if session is None:
        lines = text_tail(startup) if startup else []
        result.update(program={"state": "UNKNOWN" if startup else "NO_SESSION", "identity_verified": False,
                               "reason": "最近一次启动尚未创建会话，可能仍在启动或已失败" if startup else "尚未找到运行记录"},
                      startup_log=str(startup) if startup else None,
                      startup_detail=clean(lines[-1]) if lines else None)
        return result
    runtime = json_file(session / 'runtime.json')
    report = json_file(session / 'live.json')
    plan = json_file(session.parent / 'map_plan.json')
    program = process_state(runtime, report)
    result['program'] = program
    result['runtime'] = runtime
    result['map_load'] = map_load_status(session, plan,
        text_head(session / 'console.log') + text_tail(session / 'console.log'),
        text_head(session / 'orb.log') + text_tail(session / 'orb.log'))
    result['map_save'] = json_file(session / 'map_save.json')
    result['map_publication'] = json_file(session / 'map_session.json')
    result['final_report'] = {key: report.get(key) for key in ('status', 'error', 'orb_exit_code')} if report else None
    poses = csv_tail(session / 'poses.csv')
    rows = poses['rows']
    last = rows[-1] if rows else None
    if last:
        needed = ('frame_idx', 'camera_ns', 'map_id', 'tracking_state', 'visual_ready', 'inertial_ready', 'valid', 'recovery_id', 'phase')
        valid_structure = all(key in last for key in needed) and all(integer(last.get(k)) is not None for k in ('frame_idx', 'camera_ns', 'map_id', 'tracking_state'))
        if not valid_structure:
            poses['error'] = 'Latest complete pose row is malformed'
            last = None
    elif rows:
        poses['error'] = 'Latest complete pose row is malformed'
    if len(rows) >= 2 and last and rows[-2]:
        previous_stamp = integer(rows[-2].get('camera_ns'))
        previous_frame = integer(rows[-2].get('frame_idx'))
        if previous_stamp is not None and previous_frame is not None and (
                int(last['camera_ns']) <= previous_stamp or int(last['frame_idx']) <= previous_frame):
            poses['error'] = 'Pose frame/timestamp is not increasing'
    mtime_age = (now_ns - poses['mtime_ns']) / 1e9 if poses['mtime_ns'] is not None else None
    stamp_age = None
    if program.get('identity_verified') and last and hasattr(time, 'CLOCK_MONOTONIC_RAW'):
        stamp_age = (time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) - int(last['camera_ns'])) / 1e9
    fresh = mtime_age is not None and -1 <= mtime_age <= FRESH_SECONDS
    if stamp_age is not None:
        fresh = fresh and -1 <= stamp_age <= FRESH_SECONDS
    result['data'] = {"file_age_s": mtime_age, "camera_stamp_age_s": stamp_age, "fresh": fresh,
                      "error": poses['error'], "tail_truncated": poses['truncated']}
    if program['state'] == 'RUNNING' and not fresh:
        program['state'] = 'STALE'
        program['reason'] = '监督进程仍存在，但位姿记录超过 2 秒未刷新/时间异常；不能视为当前就绪'
    span, count, reached_tail = recent_valid_span(rows) if not poses['error'] else (0.0, 0, False)
    result['tracking'] = {key: last.get(key) for key in ('frame_idx', 'camera_ns', 'map_id', 'tracking_state', 'visual_ready', 'inertial_ready', 'valid', 'recovery_id', 'phase')} if last else None
    if result['tracking']:
        result['tracking'].update(consecutive_valid_s=span, consecutive_valid_frames=count,
                                  span_is_lower_bound=bool(reached_tail and poses['truncated']))
    result['ready'] = bool(program['state'] == 'RUNNING' and fresh and not poses['error'] and pose_valid(last))
    result['stable_ready'] = bool(result['ready'] and span >= 2.0)
    for filename, key in [('state_diagnostics.csv', 'health'), ('recovery_timing.csv', 'recovery'), ('recovery_events.csv', 'recovery_event')]:
        tail = csv_tail(session / filename)
        row = tail['rows'][-1] if tail['rows'] else None
        if row:
            # Writer updates several files in order. Show one-frame-old diagnostic
            # rows honestly instead of falsely associating them with the pose.
            row = dict(row)
            row['matches_pose_frame'] = bool(last and row.get('frame_idx') == last.get('frame_idx') and row.get('camera_ns') == last.get('camera_ns'))
        result[key] = row
    return result


def render_text(status):
    program = status['program']
    lines = [datetime.fromtimestamp(status['observed_unix_ns'] / 1e9).astimezone().strftime('%Y-%m-%d %H:%M:%S'),
             '会话: ' + str(status.get('session') or status.get('startup_log') or '无'),
             '程序: ' + program['state'] + ' — ' + program.get('reason', '')]
    if status.get('startup_detail'):
        lines.append('启动详情: ' + status['startup_detail'])
    loaded = status.get('map_load')
    if loaded:
        lines.append('旧地图: ' + loaded['state'] + ' — ' + loaded.get('reason', ''))
        if loaded.get('path'):
            lines.append('参考路径: ' + clean(loaded['path']))
        if loaded.get('map_id') is not None:
            lines.append(f"已加载地图: id={loaded['map_id']} keyframes={loaded.get('keyframes', '?')}")
    track = status.get('tracking')
    if track:
        lines.append('跟踪: {phase} | frame={frame_idx} map={map_id} visual={visual_ready} imu={inertial_ready} valid={valid}'.format(**track))
        lower = '至少 ' if track['span_is_lower_bound'] else ''
        lines.append('连续有效: ' + lower + f"{track['consecutive_valid_s']:.2f} 秒 / {track['consecutive_valid_frames']} 帧")
    data = status.get('data') or {}
    if data.get('file_age_s') is not None:
        lines.append(f"位姿文件更新时间距今: {data['file_age_s']:.2f} 秒")
    if data.get('error'):
        lines.append('数据检查: ' + data['error'])
    lines.append('当前可用: ' + ('YES' if status['ready'] else 'NO') + ' | 连续有效 ≥2 秒: ' + ('YES' if status['stable_ready'] else 'NO'))
    recovery = status.get('recovery')
    if recovery:
        lines.append('恢复: method={0} search_ms={1} global={2} window_resets={3}{4}'.format(
            recovery.get('recovery_method', '?'), recovery.get('recovery_search_ms', '?'),
            recovery.get('recovery_global_attempted', '?'), recovery.get('recovery_window_resets', '?'),
            '' if recovery['matches_pose_frame'] else '（与最新位姿帧未对齐）'))
    health = status.get('health')
    if health:
        lines.append('状态检查: ' + clean(health.get('health_reason', '未知')) +
                     ('' if health['matches_pose_frame'] else '（与最新位姿帧未对齐）'))
    event = status.get('recovery_event')
    if event and event.get('reason'):
        lines.append('最近阶段原因: ' + clean(event['reason']))
    saved, published = status.get('map_save'), status.get('map_publication')
    lines.append('地图保存: ' + str(saved.get('status') if saved else '尚无结束报告') +
                 ' | 参考图发布: ' + str(published.get('status') if published else '尚无结束报告'))
    final = status.get('final_report')
    if final and final.get('error'):
        lines.append('结束原因: ' + clean(final['error']))
    lines.append('可用/连续有效是运行状态检查，不是轨迹精度验收。')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only VIO status; no sensor, graphics or NumPy access.')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('session', nargs='?', help='Run directory or its session directory; omit for latest attempt')
    parser.add_argument('--watch', action='store_true', help='Refresh until Ctrl+C; VIO itself is not stopped')
    parser.add_argument('--json', action='store_true', help='JSON snapshot; --watch emits one JSON object per line')
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.interval) or args.interval < 0.2:
        parser.error('--interval must be finite and >=0.2 seconds')
    try:
        while True:
            value = snapshot(args.root, args.session)
            print(json.dumps(value, ensure_ascii=False, allow_nan=False) if args.json else render_text(value), flush=True)
            if not args.watch:
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError) as error:
        print(clean(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
