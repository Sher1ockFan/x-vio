#!/usr/bin/env python3
"""Quiet, stdlib-only measurement report for a mono_imu_vio live session.

CLI: python3 performance.py SESSION
API: write_report(session_dir) -> dict; analyze(session_dir) -> dict
No image files are read. This reports observations, not a calibration/accuracy PASS.
"""
import csv
import json
import math
import sys
from pathlib import Path

SCHEMAS = {
    "camera_timing.csv": "sequence driver_timestamp_ns camera_raw_ns dqbuf_raw_ns gray_start_raw_ns gray_done_raw_ns write_start_raw_ns write_end_raw_ns bytesused flags".split(),
    "imu_timing.csv": "read_index samples device_first_us device_last_us receipt_raw_ns".split(),
    "camera_read.csv": "camera_raw_ns read_done_raw_ns".split(),
    "input_timing.csv": "camera_ns selected_raw_ns imu_ready_raw_ns send_start_raw_ns send_end_raw_ns imu_count imu_first_ns imu_last_ns frames_superseded".split(),
    "tracking_timing.csv": "frame_idx camera_ns imu_count imu_first_ns imu_last_ns packet_read_end_raw_ns track_start_raw_ns track_end_raw_ns tracking_state imu_initialized_proxy".split(),
}
LABELS = {
    "driver_event_to_dq_ms": "驱动时间戳事件→DQBUF 返回（映射到 RAW）",
    "gray_extract_ms": "灰度提取",
    "camera_pipe_write_ms": "相机进程写管道（含阻塞）",
    "camera_event_to_python_read_ms": "驱动时间戳事件→Python 完整图像读取",
    "camera_dq_to_python_read_ms": "DQBUF 返回→Python 完整图像读取",
    "python_read_to_selection_ms": "Python 完整读取→选帧",
    "wait_imu_ms": "选帧→IMU 配齐",
    "bridge_write_ms": "桥接进程写完整算法数据包（含阻塞）",
    "track_call_ms": "TrackMonocular 调用",
    "camera_event_to_track_return_ms": "未加偏移的相机时间→Track 返回",
    "calibrated_camera_to_track_return_ms": "加配置偏移后的相机时间→Track 返回",
    "packet_read_to_track_return_ms": "算法读完整数据包→Track 返回",
    "latest_imu_to_track_start_ms": "最新 IMU 样本映射时间→Track 入口",
    "latest_imu_to_track_return_ms": "最新 IMU 样本映射时间→Track 返回",
    "imu_camera_tail_ms": "相机校正时间减本批最后 IMU 样本时间",
}
NOTES = [
    "所有主机耗时使用 CLOCK_MONOTONIC_RAW。driver_timestamp_ns 属于驱动时基，须先映射为 camera_raw_ns，不能直接和 RAW 相减。",
    "相机源时间频率按采集的驱动时间戳计算；实际入算法频率按 tracking_timing 中完整接包且完成调用的记录计算。未完成的最后一次调用可能没有记录。",
    "源时间密度和主机墙钟吞吐不同：积压数据可以成批交付；IMU 每批送入 TrackMonocular，不代表每个样本独立输出位姿。",
    "imu_initialized_proxy 仅在 tracking_state==2 且当前帧有效地图点属于同一张已标记 IMU 初始化的地图时为真；无地图点、地图混合或失跟时为假。这是调用返回后通过公开接口读取的保守代理，不能当作精确初始化完成时刻；筛选后的帧也不保证绝对精度。",
    "相机—IMU 配置偏移只用于时间对齐，不等于图像处理耗时，也不证明物理传感器延迟固定。启动 IMU 时钟拟合使用主机收包时间，含串口、USB 和调度延迟，不是同步标定真值。",
    "驱动日志曾报告 ts-src-eof；EOF 与真实曝光起点/中点的关系未经验证。因此事件到返回耗时不是已验证的曝光到位姿物理延迟。",
    "TrackMonocular 返回之后，后台建图、回环优化和窗口绘制仍可继续。这里未测显示刷新/屏幕呈现延迟，也未测端到端控制延迟。",
    "管道写结束和读结束可能因并行调度先后倒置；本报告不将二者相减当作传输耗时。",
    "最新 IMU 样本年龄使用映射到主机的 IMU 时间，含时钟拟合误差，不等于物理传感器或串口延迟；相机减末个 IMU 时间用于检查批次覆盖。",
    "imu_capture 全部记录可能含启动约 10 秒的时钟拟合/预热；另给 active_tracking_host_window，只选首个算法接包到最后调用返回之间收到的 IMU 批次。边界按整批收包时间筛选，不对批内样本虚构主机到达时间。",
    "本报告不含外部真值，不能判定轨迹的毫米/角度精度。负时间样本单独报告，并从非负耗时统计中排除。",
]


def _integer(value):
    try:
        return int(value)
    except (ValueError, TypeError):
        return int(value, 0)


def _read_csv(path, fields, warnings):
    meta = {"present": path.is_file(), "data_rows": 0, "valid_rows": 0,
            "invalid_rows": 0, "invalid_row_examples": [], "missing_columns": []}
    rows = []
    if not meta["present"]:
        warnings.append(f"缺失文件：{path.name}")
        return rows, meta
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            meta["missing_columns"] = sorted(set(fields) - set(reader.fieldnames or []))
            if meta["missing_columns"]:
                warnings.append(f"{path.name} 缺少列：{', '.join(meta['missing_columns'])}")
                return rows, meta
            for line, item in enumerate(reader, 2):
                meta["data_rows"] += 1
                try:
                    if None in item:
                        raise ValueError("列数超出表头")
                    row = {key: _integer(item[key]) for key in fields}
                    rows.append(row)
                except (ValueError, TypeError, KeyError) as exc:
                    meta["invalid_rows"] += 1
                    if len(meta["invalid_row_examples"]) < 5:
                        meta["invalid_row_examples"].append({"line": line, "error": str(exc)})
    except (OSError, UnicodeError, csv.Error) as exc:
        meta["read_error"] = str(exc)
        warnings.append(f"{path.name} 读取不完整：{exc}")
    meta["valid_rows"] = len(rows)
    if meta["invalid_rows"]:
        warnings.append(f"{path.name} 有 {meta['invalid_rows']} 行无效，已排除；可能包括未写完的末行。")
    if not rows:
        warnings.append(f"{path.name} 没有可用数据行。")
    return rows, meta


def _quantile(values, q):
    if not values:
        return None
    data = sorted(values)
    pos = (len(data) - 1) * q
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    return data[lo] + (data[hi] - data[lo]) * (pos - lo)


def _stats(values):
    data = [x for x in values if x is not None and math.isfinite(x)]
    return {"count": len(data), "min": min(data) if data else None,
            "median": _quantile(data, .5), "p95": _quantile(data, .95),
            "max": max(data) if data else None}


def _latency(values, warnings, name):
    finite = [x for x in values if math.isfinite(x)]
    negatives = [x for x in finite if x < 0]
    result = _stats([x for x in finite if x >= 0])
    result.update({"unit": "ms", "observed_pairs": len(finite),
                   "negative_count": len(negatives),
                   "negative_min_ms": min(negatives) if negatives else None})
    if negatives:
        warnings.append(f"{name} 出现 {len(negatives)} 个负耗时，最小 {min(negatives):.6f} ms；请核对时钟、配置偏移和记录顺序。")
    return result


def _rate(stamps, scale=1e9):
    """Event rate; no sorting hides timestamp resets or duplicate stamps."""
    result = {"count": len(stamps), "first": stamps[0] if stamps else None,
              "last": stamps[-1] if stamps else None, "span_s": None,
              "hz": None, "nonincreasing_intervals": 0}
    if len(stamps) < 2:
        return result
    diffs = [b-a for a, b in zip(stamps, stamps[1:])]
    result["nonincreasing_intervals"] = sum(d <= 0 for d in diffs)
    span = (stamps[-1]-stamps[0])/scale
    result["span_s"] = span
    if span > 0 and not result["nonincreasing_intervals"]:
        result["hz"] = (len(stamps)-1)/span
    result["positive_interval_ms"] = _stats([d*1000/scale for d in diffs if d > 0])
    return result


def _index(rows, key, label, warnings):
    indexed = {}
    duplicates = 0
    for row in rows:
        if row[key] in indexed:
            duplicates += 1
            continue
        indexed[row[key]] = row
    if duplicates:
        warnings.append(f"{label} 有 {duplicates} 个重复 {key}；关联时仅使用首次出现的行。")
    return indexed, duplicates


def _imu_rates(rows, count_key, first_key, last_key, host_key, scale):
    positive = [r for r in rows if r[count_key] > 0]
    valid = [r for r in positive if r[last_key] >= r[first_key]]
    total = sum(r[count_key] for r in valid)
    result = {"sample_count": total, "batches": len(valid),
              "zero_sample_batches": sum(r[count_key] == 0 for r in rows),
              "invalid_batches": len(positive)-len(valid)+sum(r[count_key] < 0 for r in rows),
              "batch_samples": _stats([r[count_key] for r in valid]),
              "source_first": None, "source_last": None, "source_span_s": None,
              "source_density_hz": None, "source_overlap_or_reset_boundaries": 0,
              "host_batch_arrivals": _rate([r[host_key] for r in valid]),
              "host_sample_throughput_hz": None,
              "host_throughput_samples_excluding_first_batch": 0}
    if not valid:
        return result
    first, last = valid[0][first_key], valid[-1][last_key]
    span = (last-first)/scale
    overlap = sum(b[first_key] <= a[last_key] for a, b in zip(valid, valid[1:]))
    result.update({"source_first": first, "source_last": last, "source_span_s": span,
                   "source_overlap_or_reset_boundaries": overlap})
    if total > 1 and span > 0 and not overlap:
        result["source_density_hz"] = (total-1)/span
    host = result["host_batch_arrivals"]
    samples = sum(r[count_key] for r in valid[1:])
    result["host_throughput_samples_excluding_first_batch"] = samples
    if host["span_s"] and host["span_s"] > 0 and not host["nonincreasing_intervals"]:
        result["host_sample_throughput_hz"] = samples/host["span_s"]
    return result


def _subset(rows, camera_by_raw, read_by_raw, input_by_camera, shift, warnings, label):
    values = {key: [] for key in LABELS}
    matched = {"tracking_rows": len(rows), "camera_rows": 0, "camera_read_rows": 0,
               "input_rows": 0, "all_three": 0, "imu_count_mismatches": 0}
    for row in rows:
        cam_ns = row["camera_ns"]
        raw_ns = cam_ns - shift if shift is not None else None
        camera = camera_by_raw.get(raw_ns)
        read = read_by_raw.get(raw_ns)
        inp = input_by_camera.get(cam_ns)
        matched["camera_rows"] += int(camera is not None)
        matched["camera_read_rows"] += int(read is not None)
        matched["input_rows"] += int(inp is not None)
        matched["all_three"] += int(camera is not None and read is not None and inp is not None)
        def add(key, end, start):
            values[key].append((end-start)/1e6)
        add("track_call_ms", row["track_end_raw_ns"], row["track_start_raw_ns"])
        add("calibrated_camera_to_track_return_ms", row["track_end_raw_ns"], cam_ns)
        add("packet_read_to_track_return_ms", row["track_end_raw_ns"], row["packet_read_end_raw_ns"])
        if row["imu_count"] > 0 and row["imu_last_ns"] >= row["imu_first_ns"]:
            add("latest_imu_to_track_start_ms", row["track_start_raw_ns"], row["imu_last_ns"])
            add("latest_imu_to_track_return_ms", row["track_end_raw_ns"], row["imu_last_ns"])
            add("imu_camera_tail_ms", cam_ns, row["imu_last_ns"])
        if raw_ns is not None:
            add("camera_event_to_track_return_ms", row["track_end_raw_ns"], raw_ns)
        if camera:
            add("driver_event_to_dq_ms", camera["dqbuf_raw_ns"], camera["camera_raw_ns"])
            add("gray_extract_ms", camera["gray_done_raw_ns"], camera["gray_start_raw_ns"])
            add("camera_pipe_write_ms", camera["write_end_raw_ns"], camera["write_start_raw_ns"])
        if read and raw_ns is not None:
            add("camera_event_to_python_read_ms", read["read_done_raw_ns"], raw_ns)
            if camera:
                add("camera_dq_to_python_read_ms", read["read_done_raw_ns"], camera["dqbuf_raw_ns"])
        if inp:
            if inp["imu_count"] != row["imu_count"]:
                matched["imu_count_mismatches"] += 1
            add("wait_imu_ms", inp["imu_ready_raw_ns"], inp["selected_raw_ns"])
            add("bridge_write_ms", inp["send_end_raw_ns"], inp["send_start_raw_ns"])
            if read:
                add("python_read_to_selection_ms", inp["selected_raw_ns"], read["read_done_raw_ns"])
    for name in ("camera_rows", "camera_read_rows", "input_rows", "all_three"):
        matched["unmatched_"+name] = len(rows)-matched[name]
    return {
        "frame_count": len(rows), "matches": matched,
        "camera_source_density": _rate([r["camera_ns"] for r in rows]),
        "packet_receive_throughput": _rate([r["packet_read_end_raw_ns"] for r in rows]),
        "completed_call_throughput": _rate([r["track_end_raw_ns"] for r in rows]),
        "imu_passed_to_algorithm": _imu_rates(rows, "imu_count", "imu_first_ns", "imu_last_ns", "packet_read_end_raw_ns", 1e9),
        "latency_ms": {key: _latency(data, warnings, label+" / "+LABELS[key]) for key, data in values.items()},
    }


def analyze(session_dir):
    session = Path(session_dir)
    warnings = []
    data, files = {}, {}
    for filename, schema in SCHEMAS.items():
        data[filename], files[filename] = _read_csv(session/filename, schema, warnings)
    live = {}
    try:
        live = json.loads((session/"live.json").read_text(encoding="utf-8"))
        if not isinstance(live, dict):
            raise ValueError("根节点不是对象")
    except (OSError, ValueError) as exc:
        live = {}
        warnings.append(f"live.json 不可用：{exc}；配置偏移相关关联可能不可用。")
    shift = None
    try:
        value = live.get("camera_shift_ns")
        if value is not None:
            if isinstance(value, bool) or int(value) != float(value):
                raise ValueError("必须为整纳秒")
            shift = int(value)
    except (ValueError, TypeError, OverflowError) as exc:
        warnings.append(f"camera_shift_ns 无效：{exc}")
    if shift is None:
        warnings.append("缺少 camera_shift_ns：不猜测偏移，不关联校正前后相机时间；原始相机到算法的关联耗时不可用。")
    camera, imu = data["camera_timing.csv"], data["imu_timing.csv"]
    reads, inputs, tracking = data["camera_read.csv"], data["input_timing.csv"], data["tracking_timing.csv"]
    cb, cd = _index(camera, "camera_raw_ns", "camera_timing.csv", warnings)
    rb, rd = _index(reads, "camera_raw_ns", "camera_read.csv", warnings)
    ib, idup = _index(inputs, "camera_ns", "input_timing.csv", warnings)
    _, td = _index(tracking, "camera_ns", "tracking_timing.csv", warnings)
    stable = [row for row in tracking if row["tracking_state"] == 2 and row["imu_initialized_proxy"] == 1]
    overall = _subset(tracking, cb, rb, ib, shift, warnings, "全时段已完成调用")
    ready = _subset(stable, cb, rb, ib, shift, warnings, "成功且 IMU 就绪代理为真")
    ready["available"] = bool(stable)
    ready["fraction_of_all_completed_calls"] = len(stable)/len(tracking) if tracking else None
    ready["all_completed_calls_denominator"] = len(tracking)
    whole_span = overall["completed_call_throughput"]["span_s"]
    eligible = stable if not tracking else [r for r in stable if r is not tracking[0]]
    ready["successful_returns_per_full_observation_second"] = len(eligible)/whole_span if whole_span and whole_span > 0 else None
    ready["full_observation_span_s"] = whole_span
    ready["successful_return_count_excluding_first_global_call"] = len(eligible)
    ready["scope_note"] = "子集时间密度按其首末帧计算，可能跨越中间失跟间隔，不是连续稳定片段。成功占比仍用全时段完成调用数作分母。"
    ready["imu_passed_to_algorithm"]["density_scope_note"] = "仅计筛选帧携带的 IMU 样本，除以这些批次首末样本的整个时间跨度；若筛选帧不连续，中间被排除样本不计入分子。因此此数是子集覆盖密度，不是 IMU 采样时钟频率。"
    if not stable:
        warnings.append("没有 tracking_state=2 且 imu_initialized_proxy=1 的帧；初始化后成功跟踪的频率和耗时不可评估。")
    states = {}
    for row in tracking:
        key = str(row["tracking_state"])
        states[key] = states.get(key, 0)+1
    raw_camera = {
        "driver_timestamp_events": _rate([r["driver_timestamp_ns"] for r in camera]),
        "mapped_raw_timestamp_events": _rate([r["camera_raw_ns"] for r in camera]),
        "dqbuf_host_receipts": _rate([r["dqbuf_raw_ns"] for r in camera]),
        "python_complete_image_receipts": _rate([r["read_done_raw_ns"] for r in reads]),
        "sequence_missing_positive": sum(max(b["sequence"]-a["sequence"]-1, 0) for a,b in zip(camera,camera[1:])),
        "sequence_nonincreasing": sum(b["sequence"] <= a["sequence"] for a,b in zip(camera,camera[1:])),
        "all_captured_latency_ms": {
            "driver_event_to_dq_ms": _latency([(r["dqbuf_raw_ns"]-r["camera_raw_ns"])/1e6 for r in camera], warnings, "全部相机采集 / 驱动事件→DQBUF"),
            "gray_extract_ms": _latency([(r["gray_done_raw_ns"]-r["gray_start_raw_ns"])/1e6 for r in camera], warnings, "全部相机采集 / 灰度提取"),
            "camera_pipe_write_ms": _latency([(r["write_end_raw_ns"]-r["write_start_raw_ns"])/1e6 for r in camera], warnings, "全部相机采集 / 写管道"),
        },
    }
    raw_imu = _imu_rates(imu, "samples", "device_first_us", "device_last_us", "receipt_raw_ns", 1e6)
    raw_imu["scope"] = "All logged IMU reads, including the startup clock-fit warmup when present; not restricted to active tracking."
    active_start = tracking[0]["packet_read_end_raw_ns"] if tracking else None
    active_end = tracking[-1]["track_end_raw_ns"] if tracking else None
    active_imu_rows = [r for r in imu if active_start <= r["receipt_raw_ns"] <= active_end] if tracking and active_end >= active_start else []
    active_imu = _imu_rates(active_imu_rows, "samples", "device_first_us", "device_last_us", "receipt_raw_ns", 1e6)
    active_imu["requested_host_window_raw_ns"] = {"start": active_start, "end": active_end}
    active_imu["scope"] = "Whole receive blocks whose host receipt is within the first complete ORB packet receipt and final TrackMonocular return; sample throughput uses actual first/last selected receive events."
    raw_imu["active_tracking_host_window"] = active_imu
    run_status = live.get("status") or "UNKNOWN"
    run_error = live.get("error")
    run_review = bool(run_error) or any(word in str(run_status).upper() for word in ("INCOMPLETE", "REVIEW", "ERROR", "FAIL"))
    if run_review:
        warnings.append(f"本次运行需要复核：status={run_status}；error={run_error or '无具体错误文本'}。实测指标可供诊断，不表示运行成功。")
    report = {
        "schema_version": 1, "status": "MEASURED" if tracking else "NO_COMPLETED_TRACKING_DATA",
        "run_status": run_status, "run_error": run_error, "run_requires_review": run_review,
        "session": str(session.resolve()), "hardware_accuracy_validated": False,
        "clock": "CLOCK_MONOTONIC_RAW for host and mapped sensor timestamps; raw IMU device timestamps are microseconds",
        "camera_shift_ns": shift, "camera_shift_ms": shift/1e6 if shift is not None else None,
        "files": files,
        "duplicate_match_keys": {"camera": cd, "camera_read": rd, "input": idup, "tracking": td},
        "camera_capture": raw_camera, "imu_capture": raw_imu,
        "overall": overall, "initialized_tracking": ready, "tracking_state_counts": states,
        "live_context": {key: live.get(key) for key in ("status", "error", "streams", "imu_clock_fit", "last_imu_clock_update")},
        "notes": NOTES, "warnings": warnings,
    }
    # Explicitly flag discontinuities; frequency estimates do not hide resets.
    for label, group in (("相机驱动", raw_camera["driver_timestamp_events"]),
                         ("相机 DQBUF", raw_camera["dqbuf_host_receipts"]),
                         ("算法输入图像", overall["camera_source_density"]),
                         ("算法接包", overall["packet_receive_throughput"]),
                         ("算法调用完成", overall["completed_call_throughput"])):
        if group["nonincreasing_intervals"]:
            warnings.append(f"{label} 时间戳有 {group['nonincreasing_intervals']} 个非递增间隔；对应频率不可可靠计算。")
    for label, group in (("采集 IMU", raw_imu), ("入算法 IMU", overall["imu_passed_to_algorithm"])):
        if group["source_overlap_or_reset_boundaries"] or group["invalid_batches"]:
            warnings.append(f"{label} 存在样本区间重叠/重置或无效批次，须复核时间序列。")
    match = overall["matches"]
    if tracking and match["all_three"] != len(tracking):
        warnings.append(f"完整跨进程关联 {match['all_three']}/{len(tracking)} 帧；缺少记录的耗时只按可匹配帧统计。")
    if match["imu_count_mismatches"]:
        warnings.append(f"桥接发送与算法实际接收的 IMU 数量有 {match['imu_count_mismatches']} 帧不一致。")
    if warnings and report["status"] == "MEASURED":
        report["status"] = "MEASURED_WITH_WARNINGS"
    report["measurement_status"] = report["status"]
    return report


def _fmt(value, digits=3):
    return "不可用" if value is None else f"{value:.{digits}f}"


def render_text(report):
    camera, imu = report["camera_capture"], report["imu_capture"]
    overall, ready = report["overall"], report["initialized_tracking"]
    passed = overall["imu_passed_to_algorithm"]
    active_imu = imu["active_tracking_host_window"]
    lines = ["VIO 输入频率与耗时实测报告", "", f"目录：{report['session']}",
             f"测量状态：{report['measurement_status']}（不是运行成功或精度验收结果）",
             f"实际运行状态：{report['run_status']}；{'需要复核' if report['run_requires_review'] else '以原始运行日志为准'}",
             f"实际运行错误：{report['run_error'] or '未报告'}",
             f"配置相机偏移：{_fmt(report['camera_shift_ms'], 6)} ms", "",
             "采集与实际入算法频率", 
             f"相机驱动源时间频率：{_fmt(camera['driver_timestamp_events']['hz'])} Hz；采集帧数 {camera['driver_timestamp_events']['count']}",
             f"相机 DQBUF 主机接收率：{_fmt(camera['dqbuf_host_receipts']['hz'])} 帧/s",
             f"Python 完整图像接收率：{_fmt(camera['python_complete_image_receipts']['hz'])} 帧/s",
             f"IMU 全记录源时间样本密度：{_fmt(imu['source_density_hz'])} Hz；样本 {imu['sample_count']}（含启动时钟拟合/预热）",
             f"IMU 全记录主机收包样本吞吐：{_fmt(imu['host_sample_throughput_hz'])} 样本/s（排除首批边界）",
             f"跟踪活动主机区间内 IMU：源密度 {_fmt(active_imu['source_density_hz'])} Hz，接收吞吐 {_fmt(active_imu['host_sample_throughput_hz'])} 样本/s，样本 {active_imu['sample_count']}",
             f"上述活动窗口 RAW：{active_imu['requested_host_window_raw_ns']['start']} → {active_imu['requested_host_window_raw_ns']['end']} ns（按整批收包时间筛选）",
             f"ORB 实际接收图像源时间密度：{_fmt(overall['camera_source_density']['hz'])} Hz",
             f"ORB 完整接包主机吞吐：{_fmt(overall['packet_receive_throughput']['hz'])} 帧/s",
             f"TrackMonocular 完成调用频率：{_fmt(overall['completed_call_throughput']['hz'])} 次/s；完成 {overall['frame_count']} 次",
             f"入 ORB 的 IMU 源时间样本密度：{_fmt(passed['source_density_hz'])} Hz；样本 {passed['sample_count']}",
             f"入 ORB 的 IMU 主机吞吐：{_fmt(passed['host_sample_throughput_hz'])} 样本/s（按接包时刻、排除首批边界）",
             f"每次输入 IMU 数量：中位 {_fmt(passed['batch_samples']['median'])}，P95 {_fmt(passed['batch_samples']['p95'])}，最大 {_fmt(passed['batch_samples']['max'])}", "",
             f"成功跟踪且 IMU 就绪代理为真的帧：{ready['frame_count']}/{ready['all_completed_calls_denominator']}；占比 {_fmt(100*ready['fraction_of_all_completed_calls'] if ready['fraction_of_all_completed_calls'] is not None else None, 2)}%",
             f"上述成功帧在全观察区间的返回率：{_fmt(ready['successful_returns_per_full_observation_second'])} 帧/s",
             f"状态计数：{json.dumps(report['tracking_state_counts'], ensure_ascii=False, sort_keys=True)}", ""]
    if not ready["available"]:
        lines.append("没有已初始化成功跟踪帧：不能评价初始化后的频率与耗时。")
    for label, group in (("全时段已完成调用", overall), ("成功且 IMU 就绪代理为真子集", ready)):
        span = group["camera_source_density"]
        host = group["completed_call_throughput"]
        match = group["matches"]
        lines += ["", label,
                  f"相机时间区间：{span['first']} → {span['last']} ns，跨度 {_fmt(span['span_s'])} s",
                  f"调用返回 RAW 区间：{host['first']} → {host['last']} ns，跨度 {_fmt(host['span_s'])} s",
                  f"跨进程关联：相机 {match['camera_rows']}，Python读取 {match['camera_read_rows']}，桥接 {match['input_rows']}，三项齐全 {match['all_three']}/{match['tracking_rows']}",
                  "耗时（ms）：项目 | 有效数量 | 中位 | P95 | 最大 | 负值数量"]
        for key, val in group["latency_ms"].items():
            lines.append(f"{LABELS[key]} | {val['count']} | {_fmt(val['median'])} | {_fmt(val['p95'])} | {_fmt(val['max'])} | {val['negative_count']}")
    lines += ["", ready["scope_note"], ready["imu_passed_to_algorithm"]["density_scope_note"]]
    lines += ["", "输入文件完整性"]
    for name, meta in report["files"].items():
        lines.append(f"{name}：{'存在' if meta['present'] else '缺失'}，有效 {meta['valid_rows']} 行，无效 {meta['invalid_rows']} 行")
    lines += ["", "解释范围"]+[f"- {note}" for note in report["notes"]]
    lines += ["", "复核提示"]+([f"- {warning}" for warning in report["warnings"]] or ["- 没有检测到记录格式或关联警告；仍需按上述解释范围理解结果。"])
    return "\n".join(lines)+"\n"


def write_report(session_dir):
    session = Path(session_dir)
    if not session.is_dir():
        raise ValueError("Session directory does not exist")
    report = analyze(session)
    (session/"performance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    (session/"performance.txt").write_text(render_text(report), encoding="utf-8")
    return report


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        return 2
    try:
        write_report(args[0])
    except (OSError, ValueError, TypeError, KeyError, OverflowError):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
