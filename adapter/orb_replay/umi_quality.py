"""Join finalized metric camera poses to the recorded image index and evaluate physical holds."""
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import NamedTuple

import numpy as np


class Frame(NamedTuple):
    frame_idx: int
    sequence: int
    camera_raw_ns: int
    camera_ns: int
    imu_count: int
    offset: int
    size: int


FRAME_FIELDS = Frame._fields
UMI_FIELDS = "frame_idx timestamp state is_lost is_keyframe x y z q_x q_y q_z q_w".split()
XYZQ = UMI_FIELDS[5:]


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path, data):
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as f:
        tmp = Path(f.name)
        try:
            json.dump(data, f, indent=2, allow_nan=False)
            f.write("\n")
            f.flush()
            os.replace(tmp, path)
        finally:
            tmp.unlink(missing_ok=True)


def load_frames(folder):
    original = (folder / "capture.json").read_bytes()
    cap = json.loads(original)
    if cap["status"] == "RECORDING":
        raise ValueError("Recording is still open or finalizing. Wait for the recorder terminal to return.")
    if cap["status"] == "INCOMPLETE" and cap.get("error") == "KeyboardInterrupt":
        path = folder / "recovery.json"
        if not path.is_file():
            raise ValueError("Recording interrupted. Run umi-recover on this take before replay.")
        data = path.read_bytes()
        proof = json.loads(data)
        if (proof.get("format") != "umi-recovery-v1" or
                proof.get("source_capture_sha256") != hashlib.sha256(original).hexdigest()):
            raise ValueError("Recovery report does not match capture.json.")
        for name in ("frames.csv", "stream.bin"):
            found = proof["sha256"][name]
            if not isinstance(found, str) or not re.fullmatch(r"[0-9a-f]{64}", found):
                raise ValueError(f"Invalid recovery digest for {name}.")
            if cap["sha256"].get(name, found) != found:
                raise ValueError(f"Recovery disagrees with the recorder digest for {name}.")
            cap["sha256"][name] = found
        marks = folder / "marks.json"
        if (file_sha256(marks) if marks.exists() else None) != proof["marks_sha256"]:
            raise ValueError("marks.json changed since recovery.")
        cap.update(status="RECOVERED", recovery_sha256=hashlib.sha256(data).hexdigest())
    elif cap["status"] != "COMPLETE":
        raise ValueError("Recording incomplete. Inspect capture.json and record.log.")
    return cap, checked_frames(folder, cap)


def checked_frames(folder, cap):
    for name in ("settings.yaml", "timing.json", "frames.csv"):
        if file_sha256(folder / name) != cap["sha256"][name]:
            raise ValueError(f"{name} differs from its recorded checksum.")
    with (folder / "frames.csv").open(newline="") as file:
        reader = csv.reader(file)
        if next(reader, None) != list(FRAME_FIELDS):
            raise ValueError("Unknown source frame index format.")
        frames = []
        for i, row in enumerate(reader):
            if len(row) != len(FRAME_FIELDS):
                raise ValueError(f"Incomplete source index at frame {i}.")
            frames.append(Frame(*map(int, row)))
    offset = missing = 0
    for i, f in enumerate(frames):
        if (f.frame_idx != i or f.offset != offset or not 0 <= f.sequence < 2**32 or
                f.camera_raw_ns <= 0 or f.camera_ns - f.camera_raw_ns != cap["camera_shift_ns"] or
                not 1 <= f.imu_count <= 1000 or not 1920*1440 < f.size <= 1920*1440 + 256*(f.imu_count+1)):
            raise ValueError(f"Invalid source index at frame {i}.")
        if i:
            gap = (f.sequence - frames[i-1].sequence - 1) % 2**32
            if gap >= 2**31 or not 0 < f.camera_ns - frames[i-1].camera_ns < 950_000_000:
                raise ValueError(f"Invalid source sequence or clock at frame {i}.")
            missing += gap
        offset += f.size
    totals = (len(frames), missing, sum(f.imu_count for f in frames), offset)
    expected = (cap["frames"], cap["driver_missing"], cap["imu_samples"], (folder / "stream.bin").stat().st_size)
    if len(frames) < 2 or totals != expected:
        raise ValueError(f"Capture totals (frames, gaps, IMU samples, bytes): index {totals}, recording {expected}.")
    return frames


def keyed_rows(path, allowed):
    with path.open(newline="") as f:
        rows = {}
        for row in csv.DictReader(f):
            ns = int(row["camera_ns"])
            if ns in rows or ns not in allowed:
                raise ValueError(f"{path.name}: duplicate or unknown timestamp {ns}")
            rows[ns] = row
        return rows


def euroc_poses(path, stamp_lookup):
    result = {}
    with path.open() as file:
        for i, line in enumerate(file, 1):
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(f"{path.name}:{i}: expected a timestamp plus seven pose values")
            try:
                ns = stamp_lookup[int(Decimal(fields[0]).to_integral_value())]
            except (InvalidOperation, KeyError, OverflowError) as exc:
                raise ValueError(f"{path.name}:{i}: unmatched native timestamp") from exc
            pose = np.array(fields[1:], dtype=float)
            if not np.all(np.isfinite(pose)) or abs(np.linalg.norm(pose[3:]) - 1.) > .001:
                raise ValueError(f"{path.name}:{i}: invalid pose or quaternion")
            # The pinned tracker can repeat the prior timestamp for a frame with no pose.
            # Ambiguous duplicates are excluded, never shifted onto adjacent frames.
            if ns in result and (result[ns] is None or not np.array_equal(result[ns], pose)):
                result[ns] = None
            else:
                result[ns] = pose
    return result


def extrinsic(path):
    text = path.read_text()
    blocks = re.findall(r"^IMU\.T_b_c1:\s*!!opencv-matrix[^\n]*\n((?:[ \t]+[^\n]*\n?)+)", text, re.M)
    if len(blocks) != 1 or not all(re.search(rf"^\s*{key}:\s*4\s*$", blocks[0], re.M) for key in ("rows", "cols")):
        raise ValueError("Expected one 4x4 IMU.T_b_c1 in this take's settings.yaml.")
    data = re.search(r"\bdata:\s*\[([^\]]+)\]", blocks[0])
    if not data:
        raise ValueError("Missing IMU.T_b_c1 entries.")
    T = np.array([float(x) for x in data[1].split(",")]).reshape(4, 4)
    R = T[:3, :3]
    if (not np.all(np.isfinite(T)) or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7) or
            not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or abs(np.linalg.det(R)-1) > 1e-5):
        raise ValueError("IMU.T_b_c1 must be a rigid camera-to-body transform in metres.")
    return R, T[:3, 3]


def camera_pose(body, Rbc, tbc):
    # T_C0_Ct = inv(T_B_C) @ T_B0_Bt @ T_B_C. Conjugation rotates the
    # quaternion's vector part into the camera axes; the lever arm also rotates.
    q = body[3:] / np.linalg.norm(body[3:])
    cross = np.cross(q[:3], tbc)
    lever_delta = 2 * (q[3] * cross + np.cross(q[:3], cross))
    camera_q = np.r_[Rbc.T @ q[:3], q[3]]
    return np.r_[Rbc.T @ (body[:3] + lever_delta), camera_q / np.linalg.norm(camera_q)]


def ranges(mask):
    result = []
    for i, valid in enumerate(mask):
        if valid:
            if i and mask[i-1]:
                result[-1][1] = i
            else:
                result.append([i, i])
    return result


def export_trajectory(folder, out, cap, frames):
    stamps = {f.camera_ns for f in frames}
    states = keyed_rows(out / "tracking.csv", stamps)
    if len(states) != len(frames):
        raise ValueError("ORB did not process exactly one state per source image.")
    online = keyed_rows(out / "poses.csv", stamps)
    ready = {ns for ns, r in states.items() if r["tracking_state"] == "2" and r["imu_initialized"] == "1"}
    if ready != set(online):
        raise ValueError("Per-frame metric state disagrees with the recorded map samples.")
    # Reproduce the native int64 -> double seconds -> EuRoC nanoseconds path,
    # including floating point rounding after long system uptimes.
    lookup = {round((ns * 1e-9) * 1e9): ns for ns in stamps}
    if len(lookup) != len(frames):
        raise ValueError("Two sensor timestamps alias in the native time representation.")
    bodies = euroc_poses(out / "CameraTrajectory.txt", lookup)
    keyframes = euroc_poses(out / "KeyFrameTrajectory.txt", lookup)
    Rbc, tbc = extrinsic(folder / "settings.yaml")
    mask, maps = [], set()
    for f in frames:
        sample = online.get(f.camera_ns)
        valid = sample is not None and bodies.get(f.camera_ns) is not None
        mask.append(valid)
        if valid:
            if int(sample["segment"]) <= 0 or sample["segment"] != states[f.camera_ns]["segment"]:
                raise ValueError("Inconsistent native segment metadata.")
            maps.add(int(sample["map_id"]))
    if not any(mask) or not any(p is not None for p in keyframes.values()):
        raise ValueError("No finalized metric camera trajectory. See poses.csv and orb.log.")
    # Publish only after checking all source files. A failed attempt has its own directory.
    path = out / "camera_trajectory.csv"
    with tempfile.NamedTemporaryFile(mode="w", newline="", dir=out, delete=False) as file:
        tmp = Path(file.name)
        try:
            writer = csv.writer(file)
            writer.writerow(UMI_FIELDS)
            for f, valid in zip(frames, mask):
                ns = f.camera_ns
                dt = f.camera_raw_ns - frames[0].camera_raw_ns
                seconds = f"{dt // 10**9}.{dt % 10**9:09d}"
                kf = valid and keyframes.get(ns) is not None
                values = ([f"{v:.10f}" for v in camera_pose(bodies[ns], Rbc, tbc)] if valid else [0]*7)
                writer.writerow([f.frame_idx, seconds, states[ns]["tracking_state"],
                                 str(not valid).lower(), str(kf).lower(), *values])
            file.flush()
            os.replace(tmp, path)
        finally:
            tmp.unlink(missing_ok=True)
    good, bad = ranges(mask), ranges([not v for v in mask])
    max_lost = max((j-i+1 for i, j in bad), default=0)
    deltas = np.diff([f.camera_raw_ns for f in frames]) / 1e6
    return {"valid_frames": sum(mask), "lost_frames": len(frames)-sum(mask), "max_consecutive_lost": max_lost,
            "trajectory_sha256": file_sha256(path), "online_poses_sha256": file_sha256(out / "poses.csv"),
            "valid_ranges_inclusive": good, "driver_missing": cap["driver_missing"], "online_map_ids_with_final_poses": sorted(maps),
            "measured_hz": (len(frames)-1)*1e9 / (frames[-1].camera_raw_ns-frames[0].camera_raw_ns),
            "frame_dt_median_ms": float(np.median(deltas)), "frame_dt_max_ms": float(max(deltas)),
            "coordinates": "camera at first keyframe <- camera at frame t; metres, quaternion xyzw; largest final ORB map",
            "time": "seconds from first recorded image; frames.csv holds raw and IMU-aligned sensor ns",
            "absolute_accuracy": "UNVERIFIED"}


def angle(q1, q2):
    dot = np.abs(np.asarray(q2) @ q1)
    return np.degrees(2 * np.arccos(np.clip(dot, 0, 1)))


def center(poses):
    p = np.mean(poses[:, :3], axis=0)
    qs = poses[:, 3:] / np.linalg.norm(poses[:, 3:], axis=1, keepdims=True)
    # Principal eigenvector of the scatter matrix is sign-invariant for q and -q.
    q = np.linalg.eigh(qs.T @ qs)[1][:, -1]
    radii = 1000 * np.linalg.norm(poses[:, :3] - p, axis=1)
    angles = angle(q, qs)
    return p, q, {"frames": len(poses), "translation_rms_mm": float(np.sqrt(np.mean(radii**2))),
                  "translation_p95_mm": float(np.percentile(radii, 95)),
                  "rotation_p95_deg": float(np.percentile(angles, 95))}


def checked_holds(folder, frames):
    marks = json.loads((folder / "marks.json").read_text())
    if not isinstance(marks, dict) or marks.get("clock") != "CLOCK_MONOTONIC_RAW":
        raise ValueError("Unexpected mark clock.")
    holds = marks.get("holds")
    names = ("A", "B", "return_A")
    if not isinstance(holds, dict) or any(
            not isinstance(holds.get(k), list) or len(holds[k]) != 2 or
            any(type(t) is not int for t in holds[k]) for k in names):
        raise ValueError("Expected three pairs of integer RAW timestamps in marks.json.")
    ordered = [t for name in names for t in holds[name]]
    if (not all(a < b for a, b in zip(ordered, ordered[1:])) or
            ordered[0] < frames[0].camera_raw_ns or ordered[-1] > frames[-1].camera_raw_ns):
        raise ValueError("Marks fall outside the recording or overlap.")
    return {name: holds[name] for name in names}


def assess_holds(folder, out, frames, rows):
    holds = checked_holds(folder, frames)
    timestamps = np.array([f.camera_raw_ns for f in frames], dtype=np.int64)
    first = int(np.searchsorted(timestamps, holds["A"][0], side="left"))
    stop = int(np.searchsorted(timestamps, holds["return_A"][1], side="right"))
    indices = list(range(int(first), stop))
    if not indices or any(rows[i]["is_lost"] != "false" for i in indices):
        raise ValueError("A→B→A includes a lost or uninitialized frame. Choose a new complete take.")
    seqs = [frames[i].sequence for i in indices]
    if any((b-a) % 2**32 != 1 for a, b in zip(seqs, seqs[1:])):
        raise ValueError("Driver sequence gap in A→B→A.")
    # Cadence/coverage screen only. This threshold is not a pose accuracy tolerance.
    max_gap = max(100_000_000, 3 * float(np.median(np.diff(timestamps))))
    if len(indices) < 2 or max(np.diff(timestamps[indices])) > max_gap:
        raise ValueError("Source timestamps have a long gap inside A→B→A.")
    online = keyed_rows(out / "poses.csv", {f.camera_ns for f in frames})
    map_ids = {int(online[frames[i].camera_ns]["map_id"]) for i in indices}
    if len(map_ids) != 1:
        raise ValueError("Map identity changed during the reference test.")
    centers = []
    details = {}
    for name, (a, b) in holds.items():
        ix = np.flatnonzero((timestamps >= a) & (timestamps <= b))
        if len(ix) < 3 or timestamps[ix[0]]-a > max_gap or b-timestamps[ix[-1]] > max_gap:
            raise ValueError(f"{name}: insufficient image coverage of the physical hold.")
        poses = np.array([[float(rows[i][k]) for k in XYZQ] for i in ix])
        p, q, details[name] = center(poses)
        details[name]["source_frames_inclusive"] = [int(ix[0]), int(ix[-1])]
        centers.append((p, q))
    p0, q0 = centers[0]

    def displacement(pair):
        p, q = pair
        return {"translation_mm": float(1000*np.linalg.norm(p-p0)), "rotation_deg": float(angle(q0, q))}

    return {"map_id": map_ids.pop(), "holds": details, "A_to_B": displacement(centers[1]),
            "return_to_A": displacement(centers[2])}


def measure(out, args):
    summary = json.loads((out / "solve.json").read_text())
    if summary["status"] != "EXPORTED":
        raise ValueError("No completed trajectory. Inspect solve.json and orb.log.")
    folder = out.parent
    cap, frames = load_frames(folder)
    if cap["status"] == "RECOVERED" and summary.get("source_recovery_sha256") != cap["recovery_sha256"]:
        raise ValueError("Solve and current recovery report disagree.")
    if (file_sha256(out / "camera_trajectory.csv") != summary["trajectory_sha256"] or
            file_sha256(out / "poses.csv") != summary["online_poses_sha256"]):
        raise ValueError("Trajectory files have changed since the solve completed.")
    with (out / "camera_trajectory.csv").open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames != UMI_FIELDS:
            raise ValueError("Unexpected trajectory schema.")
        rows = list(reader)
    if len(rows) != len(frames) or any(int(row["frame_idx"]) != i for i, row in enumerate(rows)):
        raise ValueError("Trajectory indices disagree with source images.")
    options = [(args.distance_mm, args.distance_tol_mm, "translation_mm"),
               (args.angle_deg, args.angle_tol_deg, "rotation_deg")]
    for ref, tol, key in options:
        if (tol is not None and ref is None or
                any(not math.isfinite(v) or v < 0 for v in (ref, tol) if v is not None) or
                key == "rotation_deg" and ref is not None and ref > 180):
            raise ValueError("Provide a finite nonnegative reference for each tolerance; rotation is in [0, 180]°.")
    report = {"status": "NO_HOLD_MARKS", "source_capture_status": cap["status"], "absolute_trajectory_accuracy": "UNVERIFIED",
              "valid_frames": summary["valid_frames"], "total_frames": len(frames),
              "driver_missing": summary["driver_missing"], "max_consecutive_lost": summary["max_consecutive_lost"]}
    if (folder / "marks.json").exists():
        try:
            report.update(assess_holds(folder, out, frames, rows))
            report["status"] = "REPEATABILITY_MEASURED"
            # B is a deliberately different physical pose. A frozen estimator
            # can otherwise appear to have perfect return-to-start accuracy.
            radii = [report["holds"][k]["translation_p95_mm"] for k in ("A", "B")]
            angles = [report["holds"][k]["rotation_p95_deg"] for k in ("A", "B")]
            if (report["A_to_B"]["translation_mm"] <= max(1e-6, sum(radii)) and
                    report["A_to_B"]["rotation_deg"] <= max(1e-5, sum(angles))):
                report.update(status="REFERENCE_RUN_REVIEW",
                              reason="The different physical pose B was unresolved above the static scatter.")
        except (ValueError, KeyError) as exc:
            report.update(status="REFERENCE_RUN_REVIEW", reason=str(exc))
    refs = {}
    for ref, tol, key in options:
        if ref is None or "A_to_B" not in report:
            continue
        error = abs(report["A_to_B"][key] - ref)
        refs[key] = {"reference": ref, "absolute_error": error}
        if tol is not None:
            refs[key].update(tolerance=tol, within_tolerance=error <= tol)
    report["A_to_B_reference_check"] = refs
    path = out / "accuracy.json"
    dump_json(path, report)
    print(f"有效位姿 {report['valid_frames']}/{len(frames)} 帧；最长连续无效 {report['max_consecutive_lost']} 帧；"
          f"驱动序号缺失 {report['driver_missing']} 帧。")
    print(report["status"], report.get("reason", ""))
    if "return_to_A" in report:
        for name, hold in report["holds"].items():
            print(f"{name} 静止 P95：{hold['translation_p95_mm']:.2f} mm，{hold['rotation_p95_deg']:.3f}°")
        for key, name in (("A_to_B", "A 到 B"), ("return_to_A", "返回 A 的偏差")):
            d = report[key]
            print(f"{name}：{d['translation_mm']:.2f} mm，{d['rotation_deg']:.3f}°")
        for key, values in refs.items():
            print(f"A→B {key}: {values}")
    print("全程绝对精度：未验证。", path)
    if (report["status"] == "REFERENCE_RUN_REVIEW" or
            report["status"] == "NO_HOLD_MARKS" and any(ref is not None for ref, _, _ in options) or
            any(v.get("within_tolerance") is False for v in refs.values())):
        raise SystemExit(3)
