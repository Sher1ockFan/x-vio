#!/usr/bin/env python3
"""Plan live Atlas reuse and publish the first clean mature map without replacement.

The core owns .osa serialization and its guarded format. This helper owns the
calibration/time-basis manifest and the lifetime of the user's reference file.
Only a per-run settings copy is edited; sealed recordings are never changed.
"""
import argparse
from contextlib import contextmanager
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

SCHEMA = 2
PUBLICATION_POLICY = "health_v2"


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def scalar(value):
    value = value.strip()
    if value.startswith(('"', "'")):
        return value.strip("\"'")
    try:
        number = float(value)
    except ValueError:
        return value
    if not math.isfinite(number):
        raise ValueError("MAP_E_CALIBRATION: non-finite calibration value")
    return number


def calibration(settings, timing):
    """Parse the limited OpenCV YAML fields used by this single-camera package."""
    blocks = {}
    key = None
    for line in Path(settings).read_text().splitlines():
        line = line.split("#", 1)[0].rstrip()
        match = re.match(r"^([A-Za-z][A-Za-z0-9_.]*):\s*(.*)$", line)
        if match:
            key, value = match.groups()
            if key in blocks:
                raise ValueError("MAP_E_CALIBRATION: duplicate settings key " + key)
            blocks[key] = [value]
        elif key and line.strip():
            blocks[key].append(line.strip())
    fields = {}
    for key, lines in blocks.items():
        if not (key.startswith("Camera1.") or key.startswith("Camera.") or
                key in ("IMU.T_b_c1", "IMU.Frequency", "IMU.NoiseGyro", "IMU.NoiseAcc",
                        "IMU.GyroWalk", "IMU.AccWalk")):
            continue
        value = " ".join(lines)
        if value.startswith("!!opencv-matrix"):
            match = re.search(r"data:\s*\[([^\]]+)\]", value)
            if not match or key != "IMU.T_b_c1":
                raise ValueError("MAP_E_CALIBRATION: unsupported calibration matrix")
            matrix = [scalar(v) for v in match.group(1).split(",")]
            if len(matrix) != 16 or not all(isinstance(v, float) for v in matrix):
                raise ValueError("MAP_E_CALIBRATION: expected a finite 4x4 IMU.T_b_c1")
            fields[key] = matrix
        else:
            fields[key] = scalar(value)
    required = {"Camera.type", "Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy",
                "Camera.width", "Camera.height", "IMU.T_b_c1", "IMU.Frequency"}
    if not required.issubset(fields):
        raise ValueError("MAP_E_CALIBRATION: missing camera/IMU calibration fields")
    clock = json.loads(Path(timing).read_text())
    ns = clock["timeshift_cam_imu_rounded_ns"]
    if type(ns) is not int or ns != round(clock["timeshift_cam_imu_seconds"] * 1e9):
        raise ValueError("MAP_E_CALIBRATION: inconsistent camera time shift")
    # Capture-specific clock slope/intercept examples and report hashes are not
    # calibration. The current session refits clocks before tracking starts.
    basis = {k: clock[k] for k in ("time_convention", "camera_mapping", "imu_mapping")}
    basis["timeshift_cam_imu_rounded_ns"] = ns
    payload = {"camera_imu": fields, "time_basis": basis}
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest(), payload


def sidecar(path):
    return Path(str(path) + ".meta.json")


@contextmanager
def reference_lock(reference):
    reference.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(reference) + ".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def validate_reference(reference, fingerprint):
    metadata = sidecar(reference)
    if not reference.is_file() or not metadata.is_file():
        raise ValueError("MAP_E_MANIFEST: reference requires both .osa and .osa.meta.json")
    manifest = json.loads(metadata.read_text())
    if manifest.get("schema") != SCHEMA or manifest.get("publication_policy") != PUBLICATION_POLICY:
        raise ValueError("MAP_E_FORMAT: reference predates health-gated Atlas v2; use a new reference path")
    if manifest.get("calibration_sha256") != fingerprint:
        raise ValueError("MAP_E_CALIBRATION: reference calibration/time basis differs")
    if (manifest.get("health_review") or {}).get("status") != "PASS":
        raise ValueError("MAP_E_HEALTH: reference requires review and cannot be used automatically")
    if reference.stat().st_size == 0 or manifest.get("atlas_sha256") != digest(reference):
        raise ValueError("MAP_E_CHECKSUM: reference map changed or is incomplete")


def prepare(root, run):
    root, run = Path(root).resolve(), Path(run).resolve()
    settings, timing = run / "settings.yaml", run / "timing.json"
    mode = os.environ.get("VIO_MAP_MODE", "auto")
    if mode not in ("auto", "new", "off"):
        raise ValueError("MAP_E_MODE: VIO_MAP_MODE must be auto, new, or off")
    # v2 adds recoverable dynamics priors and health checks. Do not automatically
    # reload a v1 reference that may already contain an accepted divergent map.
    # Keep that file untouched for diagnosis or use with its original binary.
    reference = Path(os.environ.get("VIO_MAP_FILE", str(root / "maps/workspace_v2.osa"))).expanduser().absolute()
    if reference.suffix != ".osa" or "\n" in str(reference) or "\r" in str(reference):
        raise ValueError("MAP_E_PATH: VIO_MAP_FILE must name an .osa file")
    fingerprint, parameters = calibration(settings, timing)
    load = ""
    if mode != "off":
        with reference_lock(reference):
            exists = reference.exists() or sidecar(reference).exists()
            if exists and mode == "new":
                raise ValueError("MAP_E_EXISTS: new mode needs an unused reference path")
            if exists:
                validate_reference(reference, fingerprint)
                load = str(reference)
    save = str(run / "session/atlas.osa") if mode != "off" else ""
    # Refuse duplicate injected keys rather than letting OpenCV choose one.
    text = settings.read_text()
    if re.search(r"^(MapReuse\.|System\.(LoadAtlasFromFile|SaveAtlasToFile)\s*:)", text, re.M):
        raise ValueError("MAP_E_CONFIG: base settings already contain map persistence keys")
    text += "\n# Per-run map lifecycle; base calibration and recordings are unchanged.\n"
    text += "MapReuse.Enabled: " + ("0" if mode == "off" else "1") + "\n"
    text += "MapReuse.LoadPath: " + json.dumps(load) + "\n"
    text += "MapReuse.SavePath: " + json.dumps(save) + "\n"
    settings.write_text(text)
    plan = {"schema": SCHEMA, "publication_policy": PUBLICATION_POLICY,
            "mode": mode, "reference": str(reference), "load": load,
            "save": save, "calibration_sha256": fingerprint, "calibration": parameters}
    write_json(run / "map_plan.json", plan)
    print("Map load: " + (
        "DISABLED 旧地图加载已关闭（VIO_MAP_MODE=off）。" if mode == "off" else
        "LOADING 正在加载旧地图：" + load if load else
        "NO_REFERENCE 没有旧地图，开始新建图；参考地图路径：" + str(reference)
    ), flush=True)
    return plan


def publish(reference, saved, metadata, fingerprint):
    """Commit only to a vacant reference, serialized with other live sessions."""
    with reference_lock(reference):
        if reference.exists() or sidecar(reference).exists():
            validate_reference(reference, fingerprint)
            return "EXISTING_REFERENCE_PRESERVED"
        fd, name = tempfile.mkstemp(prefix=".atlas-", suffix=".osa", dir=reference.parent)
        staged = Path(name)
        committed_metadata = False
        try:
            with os.fdopen(fd, "wb") as out, saved.open("rb") as source:
                shutil.copyfileobj(source, out, 1024 * 1024)
                out.flush()
                os.fsync(out.fileno())
            if digest(staged) != metadata["atlas_sha256"]:
                raise ValueError("MAP_E_CHECKSUM: saved Atlas changed during publication")
            # Metadata first, map last: a reader cannot observe a committed map
            # without its fingerprint. The lock covers all helper readers.
            write_json(sidecar(reference), metadata)
            committed_metadata = True
            os.link(staged, reference)  # atomic and refuses an existing map
            directory_fd = os.open(reference.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except BaseException:
            if committed_metadata and not reference.exists():
                sidecar(reference).unlink(missing_ok=True)
            raise
        finally:
            staged.unlink(missing_ok=True)
    return "REFERENCE_PUBLISHED"


def review_health(session):
    """Fail closed for automatic publication, without claiming pose accuracy.

    Every completed adapter call has one row, including invalid/searching frames.
    A low-feature rejection is ordinary loss; hard_fault is explicitly set by
    the core only for numerical or cross-sensor inconsistency.
    """
    path = Path(session) / "state_diagnostics.csv"
    review = {"policy": PUBLICATION_POLICY, "status": "REVIEW_REQUIRED", "rows": 0,
              "valid_frames": 0, "hard_fault_frames": 0, "first_faults": []}
    flags = ("health_evaluated", "health_accepted", "hard_fault", "dynamics_available", "valid")
    dynamics = tuple(prefix + axis + suffix
                     for prefix, suffix in (("velocity_w_", "_mps"),
                                            ("gyro_bias_", "_radps"),
                                            ("accel_bias_", "_mps2"))
                     for axis in ("x", "y", "z"))
    required = {"frame_idx", "camera_ns", "health_reason", *flags, *dynamics}
    previous_ns = -1
    try:
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            names = reader.fieldnames or []
            if len(names) != len(set(names)) or not required.issubset(names):
                raise ValueError("Missing or duplicate diagnostic columns")
            for row in reader:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("Truncated or oversized diagnostic row")
                index, camera_ns = int(row["frame_idx"]), int(row["camera_ns"])
                if index != review["rows"] or camera_ns <= previous_ns:
                    raise ValueError("Non-contiguous frames or non-increasing timestamps")
                if any(row[name] not in ("0", "1") for name in flags):
                    raise ValueError("Invalid diagnostic boolean")
                if row["health_accepted"] == "1" and row["health_evaluated"] != "1":
                    raise ValueError("Accepted health state was not evaluated")
                previous_ns = camera_ns
                review["rows"] += 1
                if row["hard_fault"] == "1":
                    review["hard_fault_frames"] += 1
                    if len(review["first_faults"]) < 20:
                        review["first_faults"].append({"frame_idx": index, "camera_ns": camera_ns,
                                                        "reason": row["health_reason"]})
                if row["valid"] == "1":
                    if (row["health_evaluated"] != "1" or row["health_accepted"] != "1" or
                            row["hard_fault"] != "0" or row["dynamics_available"] != "1"):
                        raise ValueError("Valid pose lacks an accepted health check")
                    if not all(math.isfinite(float(row[name])) for name in dynamics):
                        raise ValueError("Valid pose has non-finite velocity or bias")
                    review["valid_frames"] += 1
        review["diagnostics_sha256"] = digest(path)
        if not review["rows"] or not review["valid_frames"]:
            review["reason"] = "No health-accepted inertial poses were recorded."
        elif review["hard_fault_frames"]:
            review["reason"] = "Numerical or visual-inertial inconsistency requires review before reference publication."
        else:
            review["status"] = "PASS"
            review["reason"] = "No recorded hard health faults; this is not an accuracy certificate."
    except (OSError, ValueError, TypeError, csv.Error) as error:
        review["reason"] = "Missing or invalid state diagnostics: " + str(error)
    return review


def finalize(session, worker_exit):
    session = Path(session).resolve()
    plan = json.loads((session.parent / "map_plan.json").read_text())
    summary = {"schema": SCHEMA, "mode": plan["mode"], "worker_exit": worker_exit,
               "reference": plan["reference"], "status": "NOT_PUBLISHED"}
    try:
        if plan["mode"] == "off":
            summary["status"] = "DISABLED"
            return summary
        if plan.get("schema") != SCHEMA or plan.get("publication_policy") != PUBLICATION_POLICY:
            raise ValueError("MAP_E_FORMAT: live plan lacks Atlas v2 health publication policy")
        report_file = session / "map_save.json"
        report = json.loads(report_file.read_text()) if report_file.exists() else {}
        summary["core_save"] = report
        saved = Path(plan["save"])
        if report.get("status") == "SAVED":
            if (not isinstance(report.get("path"), str) or Path(report["path"]).resolve() != saved or
                    not saved.is_file() or saved.stat().st_size == 0):
                raise ValueError("MAP_E_SAVE: core save report does not match a nonempty Atlas")
            health = review_health(session)
            summary["health_review"] = health
            metadata = {"schema": SCHEMA, "publication_policy": PUBLICATION_POLICY,
                        "health_review": health, "calibration_sha256": plan["calibration_sha256"],
                        "calibration": plan["calibration"], "atlas_sha256": digest(saved),
                        "source_session": str(session), "core_save": report}
            write_json(sidecar(saved), metadata)
        if worker_exit != 0:
            summary["reason"] = "Unclean session; saved diagnostic map is not promoted."
            return summary
        if report.get("status") == "SKIPPED":
            summary["status"] = "NO_MATURE_MAP"
            summary["reason"] = report.get("reason", "")
            return summary
        if report.get("status") != "SAVED":
            raise ValueError("MAP_E_SAVE: missing or failed core map save report")
        if health["status"] != "PASS":
            summary["status"] = "HEALTH_REVIEW_REQUIRED"
            summary["reason"] = health["reason"]
            return summary
        if plan["load"]:
            summary["status"] = "EXISTING_REFERENCE_PRESERVED"
        else:
            summary["status"] = publish(Path(plan["reference"]), saved, metadata, plan["calibration_sha256"])
        return summary
    except BaseException as error:
        summary["status"] = "ERROR"
        summary["reason"] = str(error)
        raise
    finally:
        write_json(session / "map_session.json", summary)
        print("Map session: " + json.dumps(summary, sort_keys=True), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prepare_parser = sub.add_parser("prepare")
    prepare_parser.add_argument("root", type=Path)
    prepare_parser.add_argument("run", type=Path)
    finalize_parser = sub.add_parser("finalize")
    finalize_parser.add_argument("session", type=Path)
    finalize_parser.add_argument("worker_exit", type=int)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.root, args.run)
    else:
        finalize(args.session, args.worker_exit)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
