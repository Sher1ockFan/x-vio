"""Camera geometry and measured calibration snapshots for mono_vio_60."""
import hashlib
import json
import math
import os
from pathlib import Path
import re

PROFILE_NAME = "camera_profile.json"
PROFILE_KEYS = ("schema", "camera_id", "rig_id", "expected_card", "expected_bus",
                "raw_width", "raw_height", "crop_top", "crop_bottom", "requested_fps",
                "pixel_format", "timestamp_domain")
CLOCKS = {"unverified", "cntvct", "monotonic", "raw"}
TRIAL_STATUS = "borrowed_fit05_trial_unvalidated"
OMNI_CANDIDATE_STATUS = "calibrated_omni_candidate"
CALIBRATION_STATUSES = {"validated", TRIAL_STATUS, OMNI_CANDIDATE_STATUS}
FPS_CONTROL = "tegra_frame_rate_microhz"
NS = 1_000_000_000


def _error(message):
    return ValueError("VIO_E_CONFIG: " + message)


def _profile(value):
    if not isinstance(value, dict) or type(value.get("schema")) is not int or value["schema"] != 1:
        raise _error("Camera profile must have schema 1.")
    profile = dict(value)
    for key in ("camera_id", "rig_id", "device", "expected_card"):
        field = profile.get(key)
        if not isinstance(field, str) or not field.strip() or any(ord(c) < 32 for c in field):
            raise _error(f"Camera profile {key} must be a nonempty single-line string.")
    if not profile["device"].startswith("/dev/"):
        raise _error("Camera profile device must be an absolute /dev/ path.")
    bus = profile.setdefault("expected_bus", None)
    if bus is not None and (not isinstance(bus, str) or not bus.strip() or any(ord(c) < 32 for c in bus)):
        raise _error("Camera profile expected_bus must be null or a nonempty single-line string.")
    for key in ("raw_width", "raw_height", "crop_top", "crop_bottom", "requested_fps"):
        if type(profile.get(key)) is not int:
            raise _error(f"Camera profile {key} must be an integer.")
    if not 2 <= profile["raw_width"] <= 16384 or profile["raw_width"] % 2:
        raise _error("UYVY raw_width must be positive, even and at most 16384.")
    if not 1 <= profile["raw_height"] <= 16384:
        raise _error("Camera raw_height must be between 1 and 16384.")
    if min(profile["crop_top"], profile["crop_bottom"]) < 0 or \
            profile["crop_top"] + profile["crop_bottom"] >= profile["raw_height"]:
        raise _error("Camera crop must leave at least one complete image row.")
    if not 1 <= profile["requested_fps"] <= 240:
        raise _error("Camera requested_fps must be between 1 and 240.")
    if profile.get("pixel_format") != "UYVY":
        raise _error("Only UYVY camera input is supported.")
    if profile.get("timestamp_domain") not in CLOCKS:
        raise _error("Camera timestamp_domain must be unverified, cntvct, monotonic or raw.")
    if "frame_rate_control" in profile and profile["frame_rate_control"] != FPS_CONTROL:
        raise _error(f"Camera frame_rate_control must be {FPS_CONTROL} when specified.")
    return profile


def load_profile(path):
    try:
        return _profile(json.loads(Path(path).read_text()))
    except (OSError, json.JSONDecodeError) as exc:
        raise _error(f"Cannot read camera profile {path}: {exc}") from exc


def output_size(profile):
    profile = _profile(profile)
    return profile["raw_width"], profile["raw_height"] - profile["crop_top"] - profile["crop_bottom"]


def canonical_profile(profile):
    profile = _profile(profile)
    fields = {key: profile[key] for key in PROFILE_KEYS}
    if "frame_rate_control" in profile:
        fields["frame_rate_control"] = profile["frame_rate_control"]
    return fields


def profile_digest(profile):
    encoded = json.dumps(canonical_profile(profile), sort_keys=True, separators=(",", ":"),
                         allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def camera_env(profile):
    profile = _profile(profile)
    if profile["timestamp_domain"] == "unverified":
        raise _error("Camera timestamp domain is unverified; verify it before acquisition.")
    env = {key: value for key, value in os.environ.items() if not key.startswith("VIO_CAMERA_")}
    for variable, key in (("DEVICE", "device"), ("CARD", "expected_card"), ("WIDTH", "raw_width"),
                          ("HEIGHT", "raw_height"), ("CROP_TOP", "crop_top"),
                          ("CROP_BOTTOM", "crop_bottom"), ("FPS", "requested_fps"),
                          ("CLOCK", "timestamp_domain")):
        env["VIO_CAMERA_" + variable] = str(profile[key])
    if profile["expected_bus"] is not None:
        env["VIO_CAMERA_BUS"] = profile["expected_bus"]
    if "frame_rate_control" in profile:
        env["VIO_CAMERA_FPS_CONTROL"] = profile["frame_rate_control"]
    return env


def _settings(path):
    fields, key = {}, None
    try:
        lines = Path(path).read_text().splitlines()
    except OSError as exc:
        raise _error(f"Missing new camera calibration {path}; supply measured settings and timing.") from exc
    for line in lines:
        line = line.split("#", 1)[0].rstrip()
        match = re.match(r"^([A-Za-z][A-Za-z0-9_.]*):\s*(.*)$", line)
        if match:
            key, value = match.groups()
            if key in fields:
                raise _error("Duplicate calibration key " + key)
            fields[key] = value
        elif key and line.strip() and line[:1].isspace():
            fields[key] += " " + line.strip()
    return fields


def validate_calibration(settings, timing, profile):
    profile = _profile(profile)
    if profile["timestamp_domain"] == "unverified":
        raise _error("Camera timestamp domain is unverified; verify it before acquisition or VIO.")
    try:
        clock = json.loads(Path(timing).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise _error(f"Missing or invalid new camera timing calibration: {timing}") from exc
    fingerprint = profile_digest(profile)
    if not isinstance(clock, dict) or clock.get("calibration_status") not in CALIBRATION_STATUSES:
        raise _error("Camera/IMU extrinsic and timing calibration is required; missing or unknown calibration_status.")
    status = clock["calibration_status"]
    if clock.get("camera_profile_sha256") != fingerprint:
        raise _error("Calibration camera_profile_sha256 does not match this camera/rig/mode/clock.")
    seconds, shift = clock.get("timeshift_cam_imu_seconds"), clock.get("timeshift_cam_imu_rounded_ns")
    if (type(seconds) not in (int, float) or not math.isfinite(seconds) or type(shift) is not int
            or abs(shift) >= NS or shift != round(seconds * NS)):
        raise _error("Camera/IMU timing requires consistent finite seconds and integer nanoseconds.")
    for key in ("time_convention", "camera_mapping", "imu_mapping"):
        if not isinstance(clock.get(key), str) or not clock[key].strip():
            raise _error("Missing camera/IMU time convention field " + key)
    fields = _settings(settings)
    settings_status = fields.get("Camera.calibrationStatus", "").strip("\"'")
    if status == TRIAL_STATUS and settings_status != TRIAL_STATUS:
        raise _error("Borrowed trial requires matching Camera.calibrationStatus in settings.yaml.")
    if status == OMNI_CANDIDATE_STATUS and settings_status != status:
        raise _error("Omni candidate requires matching Camera.calibrationStatus in settings.yaml.")
    if status == "validated" and (settings_status == OMNI_CANDIDATE_STATUS or
                                  "borrowed" in settings_status or "unvalidated" in settings_status):
        raise _error("Calibration status conflicts: settings still identify provisional parameters.")

    def number(key):
        try:
            value = float(fields[key])
        except (ValueError, KeyError) as exc:
            raise _error("Missing or invalid calibration value " + key) from exc
        if not math.isfinite(value):
            raise _error("Nonfinite calibration value " + key)
        return value

    width, height = output_size(profile)
    for key, expected in (("Camera.width", width), ("Camera.height", height),
                          ("Camera.fps", profile["requested_fps"])):
        if number(key) != expected:
            raise _error(f"{key} must equal the camera profile value {expected}.")
    model = fields.get("Camera.type", "").strip("\"'")
    if model not in ("PinHole", "KannalaBrandt8", "OmniRadtan"):
        raise _error("Calibration Camera.type must be PinHole, KannalaBrandt8 or OmniRadtan.")
    if status == OMNI_CANDIDATE_STATUS and model != "OmniRadtan":
        raise _error("The Omni candidate requires Camera.type OmniRadtan.")
    if model == "OmniRadtan" and number("Camera1.xi") < 0:
        raise _error("Camera1.xi cannot be negative.")
    for key in ("Camera1.fx", "Camera1.fy"):
        if number(key) <= 0:
            raise _error(key + " must be positive.")
    if number("IMU.Frequency") != 500:
        raise _error("IMU.Frequency must be 500 for the configured FORSENSE packet clock.")
    for key in ("Camera1.cx", "Camera1.cy", "Camera1.k1", "Camera1.k2"):
        number(key)
    for key in (("Camera1.k3", "Camera1.k4") if model == "KannalaBrandt8" else ("Camera1.p1", "Camera1.p2")):
        number(key)
    for key in ("IMU.NoiseGyro", "IMU.NoiseAcc", "IMU.GyroWalk", "IMU.AccWalk"):
        if number(key) < 0:
            raise _error(key + " cannot be negative.")
    resize = ("Camera.newWidth" in fields, "Camera.newHeight" in fields)
    if status == OMNI_CANDIDATE_STATUS and (any(resize) or profile["crop_top"] or profile["crop_bottom"]):
        raise _error("The Omni candidate requires full-resolution images without crop or resize.")
    if any(resize):
        if not all(resize):
            raise _error("Both Camera.newWidth and Camera.newHeight are required for resize.")
        for key in ("Camera.newWidth", "Camera.newHeight"):
            value = number(key)
            if value <= 0 or value != int(value) or value > 16384:
                raise _error("Invalid resize dimension " + key)
    matrix = fields.get("IMU.T_b_c1", "")
    values = re.search(r"data:\s*\[([^\]]+)\]", matrix)
    if not matrix.startswith("!!opencv-matrix") or not values or \
            not all(re.search(r"\b" + key + r":\s*4\b", matrix) for key in ("rows", "cols")):
        raise _error("Calibration needs a finite 4x4 IMU.T_b_c1 matrix.")
    try:
        entries = [float(item) for item in values[1].split(",")]
    except ValueError as exc:
        raise _error("Invalid IMU.T_b_c1 matrix entries.") from exc
    if len(entries) != 16 or not all(math.isfinite(item) for item in entries) or \
            any(abs(a-b) > 1e-6 for a, b in zip(entries[-4:], (0, 0, 0, 1))):
        raise _error("Invalid IMU.T_b_c1 homogeneous matrix.")
    return {"width": width, "height": height, "pixels": width * height,
            "camera_shift_ns": shift, "profile_sha256": fingerprint,
            "calibration_status": status,
            "calibration_validated_for_current_rig": status == "validated"}


def load_snapshot(folder):
    folder = Path(folder)
    profile = load_profile(folder / PROFILE_NAME)
    return profile, validate_calibration(folder / "settings.yaml", folder / "timing.json", profile)
