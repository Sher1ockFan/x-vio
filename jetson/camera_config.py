#!/usr/bin/env python3
"""Inspect/validate the selected camera configuration without opening any device."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools/orb_replay"))
sys.path.insert(0, str(ROOT.parent / "adapter/orb_replay"))
import camera_profile


def incomplete_parameters(settings, timing, profile, profile_path):
    """Report independent missing inputs together, before opening any hardware."""
    issues = []
    try:
        fields = camera_profile._settings(settings)
        pending = [key for key, value in fields.items() if "REQUIRED" in value]
        if pending:
            issues.append(f"未填写 {settings}: " + ", ".join(pending))
    except (OSError, ValueError) as exc:
        issues.append(f"无法读取相机/IMU 参数 {settings}: {exc}")
    try:
        clock = json.loads(timing.read_text())
        if not isinstance(clock, dict):
            issues.append(f"{timing} 必须是 JSON 对象")
        else:
            pending = [key for key in ("timeshift_cam_imu_seconds", "timeshift_cam_imu_rounded_ns")
                       if clock.get(key) is None]
            if pending:
                issues.append(f"未填写 {timing}: " + ", ".join(pending))
            if clock.get("calibration_status") not in camera_profile.CALIBRATION_STATUSES:
                issues.append(f"{timing}: 缺少已识别的 calibration_status")
    except FileNotFoundError:
        issues.append(f"缺少时间参数文件 {timing}")
    except (OSError, ValueError) as exc:
        issues.append(f"无法读取时间参数 {timing}: {exc}")
    if profile["timestamp_domain"] == "unverified":
        issues.append(f"{profile_path}: timestamp_domain=unverified")
    return issues


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("describe", "device", "validate"))
    ap.add_argument("profile", type=Path)
    ap.add_argument("settings", type=Path, nargs="?")
    ap.add_argument("timing", type=Path, nargs="?")
    args = ap.parse_args()
    try:
        profile = camera_profile.load_profile(args.profile)
        width, height = camera_profile.output_size(profile)
        if args.action == "device":
            print(profile["device"])
        elif args.action == "validate":
            if args.settings is None or args.timing is None:
                raise ValueError("VIO_E_CONFIG: validate requires settings and timing files")
            issues = incomplete_parameters(args.settings, args.timing, profile, args.profile)
            if issues:
                raise ValueError("VIO_E_CALIBRATION: 配置不完整；" + "；".join(issues))
            layout = camera_profile.validate_calibration(args.settings, args.timing, profile)
            if layout["calibration_status"] == camera_profile.TRIAL_STATUS:
                print("Trial calibration: 暂用旧 fit_05 内参、外参、IMU 和时间偏移；"
                      "新相机未标定，时钟域暂沿用旧配置，试跑结果不能用于精度验收。")
            elif layout["calibration_status"] == camera_profile.OMNI_CANDIDATE_STATUS:
                print("Calibration candidate: 使用当前相机的 Omni-radtan 内参和联合标定结果；"
                      "保留全分辨率，独立精度复验已跳过。")
            print(f"Camera profile: {profile['device']}; {profile['raw_width']}x{profile['raw_height']} UYVY; "
                  f"crop top={profile['crop_top']} bottom={profile['crop_bottom']}; "
                  f"output={width}x{height}; requested_fps={profile['requested_fps']} (not measured).")
        else:
            print(json.dumps({"profile": profile, "output_width": width, "output_height": height,
                              "camera_profile_sha256": camera_profile.profile_digest(profile),
                              "hardware_opened": False}, ensure_ascii=False, indent=2))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        message = str(exc)
        if not message.startswith("VIO_E_"):
            message = "VIO_E_CONFIG: " + message
        print(message, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
