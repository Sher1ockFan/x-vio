"""Calibration selection and field-conversion contracts, without sensor access."""
import ast
from contextlib import redirect_stdout, redirect_stderr
import io
import json
import os
from pathlib import Path
import re
import sys
import tempfile
import unittest
from unittest.mock import patch
from test_camera_profile import ROOT, profile_api, maps, source_module, write_snapshot

config = source_module("trial_camera_config", "jetson/camera_config.py")
TRIAL = profile_api.TRIAL_STATUS


def trial_snapshot(folder):
    profile, clock = write_snapshot(folder)
    cfg = folder / "settings.yaml"
    cfg.write_text(cfg.read_text() + f'Camera.calibrationStatus: "{TRIAL}"\n')
    clock["calibration_status"] = TRIAL
    (folder / "timing.json").write_text(json.dumps(clock))
    return profile, clock


def matrix_values(value):
    return [float(v) for v in re.search(r"data:\s*\[([^\]]+)\]", value)[1].split(",")]


class BorrowedTrialTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mono60-trial-")
        self.root = Path(self.temp.name).resolve()
        self.addCleanup(self.temp.cleanup)
        clean = {k: v for k, v in os.environ.items() if not k.startswith("VIO_MAP_")}
        self.env = patch.dict(os.environ, clean, clear=True)
        self.env.start()
        self.addCleanup(self.env.stop)

    def test_borrowed_snapshot_reuses_all_legacy_calibration_values_exactly(self):
        params = ROOT / "params"
        borrowed = params / "borrowed_fit05_trial"
        old = profile_api._settings(params / "legacy_fit05/orb_trial.yaml")
        new = profile_api._settings(borrowed / "orb_trial.yaml")
        for key in old:
            if key.startswith("Camera1.") or key.startswith("IMU."):
                with self.subTest(key=key):
                    if key == "IMU.T_b_c1":
                        self.assertEqual(matrix_values(old[key]), matrix_values(new[key]))
                    else:
                        self.assertEqual(float(old[key]), float(new[key]))
        legacy_clock = json.loads((params / "legacy_fit05/timing.json").read_text())
        clock = json.loads((borrowed / "timing.json").read_text())
        for key in ("timeshift_cam_imu_seconds", "timeshift_cam_imu_rounded_ns", "time_convention", "imu_mapping"):
            self.assertEqual(clock[key], legacy_clock[key])
        self.assertFalse(any("example" in key or "first_last" in key for key in clock))
        profile = profile_api.load_profile(borrowed / "camera_profile.json")
        layout = profile_api.validate_calibration(borrowed / "orb_trial.yaml", borrowed / "timing.json", profile)
        self.assertEqual(profile["device"], "/dev/video0")
        self.assertEqual(profile["expected_card"], "vi-output, jl_gmsl 10-003a")
        self.assertEqual(profile["timestamp_domain"], "cntvct")
        self.assertEqual((profile["crop_top"], profile["crop_bottom"]), (0, 0))
        self.assertEqual((layout["width"], layout["height"]), (1920, 1536))
        self.assertEqual((float(new["Camera.newWidth"]), float(new["Camera.newHeight"])), (960, 768))
        self.assertEqual(profile["requested_fps"], 60)
        self.assertEqual(layout["calibration_status"], TRIAL)
        self.assertFalse(layout["calibration_validated_for_current_rig"])

    def test_native_omni_fit_is_exact_and_full_resolution(self):
        params = ROOT / "params"
        source = params / "calibration_60"
        settings = params / "orb_trial.yaml"
        profile = profile_api.load_profile(params / "camera_profile.json")
        layout = profile_api.validate_calibration(settings, params / "timing.json", profile)
        fields = profile_api._settings(settings)
        camchain = (source / "camchain-imucam.yaml").read_text()
        candidate = json.loads((source / "candidate.json").read_text())
        intrinsics = ast.literal_eval(re.search(r"^  intrinsics: (.+)$", camchain, re.M)[1])
        distortion = ast.literal_eval(re.search(r"^  distortion_coeffs: (.+)$", camchain, re.M)[1])
        for key, value in zip(("xi", "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"),
                              intrinsics + distortion):
            self.assertEqual(float(fields["Camera1." + key]), value)
        self.assertEqual(fields["Camera.type"], '"OmniRadtan"')
        self.assertNotIn("Camera.newWidth", fields)
        self.assertNotIn("Camera.newHeight", fields)
        self.assertEqual((profile["crop_top"], profile["crop_bottom"]), (0, 0))
        self.assertEqual((layout["width"], layout["height"]), (1920, 1536))
        self.assertEqual(profile["requested_fps"], 60)
        self.assertEqual(profile["timestamp_domain"], "cntvct")
        self.assertEqual(profile_api.camera_env(profile)["VIO_CAMERA_FPS_CONTROL"], profile_api.FPS_CONTROL)
        self.assertEqual(layout["camera_shift_ns"], 14502238)
        self.assertEqual(layout["camera_shift_ns"], round(1e9 * candidate["timeshift_cam_imu_seconds"]))
        self.assertEqual(layout["calibration_status"], profile_api.OMNI_CANDIDATE_STATUS)
        self.assertFalse(layout["calibration_validated_for_current_rig"])
        cb = matrix_values(fields["IMU.T_b_c1"])
        bc = candidate["T_cam_imu"]
        for row in range(4):
            for col in range(4):
                total = sum(bc[row][i] * cb[4 * i + col] for i in range(4))
                self.assertAlmostEqual(total, int(row == col), places=12)
        modified = self.root / "settings.yaml"
        for edit in (
            settings.read_text() + "Camera.newWidth: 960\nCamera.newHeight: 768\n",
            settings.read_text().replace("Camera1.xi:", "Camera1.k3:"),
            settings.read_text().replace('"OmniRadtan"', '"KannalaBrandt8"'),
        ):
            modified.write_text(edit)
            with self.assertRaises(ValueError):
                profile_api.validate_calibration(modified, params / "timing.json", profile)

    def test_omni_map_cannot_reuse_borrowed_reference(self):
        params = ROOT / "params"
        run = self.root / "run"
        trial_snapshot(run)
        with redirect_stdout(io.StringIO()):
            plan = maps.prepare(self.root, run)
        old_reference = Path(plan["reference"])
        old_reference.write_bytes(b"SYNTHETIC BORROWED ATLAS")
        maps.sidecar(old_reference).write_text(json.dumps(dict(
            schema=maps.SCHEMA, publication_policy=maps.PUBLICATION_POLICY,
            calibration_sha256=plan["calibration_sha256"], health_review={"status": "PASS"},
            atlas_sha256=maps.digest(old_reference))))

        def prepare_omni():
            for input_name, run_name in (("orb_trial.yaml", "settings.yaml"), ("timing.json", "timing.json"),
                                         ("camera_profile.json", "camera_profile.json")):
                (run / run_name).write_bytes((params / input_name).read_bytes())
            with redirect_stdout(io.StringIO()):
                return maps.prepare(self.root, run)

        omni = prepare_omni()
        self.assertEqual(Path(omni["reference"]), self.root / "maps/workspace_omni_60.osa")
        self.assertEqual(omni["load"], "")
        with patch.dict(os.environ, VIO_MAP_FILE=str(old_reference)):
            with self.assertRaisesRegex(ValueError, "MAP_E_CALIBRATION"):
                prepare_omni()

    def test_trial_passes_and_numeric_hash_layout_checks_still_apply(self):
        _, clock = trial_snapshot(self.root)
        _, layout = profile_api.load_snapshot(self.root)
        self.assertEqual(layout["calibration_status"], TRIAL)
        self.assertFalse(layout["calibration_validated_for_current_rig"])
        for replacement in (dict(clock, camera_profile_sha256="0" * 64),
                            dict(clock, timeshift_cam_imu_seconds=float("nan")),
                            dict(clock, timeshift_cam_imu_rounded_ns=99)):
            (self.root / "timing.json").write_text(json.dumps(replacement))
            with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                profile_api.load_snapshot(self.root)
        for before, after in (("Camera.width: 8", "Camera.width: 10"),
                              ("Camera1.fx: 4.0", "Camera1.fx: nan"),
                              ("IMU.NoiseAcc: 0.002", "IMU.NoiseAcc: REQUIRED"),
                              ("data: [1,0,0,0", "data: [nan,0,0,0"),
                              ("0,0,0,1]", "0,0,0,0]")):
            trial_snapshot(self.root)
            cfg = self.root / "settings.yaml"
            cfg.write_text(cfg.read_text().replace(before, after))
            with self.subTest(before=before), self.assertRaises(ValueError):
                profile_api.load_snapshot(self.root)

    def test_mixed_trial_and_validated_status_rejected(self):
        _, clock = trial_snapshot(self.root)
        clock["calibration_status"] = "validated"
        (self.root / "timing.json").write_text(json.dumps(clock))
        with self.assertRaisesRegex(ValueError, "status conflicts"):
            profile_api.load_snapshot(self.root)
        _, clock = write_snapshot(self.root)
        clock["calibration_status"] = TRIAL
        (self.root / "timing.json").write_text(json.dumps(clock))
        with self.assertRaisesRegex(ValueError, "matching Camera.calibrationStatus"):
            profile_api.load_snapshot(self.root)

    def test_status_is_fingerprint_bound_and_references_cannot_cross(self):
        trial_snapshot(self.root)
        cfg, timing = self.root / "settings.yaml", self.root / "timing.json"
        trial_fp, payload = maps.calibration(cfg, timing)
        self.assertEqual(payload["calibration_status"], TRIAL)
        self.assertFalse(payload["calibration_validated_for_current_rig"])
        write_snapshot(self.root)
        validated_fp, validated = maps.calibration(cfg, timing)
        self.assertNotEqual(trial_fp, validated_fp)
        self.assertEqual(validated["calibration_status"], "validated")
        for fingerprint, other in ((trial_fp, validated_fp), (validated_fp, trial_fp)):
            reference = self.root / "reference.osa"
            reference.write_bytes(b"SYNTHETIC ATLAS FILE FOR HELPER TEST")
            metadata = dict(schema=maps.SCHEMA, publication_policy=maps.PUBLICATION_POLICY,
                            calibration_sha256=fingerprint, health_review={"status": "PASS"},
                            atlas_sha256=maps.digest(reference))
            maps.sidecar(reference).write_text(json.dumps(metadata))
            saved_bytes = reference.read_bytes(), maps.sidecar(reference).read_bytes()
            maps.validate_reference(reference, fingerprint)
            with self.assertRaisesRegex(ValueError, "MAP_E_CALIBRATION"):
                maps.validate_reference(reference, other)
            self.assertEqual(saved_bytes, (reference.read_bytes(), maps.sidecar(reference).read_bytes()))

    def test_default_map_namespace_and_trial_restore(self):
        run = self.root / "run"
        trial_snapshot(run)
        with redirect_stdout(io.StringIO()):
            plan = maps.prepare(self.root, run)
        reference = self.root / "maps/workspace_trial.osa"
        self.assertEqual(Path(plan["reference"]), reference)
        self.assertEqual(plan["calibration_status"], TRIAL)
        self.assertFalse(plan["calibration_validated_for_current_rig"])
        reference.write_bytes(b"SYNTHETIC TRIAL MAP")
        maps.sidecar(reference).write_text(json.dumps(dict(
            schema=maps.SCHEMA, publication_policy=maps.PUBLICATION_POLICY,
            calibration_sha256=plan["calibration_sha256"], health_review={"status": "PASS"},
            atlas_sha256=maps.digest(reference))))
        trial_snapshot(run)
        with redirect_stdout(io.StringIO()):
            restored = maps.prepare(self.root, run)
        self.assertEqual(restored["load"], str(reference))
        write_snapshot(run)
        with redirect_stdout(io.StringIO()):
            validated_plan = maps.prepare(self.root, run)
        self.assertEqual(Path(validated_plan["reference"]), self.root / "maps/workspace_v2.osa")
        self.assertEqual(validated_plan["load"], "")
        self.assertEqual(reference.read_bytes(), b"SYNTHETIC TRIAL MAP")

    def test_trial_publication_remains_explicitly_unvalidated(self):
        run = self.root / "run"
        trial_snapshot(run)
        session = run / "session"
        session.mkdir()
        with redirect_stdout(io.StringIO()):
            plan = maps.prepare(self.root, run)
        saved = Path(plan["save"])
        saved.write_bytes(b"SYNTHETIC MATURE ATLAS")
        (session / "map_save.json").write_text(json.dumps({"status": "SAVED", "path": str(saved)}))
        with patch.object(maps, "review_health", return_value={"status": "PASS"}), redirect_stdout(io.StringIO()):
            report = maps.finalize(session, 0)
        self.assertEqual(report["status"], "REFERENCE_PUBLISHED")
        for data in (report, json.loads(maps.sidecar(saved).read_text()),
                     json.loads(maps.sidecar(Path(plan["reference"])).read_text())):
            self.assertEqual(data["calibration_status"], TRIAL)
            self.assertFalse(data["calibration_validated_for_current_rig"])

    def test_config_cli_reports_trial_before_camera_profile(self):
        trial_snapshot(self.root)
        output, error = io.StringIO(), io.StringIO()
        argv = ["camera_config.py", "validate", str(self.root / "camera_profile.json"),
                str(self.root / "settings.yaml"), str(self.root / "timing.json")]
        with patch.object(sys, "argv", argv), redirect_stdout(output), redirect_stderr(error):
            code = config.main()
        self.assertEqual(code, 0)
        self.assertEqual(error.getvalue(), "")
        self.assertEqual(output.getvalue().count("Trial calibration:"), 1)
        self.assertTrue(output.getvalue().startswith("Trial calibration:"))
        self.assertIn("Camera profile:", output.getvalue())
        self.assertIn("not measured", output.getvalue())


if __name__ == "__main__":
    unittest.main()
