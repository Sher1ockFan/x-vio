"""Synthetic profile/packet tests; fixture calibration is not hardware calibration.

Run with a Python interpreter providing NumPy (the recorder runtime dependency).
No sensors, settings installation, maps or external processes are used.
"""
from contextlib import redirect_stdout
import copy
import csv
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import struct
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "adapter/orb_replay"))
import camera_profile as profile_api
import live
import umi
from umi_quality import FRAME_FIELDS, file_sha256, load_frames


def source_module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


exporter = source_module("profile_exporter", "jetson/export_euroc.py")
playback = source_module("profile_playback", "jetson/playback.py")
maps = source_module("profile_maps", "jetson/map_session.py")


def test_profile():
    return dict(schema=1, camera_id="synthetic-camera-only", rig_id="synthetic-rig-only",
                device="/dev/video9", expected_card="synthetic capture", expected_bus=None,
                raw_width=8, raw_height=7, crop_top=1, crop_bottom=2, requested_fps=60,
                pixel_format="UYVY", timestamp_domain="raw")


def write_snapshot(folder, profile=None):
    profile = test_profile() if profile is None else profile
    folder.mkdir(exist_ok=True)
    (folder / "camera_profile.json").write_text(json.dumps(profile))
    width, height = profile_api.output_size(profile)
    # These tiny values test parsing and framing only; no VIO is executed.
    (folder / "settings.yaml").write_text(f'''%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera.width: {width}
Camera.height: {height}
Camera.fps: {profile["requested_fps"]}
Camera1.fx: 4.0
Camera1.fy: 4.0
Camera1.cx: 3.5
Camera1.cy: 1.5
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
IMU.Frequency: 500.0
IMU.NoiseGyro: 0.001
IMU.NoiseAcc: 0.002
IMU.GyroWalk: 0.00001
IMU.AccWalk: 0.00002
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
''')
    clock = dict(calibration_status="validated", camera_profile_sha256=profile_api.profile_digest(profile),
                 timeshift_cam_imu_seconds=0.001, timeshift_cam_imu_rounded_ns=1_000_000,
                 time_convention="synthetic test convention", camera_mapping="synthetic raw clock",
                 imu_mapping="synthetic raw clock")
    (folder / "timing.json").write_text(json.dumps(clock))
    return profile, clock


def write_take(folder):
    profile, _ = write_snapshot(folder)
    _, layout = profile_api.load_snapshot(folder)
    image = bytes(range(layout["pixels"]))
    previous_imu = 0
    rows, packets = [], []
    offset = 0
    for i in range(5):
        stamp = 1_001_000_000 + i * 10_000_000
        ticks = [stamp - 1_000_000] if not i else list(range(previous_imu + 2_000_000, stamp, 2_000_000))
        samples = [(tick, (0, 0, 9.8, 0, 0, 0)) for tick in ticks]
        packet = live.packet_header(stamp, samples) + image
        rows.append((i, i, stamp - layout["camera_shift_ns"], stamp, len(samples), offset, len(packet)))
        packets.append(packet)
        offset += len(packet)
        previous_imu = ticks[-1]
    (folder / "stream.bin").write_bytes(b"".join(packets))
    with (folder / "frames.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(FRAME_FIELDS)
        writer.writerows(rows)
    cap = dict(status="COMPLETE", image_format="mono8", width=layout["width"], height=layout["height"],
               camera_profile_sha256=layout["profile_sha256"], camera_shift_ns=layout["camera_shift_ns"],
               clock="CLOCK_MONOTONIC_RAW", frames=len(rows), driver_missing=0,
               imu_samples=sum(row[4] for row in rows),
               sha256={name: file_sha256(folder / name) for name in
                       ("camera_profile.json", "settings.yaml", "timing.json", "frames.csv", "stream.bin")})
    (folder / "capture.json").write_text(json.dumps(cap))
    return cap, image


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mono60-profile-")
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def test_layout_and_stable_digest(self):
        profile = test_profile()
        self.assertEqual(profile_api.output_size(profile), (8, 4))
        original = profile_api.profile_digest(profile)
        self.assertEqual(original, profile_api.profile_digest(dict(profile, device="/dev/video0", description="note")))
        for key, value in (("camera_id", "other"), ("rig_id", "other"), ("expected_card", "other"),
                           ("raw_width", 10), ("raw_height", 8), ("crop_top", 2),
                           ("crop_bottom", 1), ("requested_fps", 30), ("timestamp_domain", "monotonic")):
            with self.subTest(key=key):
                self.assertNotEqual(original, profile_api.profile_digest(dict(profile, **{key: value})))
        for change in (dict(crop_top=-1), dict(crop_bottom=7), dict(raw_width=7),
                       dict(requested_fps=True), dict(timestamp_domain="guess"), dict(pixel_format="YUYV")):
            with self.subTest(change=change), self.assertRaises(ValueError):
                profile_api.output_size(dict(profile, **change))

    def test_camera_environment_is_snapshot_bound(self):
        with patch.dict(os.environ, {"VIO_CAMERA_BUS": "wrong", "VIO_CAMERA_WIDTH": "1920", "VIO_TIMING": "1"}):
            env = profile_api.camera_env(test_profile())
        self.assertEqual(env["VIO_CAMERA_WIDTH"], "8")
        self.assertEqual(env["VIO_CAMERA_HEIGHT"], "7")
        self.assertEqual(env["VIO_CAMERA_CROP_TOP"], "1")
        self.assertEqual(env["VIO_CAMERA_CROP_BOTTOM"], "2")
        self.assertEqual(env["VIO_CAMERA_FPS"], "60")
        self.assertEqual(env["VIO_CAMERA_CLOCK"], "raw")
        self.assertEqual(env["VIO_TIMING"], "1")
        self.assertNotIn("VIO_CAMERA_BUS", env)
        with self.assertRaisesRegex(ValueError, "unverified"):
            profile_api.camera_env(dict(test_profile(), timestamp_domain="unverified"))

    def test_calibration_is_required_and_bound_to_geometry(self):
        profile, clock = write_snapshot(self.root)
        _, layout = profile_api.load_snapshot(self.root)
        self.assertEqual((layout["width"], layout["height"], layout["pixels"]), (8, 4, 32))
        self.assertEqual(live.camera_shift(self.root / "timing.json"), 1_000_000)
        for changed in (dict(clock, calibration_status="template"),
                        dict(clock, camera_profile_sha256="0" * 64),
                        dict(clock, timeshift_cam_imu_rounded_ns=0),
                        dict(clock, timeshift_cam_imu_seconds=float("nan"))):
            (self.root / "timing.json").write_text(json.dumps(changed))
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                profile_api.load_snapshot(self.root)
        write_snapshot(self.root, profile)
        for key, value in (("Camera.width: 8", "Camera.width: 10"),
                           ("Camera.height: 4", "Camera.height: 7"),
                           ("Camera.fps: 60", "Camera.fps: 30"),
                           ("Camera1.fx: 4.0", "Camera1.fx: nan"),
                           ("IMU.Frequency: 500.0", "IMU.Frequency: 1000.0")):
            write_snapshot(self.root, profile)
            cfg = self.root / "settings.yaml"
            cfg.write_text(cfg.read_text().replace(key, value))
            with self.subTest(key=key), self.assertRaises(ValueError):
                profile_api.load_snapshot(self.root)
        write_snapshot(self.root, dict(profile, timestamp_domain="unverified"))
        with self.assertRaisesRegex(ValueError, "unverified"):
            profile_api.load_snapshot(self.root)
        (self.root / "camera_profile.json").unlink()
        with self.assertRaisesRegex(ValueError, "camera profile"):
            profile_api.load_snapshot(self.root)

    def test_actual_camera_reader_uses_snapshot_byte_count(self):
        write_snapshot(self.root)
        _, layout = profile_api.load_snapshot(self.root)
        image = bytes(range(layout["pixels"]))
        feeds = live.Streams(layout["pixels"])
        with self.assertRaises(EOFError):
            live.read_camera(feeds, io.BytesIO(struct.pack("<q", 123) + image))
        self.assertEqual(feeds.latest, (123, image))
        with self.assertRaisesRegex(EOFError, "Incomplete grayscale"):
            live.read_camera(live.Streams(layout["pixels"]), io.BytesIO(struct.pack("<q", 123) + image[:-1]))
        with self.assertRaisesRegex(ValueError, "dimensions"):
            live.read_camera(live.Streams(), io.BytesIO())

    def test_take_integrity_and_dynamic_export(self):
        cap, image = write_take(self.root)
        loaded, frames = load_frames(self.root)
        self.assertEqual(loaded, cap)
        self.assertEqual(umi.verified_stream(self.root / "stream.bin", frames, len(image)), cap["sha256"]["stream.bin"])
        destination = self.root / "export"
        exporter.export_take(self.root, destination)
        export = json.loads((destination / "export.json").read_text())
        self.assertEqual(export["image_size"], [8, 4])
        self.assertEqual(export["exported_frames"], 4)
        self.assertEqual((destination / "camera_profile.json").read_bytes(), (self.root / "camera_profile.json").read_bytes())
        png = next((destination / "mav0/cam0/data").glob("*.png")).read_bytes()
        self.assertEqual(struct.unpack(">II", png[16:24]), (8, 4))
        position, payload = 8, b""
        while position < len(png):
            size, = struct.unpack(">I", png[position:position+4])
            if png[position+4:position+8] == b"IDAT":
                payload += png[position+8:position+8+size]
            position += 12 + size
        self.assertEqual(zlib.decompress(payload), b"".join(b"\0" + image[i:i+8] for i in range(0, len(image), 8)))
        # Even a relocation-only edit preserves the stable calibration binding,
        # but mutating a sealed profile must fail its recording checksum.
        profile = json.loads((self.root / "camera_profile.json").read_text())
        (self.root / "camera_profile.json").write_text(json.dumps(dict(profile, device="/dev/video0")))
        with self.assertRaisesRegex(ValueError, "recorded checksum"):
            load_frames(self.root)

    def test_recovery_uses_saved_layout(self):
        cap, _ = write_take(self.root)
        cap.update(status="INCOMPLETE", error="KeyboardInterrupt")
        (self.root / "capture.json").write_text(json.dumps(cap))
        umi.recover(self.root)
        recovered, frames = load_frames(self.root)
        self.assertEqual(recovered["status"], "RECOVERED")
        self.assertEqual(len(frames), 5)

    def test_calibration_gate_precedes_process_launch(self):
        write_take(self.root)
        clock = json.loads((self.root / "timing.json").read_text())
        (self.root / "timing.json").write_text(json.dumps(dict(clock, calibration_status="unvalidated")))
        with patch("subprocess.Popen") as process:
            for invoke in (lambda: live.run(SimpleNamespace(yaml=self.root / "settings.yaml",
                                                               timing=self.root / "timing.json"),
                                            self.root / "session", 1_000_000),
                           lambda: umi.record(self.root, Path("not-run")),
                           lambda: umi.solve(self.root, Path("not-run"), Path("not-run"), False),
                           lambda: playback.replay(self.root, Path("not-run"), Path("not-run")),
                           lambda: playback.record_with_preview(self.root, Path("not-run"), Path("not-run")),
                           lambda: exporter.export_take(self.root, self.root / "export")):
                with self.assertRaisesRegex(ValueError, "calibration is required"):
                    invoke()
            process.assert_not_called()

    def test_record_view_uses_same_capture_and_does_not_stop_on_preview_exit(self):
        profile = dict(test_profile(), raw_width=1920, raw_height=1536, crop_top=0, crop_bottom=0)
        original_handler = signal.getsignal(signal.SIGINT)
        for reason in ("closed", "failed", "unavailable"):
            with self.subTest(reason=reason):
                take = self.root / reason
                write_snapshot(take, profile)
                viewer, camera = Path("/camera_preview"), Path("/camera_umi")
                recorder = Mock()
                recorder.poll.return_value = None
                recorder.wait.return_value = 0
                window = Mock()
                window.returncode = 0 if reason == "closed" else 1
                window.poll.return_value = window.returncode
                launch_preview = FileNotFoundError("preview executable missing") if reason == "unavailable" else window

                def stop_recording(_seconds):
                    recorder.send_signal.assert_not_called()
                    signal.raise_signal(signal.SIGINT)

                output = io.StringIO()
                with patch("subprocess.Popen", side_effect=[recorder, launch_preview]) as spawn, \
                        patch.object(playback.time, "sleep", side_effect=stop_recording), \
                        patch.object(playback, "open_viewer") as orb_viewer, redirect_stdout(output):
                    playback.record_with_preview(take, viewer, camera)

                self.assertEqual([call.args[0] for call in spawn.call_args_list], [
                    [sys.executable, "-u", playback.TOOLS / "umi.py", "record", take, camera],
                    [viewer, take, "1920", "1536"],
                ])
                self.assertTrue(all(call.kwargs["start_new_session"] for call in spawn.call_args_list))
                orb_viewer.assert_not_called()
                recorder.send_signal.assert_called_once_with(signal.SIGINT)
                recorder.wait.assert_called_once()
                self.assertIs(signal.getsignal(signal.SIGINT), original_handler)
                self.assertIn("Recording continues", output.getvalue())
                self.assertIn("Recording complete", output.getvalue())
                self.assertEqual(profile_api.load_profile(take / "camera_profile.json")["requested_fps"], 60)

    def test_map_identity_is_bound_and_off_mode_does_not_bypass_calibration(self):
        profile, _ = write_snapshot(self.root)
        cfg, timing = self.root / "settings.yaml", self.root / "timing.json"
        original, payload = maps.calibration(cfg, timing)
        self.assertIn("camera_profile", payload)
        write_snapshot(self.root, dict(profile, device="/dev/video0"))
        self.assertEqual(original, maps.calibration(cfg, timing)[0])
        write_snapshot(self.root, dict(profile, rig_id="other-rig"))
        self.assertNotEqual(original, maps.calibration(cfg, timing)[0])
        clock = json.loads(timing.read_text())
        timing.write_text(json.dumps(dict(clock, calibration_status="unvalidated")))
        with patch.dict(os.environ, {"VIO_MAP_MODE": "off"}), self.assertRaises(ValueError):
            maps.prepare(self.root, self.root)


if __name__ == "__main__":
    unittest.main()
