"""Regression checks for the mono+IMU fork's supported input/build contract.

Run from any working directory: python3 -m unittest discover -s tests -v
These checks do not substitute for the native build or a sensor replay test.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "ORB_SLAM3"


def read(relative):
    return (ROOT / relative).read_text()


def cpp_code(text):
    # Strip comments while preserving quoted strings and character literals.
    token = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/'
    return re.sub(token, lambda m: " " if m[0].startswith(("//", "/*")) else m[0], text)


def constructor_args(source, classname):
    """Read the simple factory call used to create a SLAM worker."""
    calls = re.findall(r'\bnew\s+' + re.escape(classname) + r'\s*\(([^;]+)\)\s*;', source)
    if len(calls) != 1:
        raise AssertionError(f"Expected one {classname} worker creation, got {len(calls)}")
    return [re.sub(r'\s+', '', item) for item in calls[0].split(',')]


class MonocularInertialContract(unittest.TestCase):
    def test_only_supported_sensor_is_monocular_inertial(self):
        header = cpp_code(read("ORB_SLAM3/include/System.h"))
        enum = re.search(r'\benum\s+eSensor\s*\{([^}]+)\}', header)
        self.assertIsNotNone(enum)
        entries = [x.strip() for x in enum[1].split(',') if x.strip()]
        self.assertEqual(len(entries), 1)
        self.assertRegex(entries[0], r'^IMU_MONOCULAR\s*=\s*3$')

    def test_no_unsupported_input_api(self):
        for filename in ("System.h", "Tracking.h"):
            code = cpp_code(read("ORB_SLAM3/include/" + filename))
            self.assertNotRegex(code, r'\b(?:Track|GrabImage)(?:Stereo|RGBD)\s*\(')
        header = cpp_code(read("ORB_SLAM3/include/System.h"))
        tracking = re.search(r'\bTrackMonocular\s*\(([^;]+)\)\s*;', header)
        self.assertIsNotNone(tracking)
        self.assertIn("IMU::Point", tracking[1])
        # The public call must explicitly supply an IMU batch, including an
        # intentionally empty batch when no new measurements are available.
        self.assertNotRegex(tracking[1], r'vImuMeas\s*=')

    def test_no_stereo_or_depth_frame_constructors(self):
        header = cpp_code(read("ORB_SLAM3/include/Frame.h"))
        for forbidden in ("imRight", "imDepth", "ComputeStereoMatches",
                          "ComputeStereoFromRGBD", "ComputeStereoFishEyeMatches"):
            self.assertNotRegex(header, r'\b' + forbidden + r'\b')

    def test_only_mono_inertial_dataset_executable(self):
        cmake = re.sub(r'#[^\n]*', '', read("ORB_SLAM3/CMakeLists.txt"))
        targets = re.findall(r'\badd_executable\s*\(\s*([^\s)]+)', cmake, re.I)
        self.assertEqual(targets, ["mono_inertial_euroc"])
        self.assertTrue((CORE / "Examples/Monocular-Inertial/mono_inertial_euroc.cc").is_file())
        self.assertFalse((CORE / "Examples_old").exists())
        forbidden = {"Monocular", "Stereo", "Stereo-Inertial", "RGB-D", "RGB-D-Inertial", "ROS"}
        for directory in (CORE / "Examples").rglob("*"):
            if directory.is_dir():
                self.assertNotIn(directory.name, forbidden)

    def test_mono_inertial_worker_semantics_preserved(self):
        source = cpp_code(read("ORB_SLAM3/src/System.cc"))
        local = constructor_args(source, "LocalMapping")
        loop = constructor_args(source, "LoopClosing")
        # In the original IMU_MONOCULAR mode both mapping flags and the loop
        # closure scale flag evaluate to true. Do not confuse it with MONOCULAR.
        self.assertEqual(local[2:4], ["true", "true"])
        self.assertEqual(loop[3], "true")
        self.assertRegex(source, r'\bSetInertialSensor\s*\(\s*\)')

    def test_both_capture_workers_use_the_same_b_camera(self):
        capture = cpp_code(read("adapter/orb_replay/camera_live.c"))
        devices = re.findall(r'\bconst\s+char\s*\*\s*device\s*=\s*"([^"]+)"', capture)
        self.assertEqual(devices, ["/dev/gmsl/B"])
        self.assertIn('"vi-output, pnd_gmsl_mc 10-003b"', capture)
        self.assertIn('"platform:tegra-capture-vi:0"', capture)
        self.assertNotIn('/dev/video6', capture)
        build = read("docker/package.sh").replace('\\\n', ' ')
        for binary in ("camera_live", "camera_umi"):
            lines = [line for line in build.splitlines() if '-o "$pkg/bin/' + binary + '"' in line]
            self.assertEqual(len(lines), 1, binary)
            self.assertIn('$input/orb_replay/camera_live.c', lines[0])
            self.assertIn('-O3', lines[0])
            self.assertIn('-march=armv8-a', lines[0])

    def test_neon_y_extraction_and_measurements_remain_available(self):
        camera = cpp_code(read("adapter/cam_imu_motion/camera.c"))
        self.assertRegex(camera, r'\bvld2q_u8\s*\(')
        self.assertRegex(camera, r'\bvst1q_u8\s*\(')
        self.assertTrue((ROOT / "jetson/performance.py").is_file())
        for path in ("adapter/orb_replay/camera_live.c", "adapter/viewer.cc",
                     "adapter/orb_replay/live.py", "jetson/live_console.py"):
            self.assertIn('VIO_TIMING', read(path), path)


if __name__ == "__main__":
    unittest.main()
