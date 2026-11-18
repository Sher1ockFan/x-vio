#!/usr/bin/env python3
"""Map lifecycle tests: fingerprints, isolation, clean exit, and no replacement."""
import importlib.util
import csv
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile
from unittest.mock import patch
from test_camera_profile import write_snapshot

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("map_session", ROOT / "jetson/map_session.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def expect_error(fn, message):
    try:
        fn()
    except ValueError as error:
        assert message in str(error), str(error)
    else:
        raise AssertionError("Expected " + message)


with tempfile.TemporaryDirectory(prefix="map-session-test-") as tmp:
    root = Path(tmp).resolve()
    count = 0

    def run(mode="auto", reference=None):
        global count
        count += 1
        folder = root / "runs" / ("live_" + str(count))
        folder.mkdir(parents=True)
        write_snapshot(folder)
        environment = {"VIO_MAP_MODE": mode, "VIO_MAP_FILE": str(reference or root / "maps/workspace_v2.osa")}
        with patch.dict(os.environ, environment):
            plan = module.prepare(root, folder)
        session = folder / "session"
        session.mkdir()
        return folder, session, plan

    def write_diagnostics(session, rows=None):
        row = {"frame_idx": "0", "camera_ns": "1000000000", "health_evaluated": "1",
               "health_accepted": "1", "hard_fault": "0", "dynamics_available": "1", "valid": "1",
               "health_reason": "accepted"}
        for prefix, suffix in (("velocity_w_", "_mps"), ("gyro_bias_", "_radps"),
                               ("accel_bias_", "_mps2")):
            for axis in "xyz":
                row[prefix + axis + suffix] = "0.0"
        with (session / "state_diagnostics.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=row)
            writer.writeheader()
            writer.writerows(rows if rows is not None else [row])
        return row

    def core_saved(session, payload=b"guarded mature Atlas test fixture"):
        (session / "atlas.osa").write_bytes(payload)
        module.write_json(session / "map_save.json", {
            "status": "SAVED", "path": str(session / "atlas.osa"), "selected_map_id": 7, "keyframes": 40,
        })
        write_diagnostics(session)

    # The default must not load, overwrite or migrate a previous v1 map. Exercise
    # the real default with VIO_MAP_FILE absent instead of only explicit paths.
    legacy = root / "maps/workspace.osa"
    legacy.parent.mkdir()
    legacy.write_bytes(b"preserve previous reference for diagnosis")
    default_run = root / "runs/live_default"
    default_run.mkdir(parents=True)
    write_snapshot(default_run)
    default_env = dict(os.environ)
    default_env.pop("VIO_MAP_FILE", None)
    default_env["VIO_MAP_MODE"] = "auto"
    with patch.dict(os.environ, default_env, clear=True):
        default_plan = module.prepare(root, default_run)
    assert default_plan["reference"] == str(root / "maps/workspace_v2.osa")
    assert default_plan["load"] == ""
    assert legacy.read_bytes() == b"preserve previous reference for diagnosis"

    folder, session, plan = run()
    reference = Path(plan["reference"])
    assert plan["load"] == "" and not reference.exists()
    assert "MapReuse.Enabled: 1" in (folder / "settings.yaml").read_text()
    assert "MapReuse." not in (ROOT / "params/orb_trial.template.yaml").read_text()
    # First clean mature save becomes the reference with a timing/calibration guard.
    core_saved(session)
    assert module.finalize(session, 0)["status"] == "REFERENCE_PUBLISHED"
    initial_digest = module.digest(reference)
    module.validate_reference(reference, plan["calibration_sha256"])

    folder2, session2, plan2 = run()
    assert plan2["load"] == str(reference)
    core_saved(session2, b"second session Atlas must not overwrite reference")
    assert module.finalize(session2, 0)["status"] == "EXISTING_REFERENCE_PRESERVED"
    assert module.digest(reference) == initial_digest
    expect_error(lambda: run("new"), "MAP_E_EXISTS")

    # A finite-looking but unhealthy pose, a missing/truncated log, or a hard
    # health fault must never turn a diagnostic Atlas into a new reference.
    for case in ("hard_fault", "missing", "empty", "truncated", "unverified_valid",
                 "nonfinite_velocity", "duplicate_frame", "invalid_boolean"):
        _, bad, bad_plan = run(reference=root / ("maps/" + case + ".osa"))
        core_saved(bad)
        row = write_diagnostics(bad)
        path = bad / "state_diagnostics.csv"
        if case == "hard_fault":
            first = dict(row, health_accepted="0", valid="0", hard_fault="1",
                         health_reason='pose jump; "diagnostic", not accepted')
            good = dict(row, frame_idx="1", camera_ns="1100000000")
            write_diagnostics(bad, [first, good])
        elif case == "missing":
            path.unlink()
        elif case == "empty":
            write_diagnostics(bad, [])
        elif case == "truncated":
            path.write_text(path.read_text() + "1,1100000000\n")
        elif case == "unverified_valid":
            write_diagnostics(bad, [dict(row, health_evaluated="0")])
        elif case == "nonfinite_velocity":
            write_diagnostics(bad, [dict(row, velocity_w_x_mps="nan")])
        elif case == "duplicate_frame":
            write_diagnostics(bad, [row, dict(row, camera_ns="1100000000")])
        elif case == "invalid_boolean":
            write_diagnostics(bad, [dict(row, hard_fault="false")])
        result = module.finalize(bad, 0)
        assert result["status"] == "HEALTH_REVIEW_REQUIRED", (case, result)
        assert not Path(bad_plan["reference"]).exists(), case
        assert (bad / "atlas.osa").is_file(), case
        assert module.sidecar(bad / "atlas.osa").is_file(), case
        expect_error(lambda: module.validate_reference(bad / "atlas.osa", bad_plan["calibration_sha256"]),
                     "MAP_E_HEALTH")
        if case == "hard_fault":
            assert result["health_review"]["hard_fault_frames"] == 1
            assert result["health_review"]["first_faults"][0]["reason"] == first["health_reason"]

    # Ordinary searching/occlusion frames are not hard numerical failures and
    # do not quarantine a session that also has healthy inertial poses.
    _, ordinary, ordinary_plan = run(reference=root / "maps/ordinary_loss.osa")
    core_saved(ordinary)
    row = write_diagnostics(ordinary)
    unknown = dict(row, health_evaluated="0", health_accepted="0", valid="0", health_reason="searching")
    for name in list(unknown):
        if name.startswith(("velocity_w_", "gyro_bias_", "accel_bias_")):
            unknown[name] = "nan"
    write_diagnostics(ordinary, [unknown, dict(row, frame_idx="1", camera_ns="1100000000")])
    assert module.finalize(ordinary, 0)["status"] == "REFERENCE_PUBLISHED"

    # A fault after loading a good reference still preserves its bytes.
    _, fault_on_existing, _ = run()
    core_saved(fault_on_existing)
    row = write_diagnostics(fault_on_existing)
    write_diagnostics(fault_on_existing, [dict(row, valid="0", health_accepted="0", hard_fault="1")])
    assert module.finalize(fault_on_existing, 0)["status"] == "HEALTH_REVIEW_REQUIRED"
    assert module.digest(reference) == initial_digest

    # Display/extractor settings do not invalidate maps; physical calibration does.
    cfg = folder2 / "settings.yaml"
    before = module.calibration(cfg, folder2 / "timing.json")[0]
    cfg.write_text(cfg.read_text() + "\nViewer.ViewpointF: 500.0\nORBextractor.nFeatures: 1500\n")
    cfg.write_text(cfg.read_text().replace("Viewer.ViewpointF: 500.0", "Viewer.ViewpointF: 600.0")
                   .replace("ORBextractor.nFeatures: 1500", "ORBextractor.nFeatures: 2000"))
    assert module.calibration(cfg, folder2 / "timing.json")[0] == before
    cfg.write_text(cfg.read_text().replace("Camera1.fx: 4.0", "Camera1.fx: 4.4"))
    changed = module.calibration(cfg, folder2 / "timing.json")[0]
    expect_error(lambda: module.validate_reference(reference, changed), "MAP_E_CALIBRATION")
    clock = folder2 / "timing.json"
    values = json.loads(clock.read_text())
    values["timeshift_cam_imu_rounded_ns"] += 1000
    values["timeshift_cam_imu_seconds"] += .000001
    clock.write_text(json.dumps(values))
    assert module.calibration(cfg, clock)[0] != changed

    # Failed/incomplete sessions can retain diagnostics but never publish.
    _, failed, failed_plan = run(reference=root / "maps/failed.osa")
    core_saved(failed)
    assert module.finalize(failed, 1)["status"] == "NOT_PUBLISHED"
    assert not Path(failed_plan["reference"]).exists()
    assert module.sidecar(failed / "atlas.osa").is_file()
    _, skipped, skipped_plan = run(reference=root / "maps/immature.osa")
    module.write_json(skipped / "map_save.json", {"status": "SKIPPED", "reason": "No mature map"})
    assert module.finalize(skipped, 0)["status"] == "NO_MATURE_MAP"
    assert not Path(skipped_plan["reference"]).exists()
    _, missing, _ = run(reference=root / "maps/missing.osa")
    expect_error(lambda: module.finalize(missing, 0), "MAP_E_SAVE")
    assert json.loads((missing / "map_session.json").read_text())["status"] == "ERROR"

    # Disabled sessions do not read even an invalid reference or create save paths.
    off_ref = root / "off/invalid.osa"
    off_ref.parent.mkdir()
    off_ref.write_bytes(b"invalid")
    _, off, off_plan = run("off", off_ref)
    assert off_plan["load"] == off_plan["save"] == ""
    assert module.finalize(off, 0)["status"] == "DISABLED"
    assert not (off / "map_save.json").exists()

    # Failure after map commit cannot remove or replace the valid committed pair.
    _, post_failure, post_plan = run(reference=root / "maps/post_failure.osa")
    core_saved(post_failure)
    original_fsync = os.fsync
    def fail_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            raise OSError("simulated directory sync failure after commit")
        return original_fsync(fd)
    with patch.object(module.os, "fsync", side_effect=fail_directory_sync):
        try:
            module.finalize(post_failure, 0)
        except OSError as error:
            assert "after commit" in str(error)
        else:
            raise AssertionError("Expected simulated post-commit error")
    post_reference = Path(post_plan["reference"])
    module.validate_reference(post_reference, post_plan["calibration_sha256"])
    preserved_digest = module.digest(post_reference)
    _, next_session, _ = run(reference=post_reference)
    core_saved(next_session, b"different later map")
    assert module.finalize(next_session, 0)["status"] == "EXISTING_REFERENCE_PRESERVED"
    assert module.digest(post_reference) == preserved_digest

    # A stale half-published pair is rejected, never silently treated as fresh.
    orphan = root / "maps/orphan.osa"
    module.sidecar(orphan).write_text('{}')
    expect_error(lambda: run(reference=orphan), "MAP_E_MANIFEST")

    # Native legacy file keys must not coexist with generated lifecycle keys.
    for legacy in ("System.LoadAtlasFromFile", "System.SaveAtlasToFile"):
        legacy_run = root / legacy
        legacy_run.mkdir()
        write_snapshot(legacy_run)
        text = (legacy_run / "settings.yaml").read_text()
        (legacy_run / "settings.yaml").write_text(text + '\n' + legacy + ': "old"\n')
        with patch.dict(os.environ, {"VIO_MAP_MODE": "off"}):
            expect_error(lambda: module.prepare(root, legacy_run), "MAP_E_CONFIG")
    # Corruption and absent manifests fail before native tracking starts.
    reference.write_bytes(b"changed")
    expect_error(lambda: run(), "MAP_E_CHECKSUM")
    module.sidecar(reference).unlink()
    expect_error(lambda: run(), "MAP_E_MANIFEST")

print("MAP_SESSION_TEST_PASS")
