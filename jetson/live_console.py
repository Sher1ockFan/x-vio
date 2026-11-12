#!/usr/bin/env python3
"""Keep live startup messages visible and route later output to console.log."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import traceback

LIVE = "LIVE: move the rig with both translation and rotation in a textured scene."
STARTUP_PREFIXES = (
    "/dev/gmsl/B: ",
    "Trial calibration: ",
    "New live output: ",
    "Continuous tracking. ",
    "Loading ORB vocabulary; ",
    "Estimating the IMU timer from ",
)


def shell_status(code):
    return 128 - code if code < 0 else code


def supervise(root, arguments):
    ready = False
    cancelled = False
    pending_signal = None
    active = None
    log = None
    session = None
    status = 1
    originals = {}

    def note(message):
        if log is not None:
            try:
                log.write(message.encode("utf-8", errors="replace"))
            except OSError:
                pass

    def forward(signum, _frame):
        nonlocal cancelled, pending_signal
        cancelled = True
        pending_signal = signum
        if active is not None and active.poll() is None:
            try:
                os.killpg(active.pid, signum)
            except ProcessLookupError:
                pass

    def stop_active():
        if active is None or active.poll() is not None:
            return
        try:
            os.killpg(active.pid, signal.SIGTERM)
            active.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(active.pid, signal.SIGKILL)
            active.wait()
        except ProcessLookupError:
            active.wait()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            originals[sig] = signal.signal(sig, forward)
        root = Path(root).resolve()
        runs = Path(os.environ.get("ORB_HUB_RUNS", str(root / "runs"))).expanduser().resolve()
        runs.mkdir(parents=True, exist_ok=True)
        fd, startup_name = tempfile.mkstemp(prefix="startup_", suffix=".log", dir=runs)
        startup_path = Path(startup_name)
        log = os.fdopen(fd, "wb", buffering=0)
        env = {**os.environ, "VIO_TIMING": "1", "PYTHONDONTWRITEBYTECODE": "1"}
        active = subprocess.Popen(
            ["bash", str(root / "start.sh"), "--live-worker", *arguments],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=env, start_new_session=True,
        )
        if pending_signal is not None:
            forward(pending_signal, None)
        for line in active.stdout:
            log.write(line)
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            if session is None and text.startswith("New live output: "):
                candidate = Path(text[len("New live output: "):])
                # This is the child-created session under the configured run root.
                if (candidate.is_absolute() and candidate.name == "session" and
                        candidate.resolve().parent.parent == runs and candidate.is_dir()):
                    session = candidate.resolve()
                    try:
                        # Link/unlink preserves the open log and refuses to overwrite a file.
                        os.link(startup_path, session / "console.log")
                        startup_path.unlink()
                    except OSError as error:
                        note(f"Could not move startup log into session: {error}\n")
            if not ready:
                if text == LIVE:
                    ready = True
                    print(text, flush=True)
                elif text.startswith(STARTUP_PREFIXES):
                    print(text, flush=True)
        active.stdout.close()
        status = shell_status(active.wait())
        active = None
        if not ready and status == 0 and not cancelled:
            note("Live worker exited without announcing its first sensor packet.\n")
            status = 1
        if session is not None:
            analyzer = root / "performance.py"
            if not analyzer.is_file():
                note("Performance report could not run: missing performance.py.\n")
                if status == 0:
                    status = 1
            else:
                active = subprocess.Popen(
                    [sys.executable, str(analyzer), str(session)],
                    stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
                )
                try:
                    analysis_status = active.wait(timeout=120)
                except subprocess.TimeoutExpired:
                    note("Performance report exceeded its 120 s limit.\n")
                    stop_active()
                    analysis_status = 1
                if analysis_status:
                    note(f"Performance report failed: exit={analysis_status}.\n")
                    if status == 0:
                        status = shell_status(analysis_status)
                active = None
    except KeyboardInterrupt:
        cancelled = True
        note(traceback.format_exc())
        status = 130
    except BaseException:
        note(traceback.format_exc())
        status = 1
    finally:
        stop_active()
        for sig, handler in originals.items():
            signal.signal(sig, handler)
        if log is not None:
            log.close()
    if not ready and not cancelled:
        print("VIO_E_STARTUP", file=sys.stderr, flush=True)
    return status


if __name__ == "__main__":
    raise SystemExit(supervise(Path(__file__).resolve().parent, sys.argv[1:]))
