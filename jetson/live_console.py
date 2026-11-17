#!/usr/bin/env python3
"""Keep live startup messages visible and route later output to console.log."""
import os
from collections import deque
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import traceback

LIVE = "LIVE: move the rig with both translation and rotation in a textured scene."
STARTUP_PREFIXES = (
    "/dev/gmsl/B: ",
    "Trial calibration: ",
    "New live output: ",
    "Continuous tracking. ",
    "Loading ORB vocabulary; ",
    "Estimating the IMU timer from ",
    "Map reuse: ",
    "Map load: ",
)


def short_detail(value):
    """Keep one readable diagnostic, not a traceback or terminal escape sequence."""
    value = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", str(value))
    return " ".join("".join(c for c in value if c.isprintable() or c.isspace()).split())[:700]


def log_tail(path, limit=32768):
    try:
        with Path(path).open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - limit))
            return stream.read().decode("utf-8", errors="replace").splitlines()
    except OSError:
        return []


def classify_startup_failure(lines, session=None, error=None):
    """Explain the originating failure; cleanup errors must not replace it."""
    lines = [short_detail(line) for line in lines if str(line).strip()]
    if error is not None:
        lines.append(f"{type(error).__name__}: {short_detail(error)}")
    # Shell preflight and map preparation already know the exact cause.
    for line in reversed(lines):
        match = re.search(r"\b((?:VIO|MAP)_E_[A-Z_]+):\s*(.+)", line)
        if match:
            return match.group(1), match.group(2)

    reason = ""
    if session is not None:
        try:
            report = json.loads((Path(session) / "live.json").read_text())
            if isinstance(report, dict) and isinstance(report.get("error"), str):
                reason = short_detail(report["error"])
        except (OSError, ValueError):
            pass
    if not reason:
        candidates = [line for line in lines if not line.startswith(STARTUP_PREFIXES)
                      and not line.startswith(("Traceback ", 'File "', "Trial report: "))]
        reason = candidates[-1] if candidates else "Worker exited before LIVE without an error description."

    # Child helpers write their underlying error to a dedicated log. Inspect
    # only the failed component, so an unrelated GUI warning is never blamed.
    component = ""
    if session is not None:
        if "camera helper" in reason.lower() or reason.startswith("read_camera:"):
            component = "camera"
        elif any(token in reason.lower() for token in ("orb exited", "orb startup", "orb closed", "orb input")):
            component = "orb"
        if component:
            tail = log_tail(Path(session) / (component + ".log"))
            for line in reversed(tail):
                match = re.search(r"\b(MAP_E_[A-Z_]+):\s*(.+)", line)
                if match:
                    return match.group(1), short_detail(match.group(2))
            fatal = [line for line in tail if re.search(
                r"VIDIOC_.*:|Invalid camera|unexpected.*(?:format|size)|"
                r"unexpected GMSL identity|^Expected .* GMSL camera|Short MMAP buffer|"
                r"Camera stalled|Unexpected camera clock|Unexpected buffer count|"
                r"No reliable host clock bracket|No gray buffer|"
                r"^(?:inspect camera|set 30 fps|map/queue camera|camera output pipe|write camera timing|camera_timing.csv).*:|"
                r"(?:error|failed|cannot|could not|unable) .*|Permission denied|"
                r"No such (?:file|device)|undefined symbol|what\(\):|"
                r"(?:GLX|X11|Pangolin|Gtk).*?(?:error|failed|cannot)", line, re.I)
                and "canberra-gtk-module" not in line]
            if fatal:
                reason = short_detail(fatal[-1])

    low = reason.lower()
    if "permission denied" in low or "permissionerror" in low or "operation not permitted" in low:
        return "VIO_E_PERMISSION", reason
    if any(x in low for x in ("error while loading shared libraries", "undefined symbol", "modulenotfounderror", "no module named")):
        return "VIO_E_DEPENDENCY", reason
    if any(x in low for x in ("cannot access x11", "unable to open display", "couldn't open display", "cannot open display", "display set", "display unset", "failed to open x display")):
        return "VIO_E_DISPLAY", reason
    if component == "camera":
        return "VIO_E_CAMERA", reason
    if "no such file or directory" in low or "filenotfounderror" in low or low.startswith("missing "):
        return "VIO_E_PATH", reason
    if any(x in low for x in ("read_imu:", "imu timer", "imu packet", "serial device", "serial port", "ahrs", "accelerometer/gyro", "ttyusb")):
        return "VIO_E_IMU", reason
    if component == "orb":
        return "VIO_E_ORB", reason
    if "no space left" in low or "read-only file system" in low or "not a directory" in low:
        return "VIO_E_FILESYSTEM", reason
    if "usage:" in low or "unrecognized arguments" in low:
        return "VIO_E_ARGUMENT", reason
    return "VIO_E_STARTUP", reason


def shell_status(code):
    return 128 - code if code < 0 else code


def process_identity():
    """PID alone is unsafe after reuse or copying a session to another host."""
    def read(path):
        try:
            return Path(path).read_text().strip()
        except OSError:
            return None
    stat = read("/proc/self/stat")
    fields = stat.rsplit(")", 1)[-1].split() if stat else []
    return {"supervisor_pid": os.getpid(),
            "supervisor_start_ticks": fields[19] if len(fields) > 19 else None,
            "boot_id": read("/proc/sys/kernel/random/boot_id"),
            "machine_id": read("/etc/machine-id")}


def supervise(root, arguments):
    ready = False
    cancelled = False
    pending_signal = None
    active = None
    log = None
    session = None
    status = 1
    originals = {}
    worker_lines = deque(maxlen=100)
    startup_path = None
    log_path = None
    startup_error = None
    identity = process_identity()
    started_unix_ns = time.time_ns()

    def note(message):
        if log is not None:
            try:
                log.write(message.encode("utf-8", errors="replace"))
            except OSError:
                pass

    def runtime(state, exit_status=None):
        if session is None:
            return
        temporary = None
        try:
            record = {"schema": 1, **identity, "mode": arguments[0] if arguments else "live",
                      "state": state, "started_unix_ns": started_unix_ns,
                      "updated_unix_ns": time.time_ns(), "exit_status": exit_status}
            fd, temporary = tempfile.mkstemp(prefix=".runtime-", suffix=".json", dir=session)
            with os.fdopen(fd, "w") as stream:
                json.dump(record, stream, sort_keys=True)
                stream.write("\n")
            os.replace(temporary, session / "runtime.json")
            temporary = None
        except OSError as error:
            note(f"Could not write runtime status: {error}\n")
        finally:
            if temporary is not None:
                try:
                    os.unlink(temporary)
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
        log_path = startup_path
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
            if not ready:
                worker_lines.append(text)
            if session is None and text.startswith("New live output: "):
                candidate = Path(text[len("New live output: "):])
                # This is the child-created session under the configured run root.
                if (candidate.is_absolute() and candidate.name == "session" and
                        candidate.resolve().parent.parent == runs and candidate.is_dir()):
                    session = candidate.resolve()
                    runtime("STARTING")
                    try:
                        # Link/unlink preserves the open log and refuses to overwrite a file.
                        os.link(startup_path, session / "console.log")
                        log_path = session / "console.log"
                        startup_path.unlink()
                    except OSError as error:
                        note(f"Could not move startup log into session: {error}\n")
            if not ready:
                if text == LIVE:
                    ready = True
                    runtime("RUNNING")
                    print(text, flush=True)
                elif text.startswith(STARTUP_PREFIXES):
                    print(text, flush=True)
        active.stdout.close()
        status = shell_status(active.wait())
        active = None
        runtime("FINALIZING")
        if not ready and status == 0 and not cancelled:
            note("Live worker exited without announcing its first sensor packet.\n")
            worker_lines.append("Live worker exited without announcing its first sensor packet.")
            status = 1
        if session is not None:
            # The map publisher runs only after the sensor worker and native
            # writer have exited. Its output obeys the same post-LIVE silence.
            # Never promote an Atlas from an interrupted/failed worker.
            if (session.parent / "map_plan.json").is_file():
                finalizer = root / "map_session.py"
                active = subprocess.Popen(
                    [sys.executable, str(finalizer), "finalize", str(session), str(status)],
                    stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
                )
                try:
                    map_status = active.wait(timeout=120)
                except subprocess.TimeoutExpired:
                    note("Map publication exceeded its 120 s limit.\n")
                    stop_active()
                    map_status = 1
                if map_status:
                    note(f"Map publication failed: exit={map_status}.\n")
                    if status == 0:
                        status = shell_status(map_status)
                active = None
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
    except BaseException as error:
        startup_error = error
        note(traceback.format_exc())
        status = 1
    finally:
        stop_active()
        runtime("FINISHED", status)
        for sig, handler in originals.items():
            signal.signal(sig, handler)
        if log is not None:
            log.close()
    if not ready and not cancelled:
        code, detail = classify_startup_failure(worker_lines, session, startup_error)
        if code.startswith("MAP_E_"):
            detail = "地图准备或加载失败；" + detail
        print(f"{code}: {short_detail(detail)}", file=sys.stderr, flush=True)
        if log_path is not None:
            print(f"Log: {log_path}", file=sys.stderr, flush=True)
    return status


if __name__ == "__main__":
    raise SystemExit(supervise(Path(__file__).resolve().parent, sys.argv[1:]))
