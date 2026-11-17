#!/usr/bin/env python3
"""Verify real startup parsing without serial, camera, NumPy or a native viewer."""
import ast
from collections import deque
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'adapter/orb_replay/live.py'
tree = ast.parse(source.read_text())
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'wait_ready')
# Compile the actual function, not a second implementation. Its surrounding
# hardware/dependency imports are deliberately excluded from this unit fixture.
code = compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec')


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        self.now += 1
        return self.now

    def sleep(self, delay):
        pass


class Log:
    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.reads = 0

    def __str__(self):
        return '/test/session/orb.log'

    def open(self, mode):
        assert mode == 'rb'
        return self

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self, limit):
        self.reads += 1
        return self.chunks.popleft() if self.chunks else b''


class Child:
    def __init__(self, exit_code=None):
        self.returncode = exit_code

    def poll(self):
        return self.returncode


MARKER = b'MAP_REFERENCE_LOADED id=7 keyframes=123; current velocity/bias still require recovery'
READY = b'ORB_LIVE_READY\n'
parser_cases = 0


def run(name, chunks, successes=0, error=None, exit_code=None):
    global parser_cases
    clock = Clock()
    scope = {'time': clock, 're': re}
    exec(code, scope)
    output = io.StringIO()
    observed = None
    with redirect_stdout(output):
        try:
            scope['wait_ready'](Child(exit_code), Log(chunks))
        except (RuntimeError, TimeoutError) as caught:
            observed = caught
    if error is None:
        assert observed is None, (name, observed)
    else:
        assert isinstance(observed, error), (name, observed, error)
    text = output.getvalue()
    assert text.count('Map load: SUCCESS') == successes, (name, text)
    if successes:
        assert 'id=7' in text and 'keyframes=123' in text, (name, text)
        assert '仍待恢复' in text, (name, text)
    parser_cases += 1
    print('PASS parser', name)


run('real_marker', [MARKER + b'\n' + READY], 1)
run('marker_split_prefix', [MARKER[:13], MARKER[13:] + b'\n', READY], 1)
run('marker_split_after_64_bytes', [MARKER[:70], MARKER[70:] + b'\n', READY], 1)
run('marker_bytewise', [bytes([b]) for b in MARKER + b'\n'] + [READY], 1)
run('marker_waits_for_newline', [MARKER, b'\n', READY], 1)
run('duplicate_deduplicated', [MARKER + b'\n' + MARKER + b'\n' + READY], 1)
run('planned_only', [b'Map reuse: loading /maps/workspace_v2.osa\n' + READY])
run('native_load_attempt_only', [b'Initialization of Atlas from file: /maps/workspace_v2.osa\n' + READY])
run('ready_is_not_map_success', [READY])
run('malformed_markers', [b'prefix ' + MARKER + b'\n' + MARKER.replace(b'id=7', b'id=bad') + b'\n' + READY])
run('overlong_line_suffix_is_not_marker', [b'x' * 5000, MARKER + b'\n', READY])
run('overlong_line_then_real_marker', [b'x' * 5000 + b'\n', MARKER + b'\n' + READY], 1)
run('exit_before_ready', [MARKER + b'\n' + READY], error=RuntimeError, exit_code=3)
run('timeout_incomplete_record', [MARKER], error=TimeoutError)

spec = importlib.util.spec_from_file_location('map_session_under_test', ROOT / 'jetson/map_session.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
prepare_cases = 0
with tempfile.TemporaryDirectory(prefix='vio-map-startup-') as directory:
    root = Path(directory)
    reference = root / 'maps/workspace_v2.osa'

    def prepare(mode, suffix):
        folder = root / 'runs' / suffix
        folder.mkdir(parents=True)
        shutil.copy2(ROOT / 'params/orb_trial.yaml', folder / 'settings.yaml')
        shutil.copy2(ROOT / 'params/timing.json', folder / 'timing.json')
        output = io.StringIO()
        with patch.dict(os.environ, {'VIO_MAP_MODE': mode, 'VIO_MAP_FILE': str(reference)}), redirect_stdout(output):
            plan = module.prepare(root, folder)
        assert 'Map load: SUCCESS' not in output.getvalue(), output.getvalue()
        return plan, output.getvalue()

    plan, text = prepare('auto', 'fresh')
    assert plan['load'] == '' and 'Map load: NO_REFERENCE' in text and str(reference) in text
    prepare_cases += 1
    plan, text = prepare('off', 'off')
    assert plan['load'] == '' and 'Map load: DISABLED' in text
    prepare_cases += 1
    # A correctly guarded synthetic reference lets prepare validate the actual
    # sidecar/checksum path. Preparing it must still never announce load success.
    reference.parent.mkdir(exist_ok=True)
    reference.write_bytes(b'test reference bytes; not a native Atlas')
    module.write_json(module.sidecar(reference), {
        'schema': module.SCHEMA, 'publication_policy': module.PUBLICATION_POLICY,
        'calibration_sha256': plan['calibration_sha256'],
        'atlas_sha256': module.digest(reference), 'health_review': {'status': 'PASS'},
    })
    plan, text = prepare('auto', 'loading')
    assert plan['load'] == str(reference) and 'Map load: LOADING' in text and str(reference) in text
    prepare_cases += 1
    reference.write_bytes(b'changed fixture bytes')
    try:
        prepare('auto', 'corrupt')
    except ValueError as error:
        assert 'MAP_E_CHECKSUM' in str(error), error
    else:
        raise AssertionError('Corrupt reference must fail validation')
    prepare_cases += 1
print(f'MAP_STARTUP_TEST_PASS parser_cases={parser_cases} prepare_cases={prepare_cases}')
