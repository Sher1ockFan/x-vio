#!/usr/bin/env python3
"""Small subprocess checks for terminal policy, report generation, and signals."""
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
LIVE='LIVE: move the rig with both translation and rotation in a textured scene.'
WORKER=r'''
import os
from pathlib import Path
import signal
import sys
import time
scenario=sys.argv[1]
assert os.environ.get('VIO_TIMING')=='1'
if scenario=='startup_fail':
    print('secret diagnostic startup error',file=sys.stderr,flush=True)
    raise SystemExit(17)
if scenario=='startup_zero':
    raise SystemExit(0)
if scenario=='startup_wait':
    def stop(sig,frame):
        print('startup cancellation details',flush=True)
        raise SystemExit(130)
    signal.signal(signal.SIGINT,stop)
    print('Estimating the IMU timer from 10 s of fresh serial data.',flush=True)
    while True: time.sleep(.01)
root=Path(__file__).resolve().parent
out=Path(os.environ.get('ORB_HUB_RUNS',root/'runs'))/'live_test'/'session'
out.mkdir(parents=True)
print('New live output: '+str(out),flush=True)
print('Loading ORB vocabulary; progress in '+str(out/'orb.log'),flush=True)
print('not a permitted startup status',file=sys.stderr,flush=True)
print('LIVE: move the rig with both translation and rotation in a textured scene.',flush=True)
print('Live 5.0s, sent 123 frames, superseded 4.',flush=True)
print('post-ready stderr secret',file=sys.stderr,flush=True)
if scenario=='runtime_fail':
    print('runtime failure details',flush=True)
    raise SystemExit(9)
if scenario=='signal_fail':
    os.kill(os.getpid(),signal.SIGTERM)
if scenario=='running':
    def stop(sig,frame):
        print('Finishing the current frame and finalizing the session.',flush=True)
        (out/'sealed').write_text('yes')
        print('Trial report: '+str(out/'live.json'),flush=True)
        raise SystemExit(0)
    signal.signal(signal.SIGINT,stop)
    print('worker_signal_ready',flush=True)
    while True: time.sleep(.01)
print('Trial report: '+str(out/'live.json'),flush=True)
'''
ANALYZER=r'''
from pathlib import Path
import os,sys
out=Path(sys.argv[1])
print('report internal stdout')
print('report internal stderr',file=sys.stderr)
if os.environ.get('VIO_TEST_REPORT_FAIL'):
    raise SystemExit(12)
(out/'performance.txt').write_text('measurement report')
'''

def make_root(base,name):
    root=base/name
    root.mkdir()
    shutil.copy2(HERE.parent/'jetson/live_console.py',root/'live_console.py')
    (root/'worker.py').write_text(WORKER)
    (root/'performance.py').write_text(ANALYZER)
    (root/'start.sh').write_text('#!/usr/bin/env bash\nset -eu\n[[ $1 == --live-worker ]]\nshift\n[[ $1 == live ]]\nshift\nexec python3 "$(dirname "$0")/worker.py" "$1"\n')
    return root

def check_output(stdout,stderr,has_ready):
    assert 'secret' not in stdout+stderr,(stdout,stderr)
    if has_ready:
        assert stdout.rstrip().endswith(LIVE),stdout
        assert stdout.count(LIVE)==1
        assert stderr=='',stderr
    else:
        assert LIVE not in stdout

with tempfile.TemporaryDirectory(prefix='vio-console-test-') as temporary:
    base=Path(temporary)
    for scenario,expected,ready in [('startup_fail',17,False),('startup_zero',1,False),('success',0,True),('runtime_fail',9,True),('signal_fail',143,True),('report_fail',12,True)]:
        root=make_root(base,scenario)
        env=dict(os.environ)
        env.pop('ORB_HUB_RUNS',None)
        if scenario=='report_fail':env['VIO_TEST_REPORT_FAIL']='1'
        result=subprocess.run([sys.executable,str(root/'live_console.py'),'live',scenario],capture_output=True,text=True,env=env,timeout=10)
        assert result.returncode==expected,(scenario,result.returncode,result.stdout,result.stderr)
        check_output(result.stdout,result.stderr,ready)
        if ready:
            console=(root/'runs/live_test/session/console.log').read_text()
            assert 'post-ready stderr secret' in console and 'Trial report' in console if scenario in ('success','report_fail') else 'post-ready stderr secret' in console
            assert not list((root/'runs').glob('startup_*.log'))
            if scenario!='report_fail':assert (root/'runs/live_test/session/performance.txt').is_file()
        else:
            assert result.stderr=='VIO_E_STARTUP\n',result.stderr
            assert len(list((root/'runs').glob('startup_*.log')))==1
        print('PASS',scenario)
    for scenario,expected,ready in [('running',0,True),('startup_wait',130,False)]:
        root=make_root(base,scenario)
        env=dict(os.environ)
        env.pop('ORB_HUB_RUNS',None)
        proc=subprocess.Popen([sys.executable,str(root/'live_console.py'),'live',scenario],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=env)
        prefix=''
        deadline=time.monotonic()+8
        marker=LIVE if ready else 'Estimating the IMU timer'
        while marker not in prefix and time.monotonic()<deadline:
            line=proc.stdout.readline()
            assert line,(scenario,prefix,proc.poll())
            prefix+=line
        assert marker in prefix
        if ready:
            log=root/'runs/live_test/session/console.log'
            while 'worker_signal_ready' not in log.read_text() and time.monotonic()<deadline:
                time.sleep(.01)
            assert 'worker_signal_ready' in log.read_text()
        proc.send_signal(signal.SIGINT)
        stdout,stderr=proc.communicate(timeout=10)
        stdout=prefix+stdout
        assert proc.returncode==expected,(scenario,proc.returncode,stdout,stderr)
        check_output(stdout,stderr,ready)
        assert stderr=='',stderr
        if ready:
            out=root/'runs/live_test/session'
            assert (out/'sealed').read_text()=='yes'
            assert (out/'performance.txt').is_file()
            assert 'Finishing the current frame' in (out/'console.log').read_text()
        print('PASS',scenario)
    # Verify the real candidate start.sh still forwards non-live commands as before.
    after=subprocess.check_output(['bash',str(HERE.parent/'jetson/start.sh'),'--help'], text=True)
    for command in ('live', 'record', 'record-view', 'replay', 'trajectory'):
        assert 'start.sh '+command in after
    root=base/'shell_missing_helper';root.mkdir()
    shutil.copy2(HERE.parent/'jetson/start.sh',root/'start.sh')
    result=subprocess.run(['bash',str(root/'start.sh'),'live','0'],capture_output=True,text=True)
    assert result.returncode==1 and result.stdout=='' and result.stderr=='VIO_E_STARTUP\n'
    print('PASS non_live_help_and_missing_helper')
print('LIVE_CONSOLE_TEST_PASS cases=9')
