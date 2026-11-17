#!/usr/bin/env python3
"""Small subprocess checks for terminal policy, report generation, and signals."""
import os
import importlib.util
import json
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
if scenario=='startup_map':
    print('MAP_E_MANIFEST: reference requires both .osa and .osa.meta.json',file=sys.stderr,flush=True)
    raise SystemExit(1)
if scenario=='startup_permission':
    print("read_imu: [Errno 13] Permission denied: '/dev/ttyUSB0'",file=sys.stderr,flush=True)
    raise SystemExit(1)
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
if os.environ.get('VIO_TEST_MAP'):
    (out.parent/'map_plan.json').write_text('{}')
print('New live output: '+str(out),flush=True)
print('Loading ORB vocabulary; progress in '+str(out/'orb.log'),flush=True)
if scenario in ('startup_camera','startup_orb'):
    import json
    if scenario=='startup_camera':
        error='read_camera: Camera helper exited; inspect camera.log.'
        (out/'camera.log').write_text('GMSL device=/dev/gmsl/B raw=1920x1441 UYVY\nVIDIOC_STREAMON: No such device\n')
    else:
        error='ORB exited during initialization (1); inspect orb.log.'
        (out/'orb.log').write_text('Gtk-Message: Failed to load module "canberra-gtk-module"\nMAP_E_LOAD: incompatible Atlas; this build requires mono_imu_vio Atlas v2\nShutdown\n')
    (out/'live.json').write_text(json.dumps({'error':error,'status':'INCOMPLETE'}))
    print('Trial report: '+str(out/'live.json'),flush=True)
    print(error,file=sys.stderr,flush=True)
    raise SystemExit(1)
if scenario.startswith('map_message_'):
    message = {
        'map_message_loaded': 'Map load: SUCCESS id=7 keyframes=123; state recovery pending',
        'map_message_fresh': 'Map load: NO_REFERENCE /maps/workspace_v2.osa; starting new map',
        'map_message_off': 'Map load: DISABLED VIO_MAP_MODE=off',
    }[scenario]
    print(message,flush=True)
print('not a permitted startup status',file=sys.stderr,flush=True)
print('LIVE: move the rig with both translation and rotation in a textured scene.',flush=True)
print('Live 5.0s, sent 123 frames, superseded 4.',flush=True)
print('Map load: SUCCESS id=999 keyframes=999; post-LIVE must be silent',flush=True)
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
MAP_FINALIZER=r'''
from pathlib import Path
import os,sys
assert sys.argv[1]=='finalize'
out=Path(sys.argv[2])
print('map finalizer internal stdout')
print('map finalizer internal stderr',file=sys.stderr)
(out/'finalized_worker_exit').write_text(sys.argv[3])
raise SystemExit(int(os.environ.get('VIO_TEST_MAP_FAIL','0')))
'''

def make_root(base,name):
    root=base/name
    root.mkdir()
    shutil.copy2(HERE.parent/'jetson/live_console.py',root/'live_console.py')
    (root/'worker.py').write_text(WORKER)
    (root/'performance.py').write_text(ANALYZER)
    (root/'map_session.py').write_text(MAP_FINALIZER)
    (root/'start.sh').write_text('#!/usr/bin/env bash\nset -eu\n[[ $1 == --live-worker ]]\nshift\n[[ $1 == live ]]\nshift\nexec python3 "$(dirname "$0")/worker.py" "$1"\n')
    return root

def check_output(stdout,stderr,has_ready):
    if has_ready:
        assert 'secret' not in stdout+stderr,(stdout,stderr)
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
            assert result.stderr.startswith('VIO_E_STARTUP:'),result.stderr
            assert 'Log: ' in result.stderr,result.stderr
            if scenario=='startup_fail':
                assert 'secret diagnostic startup error' in result.stderr,result.stderr
            assert len(list((root/'runs').glob('startup_*.log')))==1
        print('PASS',scenario)
    for scenario,code,detail,session_log in [
        ('startup_map','MAP_E_MANIFEST','requires both .osa and .osa.meta.json',False),
        ('startup_permission','VIO_E_PERMISSION','/dev/ttyUSB0',False),
        ('startup_camera','VIO_E_CAMERA','VIDIOC_STREAMON: No such device',True),
        ('startup_orb','MAP_E_LOAD','incompatible Atlas',True),
    ]:
        root=make_root(base,scenario)
        env=dict(os.environ);env.pop('ORB_HUB_RUNS',None)
        result=subprocess.run([sys.executable,str(root/'live_console.py'),'live',scenario],capture_output=True,text=True,env=env,timeout=10)
        assert result.returncode==1,(scenario,result.returncode,result.stdout,result.stderr)
        check_output(result.stdout,result.stderr,False)
        assert result.stderr.startswith(code+':'),(scenario,result.stderr)
        assert detail in result.stderr,(scenario,result.stderr)
        assert 'Log: ' in result.stderr,(scenario,result.stderr)
        assert 'canberra-gtk-module' not in result.stderr,result.stderr
        if session_log:
            assert str(root/'runs/live_test/session/console.log') in result.stderr,result.stderr
        print('PASS',scenario)
    for scenario,status in [('map_message_loaded','SUCCESS id=7'),('map_message_fresh','NO_REFERENCE'),('map_message_off','DISABLED')]:
        root=make_root(base,scenario)
        env=dict(os.environ);env.pop('ORB_HUB_RUNS',None)
        result=subprocess.run([sys.executable,str(root/'live_console.py'),'live',scenario],capture_output=True,text=True,env=env,timeout=10)
        assert result.returncode==0,(scenario,result)
        check_output(result.stdout,result.stderr,True)
        assert 'Map load: '+status in result.stdout,(scenario,result.stdout)
        assert result.stdout.index('Map load: '+status)<result.stdout.index(LIVE)
        assert 'id=999' not in result.stdout+result.stderr,(scenario,result)
        assert 'id=999' in (root/'runs/live_test/session/console.log').read_text()
        print('PASS',scenario)
    for scenario,worker,expected in [('map_success','success',0),('map_fail','success',14),('map_runtime_failure','runtime_fail',9)]:
        root=make_root(base,scenario)
        env={**os.environ,'VIO_TEST_MAP':'1'}
        env.pop('ORB_HUB_RUNS',None)
        if scenario=='map_fail':env['VIO_TEST_MAP_FAIL']='14'
        result=subprocess.run([sys.executable,str(root/'live_console.py'),'live',worker],capture_output=True,text=True,env=env,timeout=10)
        assert result.returncode==expected,(scenario,result.returncode,result.stdout,result.stderr)
        check_output(result.stdout,result.stderr,True)
        out=root/'runs/live_test/session'
        assert (out/'finalized_worker_exit').read_text()==('9' if worker=='runtime_fail' else '0')
        assert 'map finalizer internal stdout' in (out/'console.log').read_text()
        assert (out/'performance.txt').is_file()
        print('PASS',scenario)
    for scenario,expected,ready in [('running',0,True),('startup_wait',130,False)]:
        root=make_root(base,scenario)
        env=dict(os.environ)
        env.pop('ORB_HUB_RUNS',None)
        if ready:env['VIO_TEST_MAP']='1'
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
            assert (out/'finalized_worker_exit').read_text()=='0'
            assert 'Finishing the current frame' in (out/'console.log').read_text()
        print('PASS',scenario)
    # Verify the real candidate start.sh still forwards non-live commands as before.
    after=subprocess.check_output(['bash',str(HERE.parent/'jetson/start.sh'),'--help'], text=True)
    for command in ('live', 'record', 'record-view', 'replay', 'trajectory'):
        assert 'start.sh '+command in after
    root=base/'shell_missing_helper';root.mkdir()
    shutil.copy2(HERE.parent/'jetson/start.sh',root/'start.sh')
    result=subprocess.run(['bash',str(root/'start.sh'),'live','0'],capture_output=True,text=True)
    assert result.returncode==1 and result.stdout=='' and result.stderr.startswith('VIO_E_'),result
    assert 'live_console.py' in result.stderr,result.stderr
    print('PASS non_live_help_and_missing_helper')
    # Exercise the real shell preflight without opening any sensor or X server.
    shell_root=base/'shell_display';shell_root.mkdir()
    for name in ('start.sh','live_console.py'):
        shutil.copy2(HERE.parent/'jetson'/name,shell_root/name)
    mockbin=base/'mockbin';mockbin.mkdir()
    for name,body in [('uname','printf "Linux aarch64\\n"'),('xdpyinfo','exit 0')]:
        executable=mockbin/name
        executable.write_text('#!/usr/bin/env bash\n'+body+'\n')
        executable.chmod(0o755)
    env={**os.environ,'DSIPLAY':':0','PATH':str(mockbin)+os.pathsep+os.environ['PATH']}
    env.pop('DISPLAY',None);env.pop('ORB_HUB_RUNS',None)
    result=subprocess.run(['bash',str(shell_root/'start.sh'),'live'],capture_output=True,text=True,env=env,timeout=10)
    assert result.returncode!=0 and result.stderr.startswith('VIO_E_DISPLAY:'),result
    assert 'DISPLAY' in result.stderr and 'DSIPLAY' in result.stderr,result.stderr
    print('PASS actual_shell_misspelled_display')
    # A stray misspelled variable must not override a valid inherited DISPLAY.
    env['DISPLAY']=':0'
    result=subprocess.run(['bash',str(shell_root/'start.sh'),'live'],capture_output=True,text=True,env=env,timeout=10)
    assert result.returncode!=0 and 'VIO_E_DISPLAY' not in result.stderr,result
    assert 'stock_viewer' in result.stderr,result.stderr
    print('PASS actual_shell_valid_display_not_overridden')
    # Failure before opening a startup log still includes its actual path.
    root=make_root(base,'startup_runs_not_directory')
    obstruction=base/'not-a-directory';obstruction.write_text('file')
    env={**os.environ,'ORB_HUB_RUNS':str(obstruction)}
    result=subprocess.run([sys.executable,str(root/'live_console.py'),'live','success'],capture_output=True,text=True,env=env,timeout=10)
    assert result.returncode!=0 and str(obstruction) in result.stderr,result
    assert LIVE not in result.stdout
    print('PASS startup_runs_path_failure')
    # Viewer stop is a request. The caller must still join workers/save before
    # process teardown; testing terminal signals alone would not catch this gate.
    adapter=(HERE.parent/'adapter/viewer.cc').read_text()
    main=adapter[adapter.index('int main('):]
    assert 'if (slam && !slam->isShutDown())' not in main
    assert 'if (slam) {' in main and main.count('slam->Shutdown();')==1
    assert main.index('slam->Shutdown();')<main.index('std::_Exit(result);')
    print('PASS viewer_request_caller_shutdown_contract')
    spec=importlib.util.spec_from_file_location('live_console_under_test',HERE.parent/'jetson/live_console.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    classifier_cases=[
        (['VIO_E_PATH: Missing executable: /opt/vio/bin/stock_viewer'],'VIO_E_PATH','/opt/vio/bin/stock_viewer'),
        (['MAP_E_CHECKSUM: reference map changed or is incomplete'],'MAP_E_CHECKSUM','reference map changed'),
        (["PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'"],'VIO_E_PERMISSION','/dev/ttyUSB0'),
        (["FileNotFoundError: [Errno 2] No such file or directory: '/data/settings.yaml'"],'VIO_E_PATH','/data/settings.yaml'),
        (['read_imu: AHRS framing or CRC changed during streaming.'],'VIO_E_IMU','CRC'),
        (['Cannot access X11 display :0.'],'VIO_E_DISPLAY',':0'),
        (['/opt/vio/bin/orb_live_viewer: error while loading shared libraries: libpangolin.so: cannot open shared object file: No such file or directory'],'VIO_E_DEPENDENCY','libpangolin.so'),
        (['Loading ORB vocabulary; progress in /tmp/orb.log','custom startup failure with useful detail'],'VIO_E_STARTUP','custom startup failure'),
    ]
    for lines,expected,detail in classifier_cases:
        code,reason=module.classify_startup_failure(lines)
        assert code==expected and detail in reason,(lines,code,reason)
    code,reason=module.classify_startup_failure([],error=PermissionError(13,'Permission denied','/tmp/no-write/runs'))
    assert code=='VIO_E_PERMISSION' and '/tmp/no-write/runs' in reason,(code,reason)
    for index,native_reason in enumerate([
        '/dev/gmsl/B has unexpected GMSL identity; expected vi-output, pnd_gmsl_mc 10-003b.',
        'Expected /dev/gmsl/B GMSL camera, 1920x1441 UYVY stride 3840; check v4l2-ctl --all.',
        'Short MMAP buffer.',
        'Camera stalled, poll=0 revents=0',
        'Unexpected camera clock/lag: event=123 lag_ms=2000.00. Recheck the driver clock domain.',
    ]):
        out=base/f'classifier_camera_{index}';out.mkdir()
        (out/'camera.log').write_text('GMSL device=/dev/gmsl/B raw=1920x1441 UYVY stride=3840\n'+native_reason+'\n')
        code,reason=module.classify_startup_failure(['read_camera: Camera helper exited; inspect camera.log.'],out)
        assert code=='VIO_E_CAMERA' and native_reason in reason,(code,reason,native_reason)
    print('PASS startup_classifier_cases',len(classifier_cases)+6)
print('LIVE_CONSOLE_TEST_PASS original_cases=13 additional_subprocess_cases=10 classifier_cases=14')
