"""Native replay/export smoke with synthetic images and IMU; no accuracy claim."""
import csv
import numpy as np
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import time

archive = os.environ.get('MONO_IMU_ARCHIVE', 'mono_imu_vio.tar.gz')
cv = '/build/orb-kit/opencv-4.8.0/usr/lib'
lib = C.CDLL('libX11.so.6')
lib.XOpenDisplay.argtypes = [C.c_char_p]
lib.XOpenDisplay.restype = C.c_void_p
lib.XDefaultRootWindow.argtypes = [C.c_void_p]
lib.XDefaultRootWindow.restype = C.c_ulong
lib.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_char_p)]
lib.XQueryTree.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)]
lib.XFree.argtypes = [C.c_void_p]
lib.XCloseDisplay.argtypes = [C.c_void_p]

def windows(display, win):
    name = C.c_char_p()
    if lib.XFetchName(display, win, C.byref(name)) and name.value:
        yield name.value.decode(errors='replace')
        lib.XFree(name)
    root, parent, count = C.c_ulong(), C.c_ulong(), C.c_uint()
    children = C.POINTER(C.c_ulong)()
    if lib.XQueryTree(display, win, C.byref(root), C.byref(parent), C.byref(children), C.byref(count)):
        for i in range(count.value):
            yield from windows(display, children[i])
        if children:
            lib.XFree(children)

def hashfile(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1<<20), b''):
            h.update(block)
    return h.hexdigest()

with tempfile.TemporaryDirectory(prefix='orb-stock-smoke-') as scratch:
    tmp = Path(scratch)
    subprocess.run(['tar', '-xzf', archive, '-C', tmp], check=True)
    pkg = tmp/'mono_imu_vio'
    subprocess.run(['sha256sum', '--quiet', '-c', 'SHA256SUMS'], cwd=pkg, check=True)
    assert (pkg/'bin/orb_live_viewer').readlink() == Path('stock_viewer')
    usage = subprocess.check_output(['bash', str(pkg/'start.sh'), '--help'], text=True)
    assert all(command in usage for command in ('live', 'record', 'record-view', 'replay', 'recover'))
    (tmp/'home').mkdir()
    take = tmp/'sample'
    take.mkdir()
    for name in ('settings.yaml', 'timing.json'):
        (take/name).write_bytes((pkg/'params'/('orb_trial.yaml' if name=='settings.yaml' else name)).read_bytes())
    shift = json.loads((take/'timing.json').read_text())['timeshift_cam_imu_rounded_ns']
    count, n_imu = 60, 0
    with (take/'stream.bin').open('wb') as stream, (take/'frames.csv').open('w',newline='') as index:
        writer = csv.writer(index)
        writer.writerow(['frame_idx','sequence','camera_raw_ns','camera_ns','imu_count','offset','size'])
        pixels = np.random.default_rng(39).integers(0, 256, size=(1440, 1920), dtype=np.uint8).tobytes()
        for i in range(count):
            raw = 9_000_000_000 + i * 60_000_000
            ts = raw+shift
            samples = 30 if i else 1
            header = (f'{ts} {samples}\n' + ''.join(f'{ts-1_000_000-j*2_000_000} 0 0 9.8 0 0 0\n' for j in reversed(range(samples)))).encode()
            writer.writerow([i,i,raw,ts,samples,stream.tell(),len(header)+len(pixels)])
            stream.write(header)
            stream.write(pixels)
            n_imu += samples
    (take/'capture.json').write_text(json.dumps({'status':'COMPLETE','camera_shift_ns':shift,'frames':count,'imu_samples':n_imu,'driver_missing':0,'sha256':{p.name:hashfile(p) for p in take.iterdir()}}))
    manifest_path = take/'capture.json'
    manifest = json.loads(manifest_path.read_text())
    manifest.update(image_format='mono8', width=1920, height=1440, clock='CLOCK_MONOTONIC_RAW')
    manifest_path.write_text(json.dumps(manifest))
    eu = tmp/'eu'
    subprocess.run(['python3', str(pkg/'export_euroc.py'), str(take), str(eu)], check=True,
                   env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
    exported = json.loads((eu/'export.json').read_text())
    assert exported['status'] == 'EXPORT_PASS' and exported['exported_frames'] == count - 1
    assert exported['camera_shift_in_source_ns'] == 16881057 and exported['additional_camera_shift_ns'] == 0
    read_fd, write_fd = os.pipe()
    log = (tmp/'xvfb.log').open('wb')
    server = subprocess.Popen(['Xvfb','-displayfd',str(write_fd),'-screen','0','1280x1024x24','-nolisten','tcp'], pass_fds=(write_fd,),stdout=log,stderr=log)
    os.close(write_fd)
    replay = None
    try:
        assert select.select([read_fd],[],[],10)[0], (tmp/'xvfb.log').read_text()
        display = b':' + os.read(read_fd,40).strip()
        handle = lib.XOpenDisplay(display)
        assert handle, (tmp/'xvfb.log').read_text()
        try:
            root = lib.XDefaultRootWindow(handle)
            env = {**os.environ,'DISPLAY':display.decode(),'LD_LIBRARY_PATH':cv,'LIBGL_ALWAYS_SOFTWARE':'1'}
            env.pop('ORB_VOCABULARY', None)
            replay = subprocess.Popen(['bash',str(pkg/'start.sh'),'replay',str(take)],env=env,start_new_session=True)
            names=set()
            until=time.monotonic()+90
            while replay.poll() is None and time.monotonic()<until:
                names.update(windows(handle,root))
                time.sleep(.05)
            code = replay.wait(timeout=5)
            orb = list(take.glob('stock-replay-*/orb.log'))[0].read_text()
            print(orb[-4800:])
            print('Native X11 windows:',sorted(names))
            assert code == 0 and f'STOCK_STREAM_COMPLETE frames={count}' in orb
            assert 'ORB-SLAM3: Map Viewer' in names and 'ORB-SLAM3: Current Frame' in names
            assert '960' in orb and '720' in orb and '448.291' in orb
            print('HUB_GUI_AND_REPLAY_OK')
        finally:
            lib.XCloseDisplay(handle)
    finally:
        if replay and replay.poll() is None:
            replay.terminate()
            replay.wait(timeout=15)
        server.terminate()
        server.wait(timeout=10)
        os.close(read_fd)
        log.close()
