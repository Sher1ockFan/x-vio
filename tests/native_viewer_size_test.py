#!/usr/bin/env python3
"""Exercise the packaged native parser with synthetic, non-legacy image sizes.

Linux ARM64 only. No camera, serial device, GUI, or physical calibration is used.
Usage: python3 native_viewer_size_test.py /path/to/mono_vio_60.tar.gz
"""
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import tempfile


def settings(template, width, height):
    synthetic = {
        'Camera1.fx': width / 4, 'Camera1.fy': height / 4,
        'Camera1.cx': width / 2, 'Camera1.cy': height / 2,
        'Camera1.k1': 0.0, 'Camera1.k2': 0.0, 'Camera1.k3': 0.0, 'Camera1.k4': 0.0,
        'Camera.width': width, 'Camera.height': height, 'Camera.fps': 60,
        'IMU.NoiseGyro': 0.001, 'IMU.NoiseAcc': 0.002,
        'IMU.GyroWalk': 1e-5, 'IMU.AccWalk': 2e-5, 'IMU.Frequency': 500.0,
        'Camera.newWidth': width // 2, 'Camera.newHeight': height // 2,
    }
    for key, value in synthetic.items():
        template = re.sub(r'^' + re.escape(key) + r':.*$', f'{key}: {value}', template, flags=re.M)
    return template.replace('data: [REQUIRED]', 'data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]')


def main():
    archive = Path(sys.argv[1]).resolve()
    env = {k: v for k, v in os.environ.items()
           if k not in ('LD_LIBRARY_PATH', 'LD_PRELOAD', 'LD_AUDIT', 'DISPLAY')}
    env['VIO_TIMING'] = '1'
    with tempfile.TemporaryDirectory(prefix='mono60-native-viewer-') as tmp:
        root = Path(tmp)
        with tarfile.open(archive) as tar:
            for member in tar.getmembers():
                if Path(member.name).is_absolute() or '..' in Path(member.name).parts:
                    raise ValueError('Unsafe archive path')
            tar.extractall(root)
        pkg = root / 'mono_vio_60'
        env['LD_LIBRARY_PATH'] = str(pkg / 'lib')  # Same private lookup as start.sh.
        template = (pkg / 'params/orb_trial.template.yaml').read_text()
        for width, height, truncated in ((1920, 1536, False), (640, 512, False), (640, 512, True)):
            run = root / f'case_{width}_{height}_{truncated}'
            run.mkdir()
            cfg = run / 'settings.yaml'
            cfg.write_text(settings(template, width, height))
            pixels = bytes((i * 73 + 29) % 256 for i in range(width * height))
            packet = b'1000000000 1\n999000000 0 0 9.80665 0 0 0\n' + pixels
            if truncated:
                packet = packet[:-17]
            result = subprocess.run([str(pkg / 'bin/stock_viewer'),
                                     str(pkg / 'Vocabulary/ORBvoc.txt'), str(cfg), '--headless'],
                                    input=packet, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    cwd=run, env=env, timeout=90)
            log = result.stdout.decode(errors='replace')
            if truncated:
                assert result.returncode != 0 and 'STOCK_INPUT_ERROR' in log, log[-5000:]
            else:
                assert result.returncode == 0 and 'STOCK_STREAM_COMPLETE frames=1' in log, log[-5000:]
                assert len((run / 'tracking_timing.csv').read_text().splitlines()) == 2
            print(f'NATIVE_VIEWER_SIZE_PASS {width}x{height} truncated={truncated}', flush=True)


if __name__ == '__main__':
    main()
