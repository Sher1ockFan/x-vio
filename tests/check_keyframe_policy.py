#!/usr/bin/env python3
"""Compare the current compiled keyframe policy to the original IMU_MONOCULAR policy."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
source=(root/'ORB_SLAM3/src/Tracking.cc').read_text()
start=source.index('bool Tracking::NeedNewKeyFrame()')
opening=source.index('{',start)
# Locate structural braces while skipping comments and quoted literals.
token=re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]')
depth=0
for m in token.finditer(source,opening):
    if m[0]=='{': depth+=1
    elif m[0]=='}':
        depth-=1
        if depth==0:
            body=source[opening+1:m.start()]
            break
else: raise ValueError('Function boundary not found')
harness=(root/'tests/keyframe_equivalence.cc').read_text()
a=harness.index('bool newPolicy()')
b=harness.index('{',a)
depth=0
for m in token.finditer(harness,b):
    if m[0]=='{':depth+=1
    elif m[0]=='}':
        depth-=1
        if depth==0:
            harness=harness[:b+1]+body+harness[m.start():]
            break
else:raise ValueError('Harness boundary not found')
with tempfile.TemporaryDirectory(prefix='mono-imu-keyframes-') as td:
    path=Path(td)/'policy.cc';path.write_text(harness)
    exe=Path(td)/'policy'
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-O2',str(path),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=20)
