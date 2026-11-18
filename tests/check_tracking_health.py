#!/usr/bin/env python3
"""Compile the production standalone health gate and exercise counterexamples."""
from pathlib import Path
import os,shutil,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
compiler=os.environ.get('CXX') or shutil.which('c++') or shutil.which('g++')
if not compiler:raise SystemExit('A C++11 compiler is required')
with tempfile.TemporaryDirectory(prefix='vio-health-') as temp:
    binary=Path(temp)/'test'
    subprocess.run([compiler,'-std=c++11','-O2','-I'+str(root/'vio_src/include'),
      str(root/'vio_src/src/TrackingHealth.cc'),str(root/'tests/tracking_health_test.cc'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True,timeout=30)
