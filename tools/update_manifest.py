#!/usr/bin/env python3
"""Record the current core sources; run after intentional ORB_SLAM3 edits."""
import hashlib
from pathlib import Path
root = Path(__file__).resolve().parents[1]
source = root / "ORB_SLAM3"
lines = []
for p in sorted(source.rglob("*")):
    if not p.is_file(): continue
    rel = p.relative_to(source)
    if any(part in {"build", "lib", ".git", "__pycache__"} or part.startswith("cmake-build-") for part in rel.parts): continue
    if p.name == ".DS_Store" or str(rel) in {"Thirdparty/g2o/config.h", "Vocabulary/ORBvoc.txt", "Examples/Monocular-Inertial/mono_inertial_euroc"}: continue
    lines.append(hashlib.sha256(p.read_bytes()).hexdigest() + "  ./" + rel.as_posix() + "\n")
(root / "source.sha256").write_text("".join(lines))
print(f"Recorded {len(lines)} core source files in source.sha256")
