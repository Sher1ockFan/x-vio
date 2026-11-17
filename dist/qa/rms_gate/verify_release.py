from pathlib import Path, PurePosixPath
import hashlib, json, re, tarfile

root = Path('/private/tmp/mono-rms-sthrname/mono_imu_vio')
old = Path('/Users/mot/Documents/vio_ws/mono_imu_vio')
archive = root / 'dist/mono_imu_vio.tar.gz'
sha = lambda data: hashlib.sha256(data).hexdigest()
manifest = {str(PurePosixPath(line[66:])): line[:64] for line in (root/'source.sha256').read_text().splitlines()}
for rel, digest in manifest.items():
    assert sha((root/'ORB_SLAM3'/rel).read_bytes()) == digest, rel

digests, symlinks, elfs, payloads = {}, {}, [], {}
with tarfile.open(archive, 'r|gz') as tar:
    for member in tar:
        path = PurePosixPath(member.name)
        assert not path.is_absolute() and '..' not in path.parts
        assert path.parts[0] == 'mono_imu_vio'
        rel = str(path.relative_to('mono_imu_vio'))
        if member.issym():
            symlinks[rel] = member.linkname
        elif member.isfile():
            stream = tar.extractfile(member)
            digest = hashlib.sha256()
            chunks = []
            keep = member.size < 4_000_000 or rel == 'bin/stock_viewer'
            first = stream.read(1 << 20)
            if first.startswith(b'\x7fELF'):
                assert first[4] == 2 and first[5] == 1 and int.from_bytes(first[18:20], 'little') == 183, rel
                elfs.append(rel)
            chunk = first
            while chunk:
                digest.update(chunk)
                if keep: chunks.append(chunk)
                chunk = stream.read(1 << 20)
            digests[rel] = digest.hexdigest()
            if keep: payloads[rel] = b''.join(chunks)

checks = {str(PurePosixPath(line[66:])): line[:64] for line in payloads['SHA256SUMS'].decode().splitlines()}
assert set(checks) == set(digests) - {'SHA256SUMS'}
for rel, digest in checks.items(): assert digests[rel] == digest, rel
assert len(elfs) == 8, elfs
for name in ('orb_live_viewer', 'orb_live_runner'):
    assert symlinks['bin/'+name] == 'stock_viewer'
shipped = 0
for rel, digest in digests.items():
    if rel.startswith('source/ORB_SLAM3/'):
        assert manifest[rel[len('source/ORB_SLAM3/'):]] == digest, rel
        shipped += 1
for path in (root/'jetson').iterdir():
    if path.is_file(): assert digests[path.name] == sha(path.read_bytes()), path
for path in (root/'params').iterdir():
    if path.is_file():
        assert path.read_bytes() == (old/'params'/path.name).read_bytes(), path
        assert digests['params/'+path.name] == sha(path.read_bytes()), path
for source, packaged in [('adapter/viewer.cc','source/adapter/viewer.cc'),
                         ('adapter/orb_replay/live.py','tools/orb_replay/live.py')]:
    assert digests[packaged] == sha((root/source).read_bytes()), source
for p in (root/'provenance').iterdir():
    assert digests['provenance/'+p.name] == sha(p.read_bytes()), p

def header(text, var):
    match = re.search(r'\b'+var+r'\s*<<\s*((?:"[^"\n]*"\s*)+)<<\s*std::endl', text)
    assert match, var
    return ''.join(re.findall(r'"([^"\n]*)"', match[1]))
new_adapter = (root/'adapter/viewer.cc').read_text()
old_adapter = (old/'adapter/viewer.cc').read_text()
headers = {}
for variable, filename in [('poses','poses.csv'),('events','recovery_events.csv'),
                           ('diagnostics','state_diagnostics.csv'),('timing','tracking_timing.csv')]:
    value = header(new_adapter, variable)
    assert value == header(old_adapter, variable), filename
    assert value.encode() in payloads['bin/stock_viewer'], filename
    headers[filename] = len(value.split(','))
recovery = header(new_adapter, 'recovery_timing')
assert len(recovery.split(',')) == 9 and recovery.encode() in payloads['bin/stock_viewer']
assert b'MONO_IMU_ATLAS_2' in payloads['source/ORB_SLAM3/src/System.cc']
changed = []
for p in sorted(root.rglob('*')):
    rel = p.relative_to(root)
    if rel.parts[0] in ('dist','.git') or '__pycache__' in rel.parts or not p.is_file(): continue
    previous = old/rel
    if not previous.is_file() or sha(p.read_bytes()) != sha(previous.read_bytes()):
        changed.append(str(rel))

report = dict(status='PACKAGE_AUDIT_PASS', manifest_files_verified=len(checks),
              core_source_files=len(manifest), shipped_core_source_files=shipped,
              calibration_files_unchanged=True, elf_aarch64=elfs,
              runtime_symlinks=symlinks, packaged_scripts_docs_sources_match=True,
              atlas_format='MONO_IMU_ATLAS_2', preserved_csv_schemas=headers,
              recovery_timing_columns=9, core_sha256=digests['lib/libORB_SLAM3.so'],
              source_manifest_sha256=sha((root/'source.sha256').read_bytes()),
              archive_sha256=sha(archive.read_bytes()), archive_bytes=archive.stat().st_size,
              changed_files=changed)
(root/'dist/package-audit.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
