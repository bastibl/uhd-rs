#!/usr/bin/env python3
"""Maintainer-only: fetch pinned archives, verify SHA256, refresh packaged assets."""
import hashlib, io, json, pathlib, subprocess, zipfile
images_dir = pathlib.Path(__file__).resolve().parent
base = 'https://files.ettus.com/binaries/cache/'
records = []
for line in (images_dir / 'uhd-4.8-manifest.txt').read_text().splitlines():
    if not line.startswith('b2xx_'):
        continue
    target, revision, path, checksum = line.split()
    data = subprocess.check_output(['curl', '-fsSL', '--retry', '3', base + path])
    assert hashlib.sha256(data).hexdigest() == checksum, path
    files = {}
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        print(target, archive.namelist())
        for name in archive.namelist():
            leaf = pathlib.Path(name).name
            if leaf.endswith(('.bin', '.hex', '.img')):
                content = archive.read(name)
                (images_dir / 'assets' / leaf).write_bytes(content)
                files[leaf] = hashlib.sha256(content).hexdigest()
            elif not name.endswith('/'):
                dest = images_dir / 'notices' / target / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(archive.read(name))
    records.append(dict(target=target, revision=revision, url=base+path, sha256=checksum, files=files))
(images_dir / 'checksums.json').write_text(json.dumps(records, indent=2)+'\n')
