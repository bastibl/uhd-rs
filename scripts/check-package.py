#!/usr/bin/env python3
"""Verify packaged assets and prove debug tests need no runtime image files."""
import json, pathlib, subprocess, tarfile, tempfile
metadata=json.loads(subprocess.check_output(['cargo','metadata','--format-version=1','--no-deps']))
package=metadata['packages'][0]
subprocess.run(['cargo','package','--allow-dirty','--no-verify'],check=True)
archive=pathlib.Path(metadata['target_directory'])/'package'/f"{package['name']}-{package['version']}.crate"
with tempfile.TemporaryDirectory(prefix='uhd-pure-package-') as directory:
    with tarfile.open(archive) as tar: tar.extractall(directory,filter='data')
    root=pathlib.Path(directory)/f"{package['name']}-{package['version']}"
    assert len(list((root/'images/assets').iterdir()))==6
    assert (root/'LICENSE').is_file()
    proc=subprocess.run(['cargo','test','--manifest-path',str(root/'Cargo.toml'),'--lib','--no-run','--message-format=json'],capture_output=True,text=True,check=True)
    artifacts=[json.loads(line) for line in proc.stdout.splitlines() if line.startswith('{')]
    executable=next(x['executable'] for x in artifacts if x.get('reason')=='compiler-artifact' and x.get('executable'))
    (root/'images/assets').rename(root/'images/assets-unavailable')
    subprocess.run([executable,'images::tests'],cwd=directory,check=True)
