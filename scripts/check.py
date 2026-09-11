#!/usr/bin/env python3
"""Local build/test matrix; no hardware access or image downloads."""
import subprocess
commands = [
    ['cargo', 'fmt', '--all', '--check'],
    ['cargo', 'test', '--all-targets'],
    ['cargo', 'test', '--no-default-features', '--all-targets'],
    ['cargo', 'check', '--all-targets', '--features', 'smol'],
    ['cargo', 'check', '--all-targets', '--features', 'tokio'],
    ['cargo', 'check', '--all-targets', '--target', 'wasm32-unknown-unknown'],
    ['cargo', 'check', '--all-targets', '--target', 'wasm32-unknown-unknown', '--no-default-features'],
    ['cargo', 'test', '--release', '--lib', 'images::tests'],
    ['cargo', 'clippy', '--all-targets', '--', '-D', 'warnings'],
]
for command in commands:
    subprocess.run(command, check=True)
