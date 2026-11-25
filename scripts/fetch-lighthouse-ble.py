#!/usr/bin/env python3
"""Puts ShayBox's Lighthouse library (BLE channel changes, apps/openhouse-overlay/base_channels.cpp)
into a package: scripts/fetch-lighthouse-ble.py linux|windows OUT_DIR
Pinned to a release and checked against its SHA-256; kept in ~/.cache/openhouse once downloaded.
Its licenses (MIT, and its main dependencies') are in third_party/lighthouse-ble."""
import hashlib
from pathlib import Path
import shutil
import sys
import urllib.request

TAG = '2.1.0'
FILES = {
    'linux': ('liblighthouse-x86_64-unknown-linux-gnu.so', 'liblighthouse.so',
              '2856023bc93f72a48731865b281cb35fa530088e78d8a9fb6319e15864dc1577'),
    'windows': ('lighthouse-x86_64-pc-windows-msvc.dll', 'lighthouse.dll',
                '07c7e57e563458da84b3a2bd1599584eabc088f5c299b6bf135176bca2f6d8d3'),
}
asset, name, sha = FILES[sys.argv[1]]
out = Path(sys.argv[2])
cache = Path.home() / '.cache/openhouse' / f'lighthouse-{TAG}-{asset}'
if not cache.is_file() or hashlib.sha256(cache.read_bytes()).hexdigest() != sha:
    cache.parent.mkdir(parents=True, exist_ok=True)
    url = f'https://github.com/ShayBox/Lighthouse/releases/download/{TAG}/{asset}'
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    if hashlib.sha256(data).hexdigest() != sha:
        sys.exit(f'{asset}: SHA-256 mismatch')
    cache.write_bytes(data)
out.mkdir(parents=True, exist_ok=True)
shutil.copy2(cache, out / name)
print(out / name)
