#!/usr/bin/env python3
"""Strip and package the cross-built Windows DLL and standalone apps.
Reject unexpected third-party DLL dependencies rather than ship a broken bundle.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

repo = Path(__file__).resolve().parent.parent
build = Path(sys.argv[1]).resolve()
prefix = Path(sys.argv[2]).resolve()
out = repo / 'dist/openhouse-windows-x64'
shutil.rmtree(out, ignore_errors=True)
(out / 'apps').mkdir(parents=True)
files = {build / 'openvr/lighthouse/bin/win64/driver_lighthouse.dll': out / 'driver_lighthouse.dll',
         build / 'openhouse-installer.exe': out / 'openhouse-installer.exe'}
files.update({build / name: out / 'apps' / name for name in ['lhtool.exe', 'lhcapture.exe', 'lhviz.exe', 'openhouse-viewer.exe', 'openhouse-overlay.exe', 'doorman-dongle-manager.exe']})
system = {'advapi32.dll', 'winhttp.dll', 'kernel32.dll', 'shlwapi.dll', 'user32.dll', 'gdi32.dll',
          'shell32.dll', 'winmm.dll', 'ole32.dll', 'comdlg32.dll', 'msvcrt.dll',
          'ucrtbase.dll', 'ntdll.dll', 'imm32.dll', 'version.dll', 'oleaut32.dll', 'setupapi.dll', 'opengl32.dll', 'hid.dll', 'ws2_32.dll'}
artifacts = []
for source, target in files.items():
    if not source.is_file():
        sys.exit(f'Missing build artifact: {source}')
    subprocess.run(['x86_64-w64-mingw32-strip', '--strip-unneeded', '-o', str(target), str(source)], check=True)
    pe = subprocess.check_output(['x86_64-w64-mingw32-objdump', '-p', str(target)], text=True)
    imports = re.findall(r'DLL Name:\s*(\S+)', pe)
    for name in imports:
        if name.lower() not in system and not name.lower().startswith('api-ms-win-') and not (target.parent / name).is_file():
            sys.exit(f'Unbundled Windows dependency in {target}: {name}')
    artifacts.append({'file': str(target.relative_to(out)), 'imports': imports,
                      'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
subprocess.run([sys.executable, str(repo / 'scripts/stage-firmwares.py'), str(out)], check=True)
licenses = out / 'licenses'
licenses.mkdir(exist_ok=True)
for copyright in (prefix / 'share').glob('*/copyright'):
    shutil.copy2(copyright, licenses / (copyright.parent.name + '.txt'))
shutil.copy2(repo / 'third_party/imgui/LICENSE.txt', licenses / 'imgui.txt')
for notice in (repo / 'third_party/lighthouse-ble').glob('*.txt'):
    shutil.copy2(notice, licenses / ('lighthouse-ble-' + notice.name))
# ShayBox's Lighthouse library: the overlay's channel changes over Bluetooth (installed next to it).
# Built with MSVC (VCRUNTIME140, which SteamVR installs); without it the overlay just can't change channels.
subprocess.run([sys.executable, str(repo / 'scripts/fetch-lighthouse-ble.py'), 'windows', str(out / 'apps')], check=True, stdout=subprocess.DEVNULL)
shutil.copy2(repo / 'LICENSE', out / 'LICENSE.txt')
if not (build / 'VERSION.txt').is_file():
    sys.exit(f'Missing build artifact: {build / "VERSION.txt"}')
shutil.copy2(build / 'VERSION.txt', out / 'VERSION.txt')   # the build's version (cmake/build_id.cmake)
if Path('/usr/share/licenses/openvr').is_dir():
    shutil.copytree('/usr/share/licenses/openvr', licenses / 'openvr', dirs_exist_ok=True)
for package in ['mingw-w64-crt', 'mingw-w64-headers', 'mingw-w64-winpthreads', 'gcc']:
    notice = Path('/usr/share/licenses') / package
    if notice.is_dir():
        shutil.copytree(notice, licenses / package, dirs_exist_ok=True)
gpl = Path('/usr/share/licenses/spdx/GPL-3.0-only.txt')
if gpl.is_file():
    shutil.copy2(gpl, licenses / 'GCC-GPL-3.0.txt')
(out / 'BUILD.json').write_text(json.dumps({
    'version': (build / 'VERSION.txt').read_text().strip(),
    'source_commit': subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
    'source_dirty': bool(subprocess.check_output(['git', '-C', str(repo), 'diff', '--name-only'], text=True).strip()),
    'compiler': subprocess.check_output(['x86_64-w64-mingw32-g++', '-dumpfullversion'], text=True).strip(),
    'vcpkg_baseline': json.loads((repo / 'vcpkg.json').read_text())['builtin-baseline'],
    'artifacts': artifacts,
}, indent=2) + '\n')
# Executable bits too, for running them under Wine straight out of the zip.
for exe in list(out.rglob('*.exe')) + list(out.rglob('*.dll')):
    exe.chmod(0o755)
archive = shutil.make_archive(str(repo / 'dist/openhouse-windows-x64'), 'zip', out.parent, out.name)
# The unstripped binaries, to turn a crash record's module+offset (common/crash_log.hpp) into
# functions: x86_64-w64-mingw32-addr2line -fpCe FILE 0x140000000+OFFSET. Not shipped.
symbols = repo / 'dist/openhouse-windows-x64-symbols'
shutil.rmtree(symbols, ignore_errors=True)
symbols.mkdir()
for source, target in files.items():
    shutil.copy2(source, symbols / target.name)
shutil.make_archive(str(symbols), 'zip', symbols.parent, symbols.name)
shutil.rmtree(symbols)
print(f'Packaged {archive}')
for item in artifacts:
    print(f"  {item['file']}: {(out / item['file']).stat().st_size / 1048576:.1f} MiB")
