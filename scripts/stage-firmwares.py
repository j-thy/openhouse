#!/usr/bin/env python3
"""Builds openhouse's firmware and puts it in a package's firmwares/ folder, where
openhouse-installer looks for it:

  firmwares/basestation/lhbs-<date>-<commit>.bin       1.0 base stations
  firmwares/dongle/doorman-dongle-sd.uf2              doorman dongle, UF2 bootloader with a SoftDevice
  firmwares/dongle/doorman-dongle-nosd.uf2            ... without one
  firmwares/dongle/doorman-holyiot.hex                Holyiot 21017 (Nordic DFU)
  firmwares/dongle/doorman-pca10059.hex               Nordic nRF52840 Dongle PCA10059 (same)
  firmwares/dongle/doorman-kuroshin.hex               Kuroshin dongle (same; DFU only by its reset pads)
    (the above are what a bootloader installs: boot stub + firmware, for a first install or a rescue)
  firmwares/dongle/bank/<board>-a.bin, -b.bin         the firmware for bank A / B: what a running
                                                      dongle updates itself with (board: sd, nosd,
                                                      holyiot, pca10059, kuroshin)
  firmwares/dongle/bank/version.txt                   their firmware version (hex, DONGLE_FW_VERSION):
                                                      the installer offers an update to a dongle on any other
  firmwares/radio/doorman-radio-wm3.fw      every 2.0-era device (nRF52 radio)
  firmwares/radio/doorman-radio-neo.bin  VIVE Tracker 1.0 (nRF51 radio)

usage: stage-firmwares.py PACKAGE_DIR      (needs arm-none-eabi-gcc with newlib)
"""
import datetime
import shutil
import subprocess
import sys
from pathlib import Path

repo = Path(__file__).resolve().parent.parent
out = Path(sys.argv[1]).resolve() / 'firmwares'
shutil.rmtree(out, ignore_errors=True)
for sub in ('basestation', 'dongle', 'radio'):
    (out / sub).mkdir(parents=True)

def make(path, *args):
    subprocess.run(['make', '-s', '-C', str(repo / path), *args], check=True)

make('basestation_firmware')
image = repo / 'basestation_firmware/build/firmware.bin'
data = image.read_bytes()
if sum(int.from_bytes(data[i:i + 4], 'little') for i in range(0, 32, 4)) % 2**32:
    sys.exit('lhbs: bad vector checksum')
commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', '--short', 'HEAD'], text=True).strip()
shutil.copy2(image, out / 'basestation' / f'lhbs-{datetime.date.today():%Y%m%d}-{commit}.bin')

make('doorman/dongle-fw')
shutil.copy2(repo / 'doorman/dongle-fw/build/doorman_dongle_sd.uf2', out / 'dongle/doorman-dongle-sd.uf2')
shutil.copy2(repo / 'doorman/dongle-fw/build/doorman_dongle_nosd.uf2', out / 'dongle/doorman-dongle-nosd.uf2')
shutil.copy2(repo / 'doorman/dongle-fw/build/doorman_dongle_holyiot.hex', out / 'dongle/doorman-holyiot.hex')
shutil.copy2(repo / 'doorman/dongle-fw/build/doorman_dongle_pca10059.hex', out / 'dongle/doorman-pca10059.hex')
shutil.copy2(repo / 'doorman/dongle-fw/build/doorman_dongle_kuroshin.hex', out / 'dongle/doorman-kuroshin.hex')
(out / 'dongle/bank').mkdir()
for board in ('sd', 'nosd', 'holyiot', 'pca10059', 'kuroshin'):
    for bank in 'ab':
        shutil.copy2(repo / f'doorman/dongle-fw/build/doorman_dongle_{board}_{bank}.bin', out / f'dongle/bank/{board}-{bank}.bin')
import re
version = re.search(r'#define\s+DONGLE_FW_VERSION\s+(0x[0-9A-Fa-f]+)', (repo / 'doorman/dongle-fw/src/dongle.h').read_text()).group(1)
(out / 'dongle/bank/version.txt').write_text(f'{int(version, 16):04X}\n')

make('doorman/radio-fw')
make('doorman/radio-fw', 'TARGET=idx')
shutil.copy2(repo / 'doorman/radio-fw/build-idx/radio_idx.fw', out / 'radio/doorman-radio-wm3.fw')
shutil.copy2(repo / 'doorman/radio-fw/build/radio.bin', out / 'radio/doorman-radio-neo.bin')
print(f'Staged firmware in {out}')
