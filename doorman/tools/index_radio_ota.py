#!/usr/bin/env python3
"""Over-the-air radio update through a Valve/HTC Watchman dongle the device is paired to and connected
through: Valve nRF52 radios (Index controller family) and HTC nRF51 radios (Vive Tracker 1.0) alike.

Protocol (Valve radio FUN_0000478c, the host commands the dongle tunnels to the radio; same 64-byte
feature report the dongle uses for every device command, report id 0xff):
  ff 97                      open staging                       (= SPI 'Y')
  ff 98 <n> <n bytes>        write the next block, n <= 64, n % 4 == 0   (= 'Z')
  ff 99 <16-byte signature>  check + commit; radio answers 94 02 00 00   (= '[')
  ff 94 (GET)                last status from the device
The image is the radio app as linked at 0x1000 (a SteamVR .fw is accepted: its 56-byte trailer is
dropped), sent from offset 0. Signature = the 4-word LFSR over the image padded with 0xFF to 16 bytes
(radio FUN_00004d80 == app FUN_00431f48 == radio_image.radio_signature).
The radio installs the committed image at its next reset: the tool powers the device off afterwards
(0xff 0x9f "off!") and the user turns it back on.

  index_radio_ota.py probe   DONGLE_SERIAL           open staging only, print status (writes nothing)
  index_radio_ota.py install DONGLE_SERIAL IMAGE     stage + commit

The HTC Watchman dongle (28de:2101) relays 97/98 but doesn't complete the USB SET_REPORT for them
(the radio sends no reply), so hidraw's fixed 5 s timeout makes every block take 5 s. The dongle is
driven through libusb instead: SET_REPORT with a short timeout (a timeout is expected and ignored),
status via GET_REPORT. The kernel HID driver is detached for the run and re-attached afterwards.
"""
import os, struct, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radio_image import radio_signature

BLOCK = 48

import usb.core, usb.util

SET_TIMEOUT_MS = 150

class Dongle:
    def __init__(self, serial):
        self.d = None
        for d in usb.core.find(find_all=True, idVendor=0x28de):
            if d.idProduct in (0x2101, 0x2102) and usb.util.get_string(d, d.iSerialNumber) == serial:
                self.d = d
        if not self.d: sys.exit(f'dongle {serial} not found')
        self.reattach = self.d.is_kernel_driver_active(0)
        if self.reattach: self.d.detach_kernel_driver(0)
        usb.util.claim_interface(self.d, 0)
    def close(self):
        usb.util.release_interface(self.d, 0)
        if self.reattach: self.d.attach_kernel_driver(0)
    def set(self, pkt):
        try:
            self.d.ctrl_transfer(0x21, 0x09, 0x03ff, 0, pkt + bytes(65 - len(pkt)), SET_TIMEOUT_MS)
            return True
        except usb.core.USBTimeoutError:
            return False
    def get(self):
        return bytes(self.d.ctrl_transfer(0xa1, 0x01, 0x03ff, 0, 65, 1000))

def cmd(h, op, data=b''):
    t = time.time()
    ok = h.set(bytes([0xff, op, len(data)]) + data)
    return time.time() - t if ok else -1.0

def status(h):
    return h.get()[1:8]

# Radio app layouts: Valve nRF52 family (Index controller, 2018 trackers ...): app at 0x1000, length at
# +0x1C, commit slot +0x20; HTC nRF51 (Vive Tracker 1.0, wands): app at 0x400, length +0x10, slot +0x14.
LAYOUTS = (('nRF52 (Valve)', 0x1000, 0x1C, 0x20, 0x20010000), ('nRF51 (HTC)', 0x400, 0x10, 0x14, 0x20004000))

def load(path):
    m = open(path, 'rb').read()
    sp, rv = struct.unpack_from('<II', m, 0)
    for name, base, lo, mo, ram_end in LAYOUTS:
        n = struct.unpack_from('<I', m, lo)[0]
        if not (base < n <= len(m) and n <= 0xF800 and base < rv < base + n and 0x20000000 < sp <= ram_end): continue
        assert struct.unpack_from('<I', m, mo)[0] == 0xFFFFFFFF, f'+{mo:#x} (commit slot) must be blank'
        print(f'{name} radio app, {n} B, version {struct.unpack_from("<I", m, lo + 8)[0]:#x}')
        return m[:n]
    sys.exit('not a radio app image (neither layout fits)')

def main():
    a = sys.argv[1:]
    if len(a) < 2: sys.exit(__doc__)
    h = Dongle(a[1])
    try:
        run(h, a)
    finally:
        h.close()

def run(h, a):
    if a[0] == 'probe':
        print('status before:', status(h).hex())
        print(f'97 open: {cmd(h, 0x97):.2f} s (-1 = no USB ack, expected)'); time.sleep(0.3)
        print('status after: ', status(h).hex())
        return
    img = load(a[2])
    sig = radio_signature(img)
    pad = img + b'\xff' * (-len(img) % 4)
    print(f'image {len(img)} B, signature {sig.hex()}')
    print(f'97 open: {cmd(h, 0x97):.2f} s'); time.sleep(0.3)
    t0 = time.time()
    for off in range(0, len(pad), BLOCK):
        cmd(h, 0x98, pad[off:off + BLOCK])
        if (off + BLOCK) % 4096 < BLOCK: time.sleep(0.15)        # radio erases a 4 KB page
        else: time.sleep(0.01)
        if off % 4800 == 0: print(f'  {off}/{len(pad)}', flush=True)
    print(f'sent in {time.time() - t0:.1f} s')
    print(f'99 commit: {cmd(h, 0x99, sig):.2f} s')
    for _ in range(20):
        time.sleep(0.2)
        s = status(h)
        if s[:2] == b'\x94\x02': print('status:', s.hex()); break
    else:
        print('no status; last:', s.hex())
    # power the device off (host command 0xff 0x9f "off!", tunnelled by the dongle): the user turns it
    # back on, and the radio's bootloader installs the committed image at that boot
    cmd(h, 0x9f, b'off!')
    print('device told to power off: turn it back on to install, then check: lighthouse_console version')

if __name__ == '__main__':
    main()
