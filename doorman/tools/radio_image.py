#!/usr/bin/env python3
"""Radio image tools.
  radio_image.py fix [--idx] FILE     patch the image length into the header and check it
                                      (default: HTC nRF51 layout +0x10/+0x14/+0x18, app at 0x400;
                                       --idx: Valve nRF52 layout +0x1C/+0x20/+0x24, app at 0x1000)
  radio_image.py container IN OUT     wrap an --idx image in SteamVR's .fw container (for
                                      lighthouse_watchman_update ... --target=nrf52)
  radio_image.py check FILE.fw        verify a .fw container (magic, lengths, both CRC32s)

.fw container (from Valve's indexcontroller/watchmanv3/tundra radio files): image, then a 56-byte
trailer: u32 0xC0DEA1DE, u32 6, u32 image length, u32 CRC32(image), u32 0, char[16] "Unknown",
char[16] "Unknown", u32 CRC32(trailer with this field zeroed).
"""
import struct, sys, zlib


def radio_signature(img):
    """The radio updater's staging check (HTC FUN_000020cc; Valve's nRF52 radio uses the same): 4-word LFSR over 0x10000..(len+15)&~15, padding = erased 0xFF."""
    n = (len(img) + 15) & ~15
    data = img + b'\xff' * (n - len(img))
    M = 0xFFFFFFFF
    s0 = s1 = s2 = s3 = 0
    for o in range(0, n, 16):
        w0, w1, w2, w3 = struct.unpack_from('<4I', data, o)
        top = (s1 << 31) & M
        s1n = (s1 >> 1) ^ ((s2 << 31) & M) ^ w1
        s2n = w2 ^ ((s2 >> 1) | ((s3 << 31) & M))
        t = s0 ^ (w3 >> 31)
        s3n = (((w3 & 0x7FFFFFFF) ^ (s3 >> 1)) | (((t ^ (s0 >> 2) ^ (s0 >> 27) ^ (s0 >> 29)) & 1) << 31)) & M
        s0 = ((s0 >> 1) | top) ^ w0
        s1, s2, s3 = s1n, s2n, s3n
    return struct.pack('<4I', s0, s1, s2, s3)

MAGIC = 0xC0DEA1DE

def fix(path, idx):
    d = bytearray(open(path, 'rb').read())
    d += b'\xff' * (-len(d) % 4)
    lo, mo, base, spmax = (0x1C, 0x20, 0x1000, 0x20003240) if idx else (0x10, 0x14, 0x400, 0x20001578)
    struct.pack_into('<I', d, lo, len(d))
    assert struct.unpack_from('<I', d, mo)[0] == 0xFFFFFFFF, f'+{mo:#x} must be blank'
    assert len(d) <= (0xF000 if idx else 0xF800), 'image too large'
    sp, rv = struct.unpack_from('<II', d, 0)
    assert 0x20000000 < sp <= spmax and base < rv < base + len(d) and rv & 1, (hex(sp), hex(rv))
    open(path, 'wb').write(d)
    print(f'{path}: {len(d)} B, sp {sp:#x}, reset {rv:#x}, version {struct.unpack_from("<I", d, mo + 4)[0]:#x}')

def trailer(img):
    t = struct.pack('<5I', MAGIC, 6, len(img), zlib.crc32(img), 0) + b'Unknown'.ljust(16, b'\0') * 2
    return t + struct.pack('<I', zlib.crc32(t + bytes(4)))

def check(path):
    m = open(path, 'rb').read()
    n = struct.unpack_from('<I', m, 0x1C)[0]
    img, t = m[:n], m[n:]
    assert len(t) == 56, f'trailer {len(t)} B'
    magic, ver, ln, crc, _ = struct.unpack_from('<5I', t)
    assert magic == MAGIC and ver == 6 and ln == n, (hex(magic), ver, ln, n)
    assert crc == zlib.crc32(img), 'image CRC32'
    assert struct.unpack_from('<I', t, 52)[0] == zlib.crc32(t[:52] + bytes(4)), 'trailer CRC32'
    return n

if __name__ == '__main__':
    a = sys.argv[1:]
    if a[0] == 'fix':
        idx = '--idx' in a
        fix([x for x in a[1:] if x != '--idx'][0], idx)
    elif a[0] == 'container':
        img = open(a[1], 'rb').read()
        open(a[2], 'wb').write(img + trailer(img)); print(f'{a[2]}: {check(a[2])} B image + trailer, ok')
    elif a[0] == 'check':
        print(f'{a[1]}: {check(a[1])} B image, container ok')
    else:
        sys.exit(__doc__)
