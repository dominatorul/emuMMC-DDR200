#!/usr/bin/env python3
"""Verify KIP against ELF, then produce uncompressed KIP and hekate KIPM.

BLZ decoder follows SciresM/hactool's kip1_blz_uncompress (ISC license).
Copyright (c) 2017-2018 SciresM.
Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright notice
and this permission notice appear in all copies. THE SOFTWARE IS PROVIDED "AS
IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE.
"""
import pathlib
import struct

root = pathlib.Path(__file__).resolve().parents[1]
kip = (root / 'emummc.kip').read_bytes()
elf = (root / 'emummc.elf').read_bytes()
assert kip[:4] == b'KIP1' and kip[4:16].rstrip(b'\0') == b'FS'
assert struct.unpack_from('<Q', kip, 16)[0] == 0x0100000000000000
assert kip[28:30] == bytes([45, 3])
assert elf[:6] == b'\x7fELF\x02\x01'
assert struct.unpack_from('<H', elf, 18)[0] == 183
phoff = struct.unpack_from('<Q', elf, 32)[0]
phsize, phnum = struct.unpack_from('<HH', elf, 54)
loads = []
for i in range(phnum):
    ph = struct.unpack_from('<IIQQQQQQ', elf, phoff + i * phsize)
    if ph[0] == 1:
        loads.append(ph)
assert len(loads) == 3

def decode(data, size):
    total, header, additional = struct.unpack_from('<III', data, len(data) - 12)
    assert 12 <= header <= total <= len(data)
    start = len(data) - total
    src, dst = total - header, total + additional
    assert start + dst == size
    buf = bytearray(data) + bytearray(size - len(data))
    while dst:
        assert src > 0
        src -= 1
        control = buf[start + src]
        for _ in range(8):
            if control & 0x80:
                assert src >= 2
                src -= 2
                code = struct.unpack_from('<H', buf, start + src)[0]
                length, distance = min((code >> 12) + 3, dst), (code & 0xFFF) + 3
                dst -= length
                for j in range(length):
                    assert start + dst + j + distance < size
                    buf[start + dst + j] = buf[start + dst + j + distance]
            else:
                assert src > 0
                src -= 1
                dst -= 1
                buf[start + dst] = buf[start + src]
            control <<= 1
            if not dst:
                break
    return bytes(buf)

header = bytearray(kip[:0x100])
header[31] &= ~7
sections = []
offset = 0x100
flat = bytearray()
for i, ph in enumerate(loads):
    dest, size, compressed, attr = struct.unpack_from('<IIII', kip, 0x20 + 16 * i)
    assert dest == ph[3] and size == ph[5]
    raw = kip[offset:offset + compressed]
    if kip[31] & (1 << i):
        raw = decode(raw, size)
    assert raw == elf[ph[2]:ph[2] + ph[5]], 'KIP differs from ELF'
    struct.pack_into('<I', header, 0x28 + 16 * i, size)
    sections.append(raw)
    flat.extend(bytes(dest - len(flat)))
    flat.extend(raw)
    offset += compressed
assert offset == len(kip)
bss, bss_size, _, _ = struct.unpack_from('<IIII', kip, 0x50)
assert bss == (loads[2][3] + loads[2][5] + 0xFFF) & ~0xFFF
end = (loads[2][3] + loads[2][6] + 0xFFF) & ~0xFFF
assert bss + bss_size == end
flat.extend(bytes(end - len(flat)))
(root / 'emummc_unpacked.kip').write_bytes(header + b''.join(sections))
(root / 'emummc.kipm').write_bytes(kip[0x80:0x100] + flat)
print(f'KIP verified against ELF; KIPM image size {end:#x}, BSS {bss_size:#x}')
