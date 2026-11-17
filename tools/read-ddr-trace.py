#!/usr/bin/env python3
"""Read the original-driver Fusee diagnostic from an Atmosphère AFE2 report."""

import struct
import sys
from pathlib import Path


STAGES = (
    'boot', 'card initialization', 'ready', 'native SD open', 'host state check',
    'read CMD6 status', 'check DDR200 function', 'set DDR200 function', 'set clock',
    'tuning sweep', 'card status check', 'read', 'write', 'restore SDR12 access mode',
)
SNAPSHOTS = ('initial tune', 'latest tune', 'before native open', 'after native open', 'reassert entry')
REGISTERS = (
    'flags', 'CAR.source', 'CAR.pllc4_base', 'CAR.pllc4_misc', 'CAR.reset_l', 'CAR.enable_l',
    'pad.drive', 'pad.loopback', 'pinmux.clk', 'pinmux.cmd', 'pinmux.dat0', 'card.power_gpio',
    'host.ctrl2_and_clock', 'host.ctrl_and_power', 'vendor.clock', 'vendor.tuning0',
    'vendor.tuning1', 'autocal.config', 'autocal.status', 'pad.compare', 'vendor.trim',
    'vendor.spare', 'vendor.misc', 'host.err_and_irq',
)


def decode(path):
    data = path.read_bytes()
    if len(data) < 0x350:
        raise ValueError(f'{path} is too short for a DDR200 trace')
    magic, reason, title = struct.unpack_from('<IIQ', data)
    gprs = struct.unpack_from('<32Q', data, 0x10)
    if magic != 0x32454641 or gprs[0] != 0x4444523230300006:
        raise ValueError(f'{path} does not contain a Fusee DDR200 trace')
    stage = gprs[1] >> 32
    print(f'{path}\nFS title {title:016X}, reason {reason:08X}')
    print(f'Stage {STAGES[stage] if stage < len(STAGES) else stage}, detail {gprs[1] & 0xFFFFFFFF:#x}')
    print(f'Completed reads {gprs[2] >> 32}, writes {gprs[2] & 0xFFFFFFFF}')
    print(f'Reassert attempts {gprs[3] >> 32}, completed sweeps {gprs[3] & 0xFFFFFFFF}')

    for label, window, mode, offset in (
        ('Initial', gprs[4] >> 32, gprs[5], 0x50),
        ('Latest', gprs[4] & 0xFFFFFFFF, gprs[6], 0x70),
    ):
        print(f'{label} sweep: {window & 0x1FF} taps, center {(window >> 9) & 0xFF}, width {window >> 17}')
        for name, word in (('query', mode >> 32), ('set DDR200', mode & 0xFFFFFFFF)):
            print(f'  CMD6 {name}: command system {word >> 28:#x}, access mode {(word >> 24) & 0xF:#x}, '
                  f'group 2 support {word & 0xFFFF:#06x}, access support {(word >> 16) & 0xFF:#04x}')
        vals = struct.unpack_from('<8I', data, offset)
        print('  Comparator map: ' + ' '.join(f'{v:08X}' for v in vals[:4]))
        print('  Completed and passing: ' + ' '.join(f'{v:08X}' for v in vals[4:]))

    # AFE2 stack-trace and stack-dump regions carry the snapshots.
    states = {}
    for i, label in enumerate(SNAPSHOTS):
        values = struct.unpack_from('<24I', data, 0x150 + i * 96)
        if not values[0] & 1:
            continue
        states[label] = values
        print(f'{label} (host accessible: {bool(values[0] & 2)}, Mariko: {bool(values[0] & 4)})')
        for name, val in zip(REGISTERS[1:], values[1:]):
            print(f'  {name:24s} {val:08X}')
    for left, right in (('before native open', 'after native open'), ('initial tune', 'latest tune')):
        if left not in states or right not in states:
            continue
        print(f'{left} → {right}')
        for name, before, after in zip(REGISTERS, states[left], states[right]):
            if before != after:
                print(f'  {name:24s} {before:08X} → {after:08X}')


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        decode(Path(argument))
