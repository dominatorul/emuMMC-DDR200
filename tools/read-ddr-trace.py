#!/usr/bin/env python3
"""Read the original-driver Fusee diagnostic from an Atmosphère AFE2 report."""

import struct
import sys
from pathlib import Path


STAGES = (
    'boot', 'card initialization', 'ready', 'native SD open', 'host state check',
    'read CMD6 status', 'check DDR200 function', 'set DDR200 function', 'set clock',
    'tuning sweep', 'card status check', 'read', 'write', 'restore SDR12 access mode',
    'DMA polling', 'native SD power recovery', 'early SD power recovery',
)
SNAPSHOTS = ('initial tune', 'latest tune', 'before native open', 'after native open', 'reassert entry')
PERF_OFFSET = 0x350 + len(SNAPSHOTS) * 12
PERF_TOTALS = struct.Struct('<18Q')
PERF_CLOCK = struct.Struct('<5I')
COMMAND_FAILURES = ('unknown', 'clock disabled', 'command inhibit', 'DMA setup',
                    'command response', 'DMA transfer', 'card busy')
OPERATIONS = ('none', 'read', 'write', 'controller open', 'controller close')
OP_PHASES = ('none', 'entering', 'waiting on SD mutex', 'waiting on NAND mutex',
             'inside operation', 'releasing mutexes')
REGISTERS = (
    'flags', 'CAR.source', 'CAR.pllc4_base', 'CAR.pllc4_misc', 'CAR.reset_l', 'CAR.enable_l',
    'pad.drive', 'pad.loopback', 'pinmux.clk', 'pinmux.cmd', 'pinmux.dat0', 'card.power_gpio',
    'host.ctrl2_and_clock', 'host.ctrl_and_power', 'vendor.clock', 'vendor.tuning0',
    'vendor.tuning1', 'autocal.config', 'autocal.status', 'pad.compare', 'vendor.trim',
    'vendor.spare', 'vendor.misc', 'host.err_and_irq',
)


def print_performance(data):
    counters = PERF_TOTALS.unpack_from(data, PERF_OFFSET)
    khz, source, pll, host_clock, vendor = PERF_CLOCK.unpack_from(data, PERF_OFFSET + PERF_TOTALS.size)
    print(f'Last data command clock {khz / 1000:.2f} MHz, host mode {(host_clock >> 16) & 7}, '
          f'CAR.source {source:08X}, CAR.pllc4_base {pll:08X}, '
          f'host.ctrl2_and_clock {host_clock:08X}, vendor.clock {vendor:08X}')
    print('Cumulative transfer counters since boot')
    for i, direction in enumerate(('Read', 'Write')):
        calls, blocks, elapsed, dma, copied, errors = counters[i * 6:i * 6 + 6]
        successful = calls - errors
        rate = f'{blocks * 512 / elapsed:.2f} MB/s' if elapsed else 'n/a'
        average = f'{blocks / (2 * successful):.1f} KiB' if successful > 0 else 'n/a'
        bounce = f'{100 * copied / blocks:.1f}%' if blocks else 'n/a'
        print(f'  {direction} SD commands at {rate} during {elapsed / 1e6:.3f}s, '
              f'{calls} attempts, {errors} failed, {average} per successful command, '
              f'{bounce} copied through DMA buffers, {dma / 1e6:.3f}s in DMA polling')
        fs_calls, fs_blocks, fs_elapsed = counters[12 + i * 3:15 + i * 3]
        fs_rate = f'{fs_blocks * 512 / fs_elapsed:.2f} MB/s' if fs_elapsed else 'n/a'
        print(f'  {direction} SD filesystem at {fs_rate} across {fs_calls} successful requests, '
              f'{fs_elapsed / 1e6:.3f}s including lock wait and recovery')


def decode(path):
    data = path.read_bytes()
    if len(data) < 0x350:
        raise ValueError(f'{path} is too short for a DDR200 trace')
    magic, reason, title = struct.unpack_from('<IIQ', data)
    gprs = struct.unpack_from('<32Q', data, 0x10)
    trace_version = gprs[0] & 0xFF
    if magic != 0x32454641 or gprs[0] >> 8 != 0x44445232303000 or trace_version not in (6, 7, 8, 9, 10):
        raise ValueError(f'{path} does not contain a Fusee DDR200 trace')
    if trace_version >= 9 and len(data) < PERF_OFFSET:
        raise ValueError(f'{path} is too short for a GPIO trace')
    if trace_version >= 10 and len(data) < PERF_OFFSET + PERF_TOTALS.size + PERF_CLOCK.size:
        raise ValueError(f'{path} is too short for a performance trace')
    has_rates = trace_version >= 7
    stage = gprs[1] >> 32
    print(f'{path}\nFS title {title:016X}, reason {reason:08X}')
    print(f'Stage {STAGES[stage] if stage < len(STAGES) else stage}, detail {gprs[1] & 0xFFFFFFFF:#x}')
    if stage == STAGES.index('DMA polling'):
        print(f'  CMD{(gprs[1] >> 16) & 0xFFFF}, blocks {gprs[1] & 0xFFFF}')
    if stage == STAGES.index('native SD power recovery') and gprs[1] & 0xFFFFFFFF == 0x218:
        print('  Native SDMMC ResultNoDevice (0x218)')
    print(f'Completed reads {gprs[2] >> 32}, writes {gprs[2] & 0xFFFFFFFF}')
    print(f'Reassert attempts {gprs[3] >> 32}, completed sweeps {gprs[3] & 0xFFFFFFFF}')

    for label, window, mode, offset, khz in (
        ('Initial', gprs[4] >> 32, gprs[5], 0x50, gprs[7] >> 32),
        ('Latest', gprs[4] & 0xFFFFFFFF, gprs[6], 0x70, gprs[7] & 0xFFFFFFFF),
    ):
        rate = f' at {khz / 1000:.2f} MHz' if has_rates else ''
        print(f'{label} sweep{rate}: {window & 0x1FF} taps, center {(window >> 9) & 0xFF}, width {window >> 17}')
        for name, word in (('query', mode >> 32), ('set DDR200', mode & 0xFFFFFFFF)):
            print(f'  CMD6 {name}: command system {word >> 28:#x}, access mode {(word >> 24) & 0xF:#x}, '
                  f'group 2 support {word & 0xFFFF:#06x}, access support {(word >> 16) & 0xFF:#04x}')
        vals = struct.unpack_from('<8I', data, offset)
        print('  Comparator map: ' + ' '.join(f'{v:08X}' for v in vals[:4]))
        print('  Completed and passing: ' + ' '.join(f'{v:08X}' for v in vals[4:]))

    if has_rates:
        print(f'Last verified DDR200 clock {(gprs[16] & 0xFFFFFFFF) / 1000:.2f} MHz')
        for name, val in (('Full rate', gprs[23] >> 32), ('Reduced clock', gprs[23] & 0xFFFFFFFF)):
            if val:
                print(f'{name} latest window: center {(val >> 9) & 0xFF}, width {val >> 17}')
        print(f'Failed data commands {gprs[16] >> 32}')
        if gprs[16] >> 32:
            for label, start in (('First', 17), ('Latest', 20)):
                op, rate_irq, state = gprs[start:start + 3]
                phase = op >> 32
                print(f'{label} failure: CMD{op & 0xFFFFFFFF}, '
                      f'{COMMAND_FAILURES[phase] if phase < len(COMMAND_FAILURES) else phase}, '
                      f'{(rate_irq >> 32) / 1000:.2f} MHz, error {(rate_irq >> 16) & 0xFFFF:04X}, '
                      f'IRQ {rate_irq & 0xFFFF:04X}, present state {state >> 32:08X}, blocks {state & 0xFFFFFFFF}')

    if trace_version >= 8:
        if trace_version >= 9:
            print(f'Early SD power recoveries attempted: {gprs[28]}')
        elif gprs[28]:
            print('One-time early SD power recovery was attempted')
        op, phase, age, depth = gprs[24:28]
        if depth:
            kind = op >> 16
            print(f'Oldest active wrapper: {OPERATIONS[kind] if kind < len(OPERATIONS) else kind}, '
                  f'MMC ID {op & 0xFFFF}, {OP_PHASES[phase] if phase < len(OP_PHASES) else phase}, '
                  f'{age / 1000000:.3f}s, nesting {depth}')
        else:
            print('No active SDMMC wrapper at the time of the report')

    if trace_version >= 10:
        print_performance(data)

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
        if trace_version >= 9:
            power, mux, detect = struct.unpack_from('<3I', data, 0x350 + i * 12)
            print(f'  Power pin: input {power & 1}, GPIO {(power >> 1) & 1}, '
                  f'output enable {(power >> 2) & 1}, output {(power >> 3) & 1}, pinmux {mux:08X}')
            print(f'  Card detect input: {detect & 1} (active low)')
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
