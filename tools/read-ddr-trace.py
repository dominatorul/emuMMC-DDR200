#!/usr/bin/env python3
"""Read the original-driver Fusee diagnostic from an Atmosphère AFE2 report."""

import struct
import sys
from pathlib import Path


STAGES = (
    'boot', 'card initialization', 'ready', 'native SD open', 'host state check',
    'read CMD6 status', 'check DDR200 function', 'set DDR200 function', 'set clock',
    'tuning sweep', 'card status check', 'read', 'write', 'restore SDR12 access mode',
    'DMA polling', 'native SD power recovery', 'early SD power recovery', 'verify SD card identity',
    'verify DMA buffers after native open', 'reset DDR200 command system', 'wait for Horizon SD power',
    'native SD close',
)
SNAPSHOTS = ('initial tune', 'latest tune', 'before native open', 'after native open', 'reassert entry')
PERF_OFFSET = 0x350 + len(SNAPSHOTS) * 12
PERF_TOTALS = struct.Struct('<18Q')
PERF_CLOCK = struct.Struct('<5I')
LEGACY_SWEEP_OFFSET = PERF_OFFSET + PERF_TOTALS.size + PERF_CLOCK.size
LEGACY_SWEEPS = struct.Struct('<8I')
SWEEP_OFFSET = 0x240
SWEEP_HISTORY = struct.Struct('<32Q')
HANDOFF_REGISTERS = ('flags', 'CAR.source', 'host.ctrl2_and_clock', 'vendor.clock')
POLICY_SWEEPS = {
    0: 'initial', 1: 'handoff', 2: 'periodic', 3: 'read recovery', 4: 'full-rate retry',
    16: 'noise floor', 17: 'clock gated between taps', 18: 'clock on between taps',
    19: 'CMD6 settle target 0 ms', 20: 'CMD6 settle target 10 ms', 21: 'CMD6 settle target 100 ms',
    22: 'stream', 23: 'after idle', 24: 'drive strength', 25: 'CMD6 switch complete',
}
RATES = ('unavailable', 'SDR104', '166.40 MHz DDR200', 'full-rate DDR200')
MATRIX_PHASES = ('noise floor', 'clock gating', 'free-running clock', 'stream',
                 'idle', 'driver strength', 'throughput', 'CMD6 settling')
MATRIX_STATES = ('pending', 'running', 'complete', 'skipped', 'interrupted')
MATRIX_FLAGS = ('DMA window or sector range unavailable', 'full-rate transfer failed',
                'reduced-rate transfer failed', '30-second idle not reached',
                'drive strength test failed', 'requested rate or power unavailable',
                '64 MiB stream ended early', 'CMD6 settling comparison unavailable')
POWER_EVENTS = ('paired native close started', 'SDR104 before native close', 'native SD closed',
                'native SD opened', 'DDR200 reassertion after native close', 'retained DDR200 verified')
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


def print_sweeps(data, sweeps, full_khz, trace_version):
    history = SWEEP_HISTORY.unpack_from(data, SWEEP_OFFSET)
    taps = data[LEGACY_SWEEP_OFFSET:LEGACY_SWEEP_OFFSET + len(history)] if trace_version >= 14 else None
    omitted = max(0, sweeps - len(history))
    if omitted:
        print(f'{omitted} early sweeps exceed the report history capacity')
    for i in range(omitted, sweeps):
        sample = history[i % len(history)]
        tap = sample & 0xFF
        width = (sample >> 8) & 0xFF
        start = tap - (width - 1) // 2
        interval = f', passing taps {start}–{start + width - 1}' if width else ''
        if taps is not None:
            applied = taps[i % len(history)]
            interval += f', applied tap {applied}' if applied != 0xFF else ', no tap applied'
        env = (sample >> 32) & 0xFFFF
        khz = 166400 if env & 0x200 else full_khz
        volts = ('off', '1.8 V', '3.3 V', 'unknown')[(env >> 10) & 3]
        temperature = 'unavailable' if env & 0x1FF == 0x1FF else f'{((env & 0x1FF) - 256) / 2:.1f} °C'
        time = sample >> 48
        print(f'Sweep {i + 1} +{time / 100:.2f}s{" or later" if time == 65535 else ""} '
              f'at {khz / 1000:.2f} MHz: center {tap}, width {width}{interval}, '
              f'reassertions {(sample >> 16) & 0xFF}, data errors {(sample >> 24) & 0xFF}, '
              f'CPU temperature {temperature}, SD I/O setting {volts}')


def print_policy_trace(data, gprs):
    if len(data) < 0x450:
        raise ValueError('report is too short for the DDR200 policy trace')
    matrix_capacity = gprs[4] & 0xFFFFFFFF
    if matrix_capacity not in (0, 40):
        raise ValueError('invalid DDR200 trace capacity')
    state = gprs[30]
    ready_khz = gprs[6] & 0xFFFFFFFF
    rate = f'{ready_khz / 1000:.2f} MHz DDR200' if (state & 0x103) == 0x103 else RATES[state & 3]
    print(f'Policy {rate}, ready {bool(state & 0x100)}, '
          f'full rate unstable {bool(state & 0x200)}, upgrade attempts {(state >> 16) & 0xFF}')
    print(f'Last verified SD clock {(gprs[6] & 0xFFFFFFFF) / 1000:.2f} MHz, '
          f'last sampled data/scan clock {(gprs[28] >> 32) / 1000:.2f} MHz, '
          f'HC2/clock {gprs[28] & 0xFFFFFFFF:08X}, CAR source {gprs[29] >> 32:08X}, '
          f'vendor clock {gprs[29] & 0xFFFFFFFF:08X}')
    print(f'CMD6 group 3 supported {gprs[5] >> 32:#06x}, selected {(gprs[5] >> 16) & 0xFF}, '
          f'group 4 supported {gprs[5] & 0xFFFF:#06x}, selected {(gprs[5] >> 24) & 0xFF}')
    errors = gprs[6] >> 32
    print(f'Data command failures {errors}')
    if errors:
        for label, meta, status in (('First', gprs[9], gprs[10]), ('Last', gprs[7], gprs[8])):
            print(f'{label} failed CMD{(meta >> 16) & 0xFFFF}, tap {(meta >> 40) & 0xFF}, '
                  f'{(status >> 32) / 1000:.2f} MHz, error {(status >> 16) & 0xFFFF:04X}, '
                  f'IRQ {status & 0xFFFF:04X}, Auto CMD12 {meta >> 48:04X}, blocks {meta & 0xFFFF}')
    hygiene = gprs[11]
    print(f'CMD12 / CMD13 / CMD-DAT reset passed {(hygiene >> 8) & 0xFFFFFF}/{hygiene >> 32}, '
          f'last flags {hygiene & 7:03b}')
    verify = gprs[12]
    print(f'Two-tap data checks passed {(verify >> 16) & 0xFFFF}/{verify >> 32}, '
          f'last pair {(verify >> 8) & 0xFF} and {verify & 0xFF}')
    print(f'Early card power recoveries {gprs[13] & 0xFFFFFFFF}, waits for Horizon {gprs[13] >> 32}')
    before, after, _ = struct.unpack_from('<III', data, 0x124)
    for label, sample in (('Before handoff', before), ('After native open', after)):
        print(f'{label} GPIO state {sample & 0xFFFF:04X}, power pinmux {sample >> 16:04X}')
    for i, label in enumerate(('Read', 'Write')):
        for base, name in ((14, 'SD command'), (18, 'SD filesystem')):
            blocks, usec = gprs[base + i * 2:base + i * 2 + 2]
            rate = f'{blocks * 512 / usec:.2f} MB/s' if usec else 'n/a'
            print(f'{label} {name} {rate} across {blocks * 512 / 1048576:.2f} MiB since boot')
    if matrix_capacity:
        active = gprs[31]
        phase = (active >> 8) & 0x7F
        state_id = active & 0xFF
        rate = '166.40' if active & (0x80 << 8) else '199.68'
        matrix_state = MATRIX_STATES[state_id] if state_id < len(MATRIX_STATES) else str(state_id)
        phase_name = MATRIX_PHASES[phase] if phase < len(MATRIX_PHASES) else str(phase)
        print(f'Matrix {matrix_state}, {rate} MHz {phase_name}')
        for bit, flag in enumerate(MATRIX_FLAGS):
            if (active >> 32) & (1 << bit):
                print(f'  {flag}')
        for label, value in zip(('199.68', '166.40'), gprs[22:24]):
            usec, blocks = value >> 32, value & 0xFFFFFFFF
            result = f'{blocks * 512 / usec:.2f} MB/s' if usec else 'unavailable'
            print(f'{label} MHz timed read {result}, {blocks * 512 / 1048576:.2f}/256 MiB '
                  f'in {usec / 1e6:.3f}s of active commands')
    if (gprs[0] & 0xFF) == 16:
        blocks, usec = gprs[22] & 0xFFFFFFFF, gprs[22] >> 32
        copied, dma_us = gprs[23] & 0xFFFFFFFF, gprs[23] >> 32
        if blocks and usec and dma_us:
            print(f'199.68 MHz reads of at least 256 KiB: {blocks * 512 / usec:.2f} MB/s '
                  f'across {blocks / 2048:.2f} MiB')
            print(f'DMA wait {dma_us / usec:.1%} of command time, DMA-window rate '
                  f'{blocks * 512 / dma_us:.2f} MB/s, bounce-buffer data {copied / blocks:.1%}')
    runtime_capacity = 48 - matrix_capacity
    for title, count, capacity, offset in (
        ('Runtime', gprs[3] & 0xFFFFFFFF, runtime_capacity, 0),
        ('Matrix', gprs[4] >> 32, matrix_capacity, runtime_capacity),
    ):
        if not count or not capacity:
            continue
        print(f'{title} chronological trace')
        if count > capacity:
            print(f'  {count - capacity} earlier events exceeded capacity')
        switched_at = None
        for i in range(max(0, count - capacity), count):
            usec, byte_count, clock, reasserts, kind, left, right, tap, drive = struct.unpack_from(
                '<IIH6B', data, 0x150 + (offset + i % capacity) * 16)
            if kind == 5:
                print(f'  +{usec} us policy {RATES[left & 3]}, ready {bool(right)}, '
                      f'unstable {bool(drive & 128)}, upgrade attempts {drive & 127}')
                continue
            if kind == 6:
                event = POWER_EVENTS[left] if left < len(POWER_EVENTS) else str(left)
                status = 'OK' if right else 'failed'
                print(f'  +{usec} us {event}, {status}')
                continue
            if kind == 25:
                switched_at = usec
                print(f'  +{usec} us CMD6 switch complete at {clock / 100:.2f} MHz')
                continue
            eye = f'{left}–{right} ({right - left + 1})' if left <= right else 'unavailable'
            applied = 'none' if tap == 255 else str(tap)
            settle = f', began {(usec - switched_at) / 1000:.3f} ms after CMD6' if (
                switched_at is not None and 19 <= kind <= 21) else ''
            print(f'  +{usec} us {POLICY_SWEEPS.get(kind, kind)} at {clock / 100:.2f} MHz, '
                  f'eye {eye}, applied tap {applied}, {byte_count} bytes since reassert {reasserts}, '
                  f'card driver strength {drive}{settle}')


def decode(path):
    data = path.read_bytes()
    if len(data) < 0x350:
        raise ValueError(f'{path} is too short for a DDR200 trace')
    magic, reason, title = struct.unpack_from('<IIQ', data)
    gprs = struct.unpack_from('<32Q', data, 0x10)
    trace_version = gprs[0] & 0xFF
    if magic != 0x32454641 or gprs[0] >> 8 != 0x44445232303000 or not 6 <= trace_version <= 16:
        raise ValueError(f'{path} does not contain a Fusee DDR200 trace')
    if trace_version >= 9 and len(data) < PERF_OFFSET:
        raise ValueError(f'{path} is too short for a GPIO trace')
    if trace_version >= 10 and len(data) < LEGACY_SWEEP_OFFSET:
        raise ValueError(f'{path} is too short for a performance trace')
    if trace_version in (11, 12, 14) and len(data) < LEGACY_SWEEP_OFFSET + LEGACY_SWEEPS.size:
        raise ValueError(f'{path} is too short for a sweep history')
    has_rates = trace_version >= 7
    stage = gprs[1] >> 32
    print(f'{path}\nFS title {title:016X}, reason {reason:08X}')
    print(f'Stage {STAGES[stage] if stage < len(STAGES) else stage}, detail {gprs[1] & 0xFFFFFFFF:#x}')
    if trace_version >= 15:
        if stage == STAGES.index('DMA polling') and ((gprs[1] >> 16) & 0xFFFF) == 6:
            mode = 'SET' if gprs[1] & 0x8000 else 'CHECK'
            print(f'  CMD6 {mode}, groups 3 to 1 function bits {gprs[1] & 0x0FFF:03x}')
        print_policy_trace(data, gprs)
        return
    if stage == STAGES.index('DMA polling'):
        print(f'  CMD{(gprs[1] >> 16) & 0xFFFF}, blocks {gprs[1] & 0xFFFF}')
    if stage == STAGES.index('native SD power recovery') and gprs[1] & 0xFFFFFFFF == 0x218:
        print('  Native SDMMC ResultNoDevice (0x218)')
    if stage == STAGES.index('reset DDR200 command system'):
        steps = ('prepare command clock', 'check default command system',
                 'select default command system', 'return host to SDR12 for negotiation')
        step = gprs[1] & 0xFFFFFFFF
        if step < len(steps):
            print(f'  {steps[step]}')
    print(f'Completed reads {gprs[2] >> 32}, writes {gprs[2] & 0xFFFFFFFF}')
    sweeps = gprs[3] & 0xFFFFFFFF
    print(f'Reassert attempts {gprs[3] >> 32}, completed sweeps {sweeps}')

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
                if trace_version >= 11:
                    hw = gprs[29 if label == 'First' else 30]
                    print(f'  Tap {(hw >> 48) & 0xFF}, host control 2 {(hw >> 16) & 0xFFFF:04X}, '
                          f'Auto CMD12 status {hw & 0xFFFF:04X}')

    if trace_version >= 11:
        attempts = gprs[31] & (0xFFFF if trace_version >= 12 else 0xFFFFFFFF)
        recovered = (gprs[31] >> 32) & 0xFFFFFF
        print(f'Full-rate read recoveries {recovered}/{attempts} successful')
        if trace_version >= 12:
            label = 'CMD6 resets for failed reads' if trace_version >= 13 else 'Native SD reopens for failed reads'
            print(f'{label} {(gprs[31] >> 16) & 0xFFFF}')
        if recovered:
            print(f'Last recovered read passed at tap {gprs[31] >> 56}')
    if trace_version >= 13:
        first_khz, last_khz = gprs[7] >> 32, gprs[7] & 0xFFFFFFFF
        print_sweeps(data, sweeps, max(first_khz, last_khz, gprs[16] & 0xFFFFFFFF), trace_version)
        if trace_version >= 14:
            commands = struct.unpack_from('<2Q', data, 0x340)
            for label, entry in zip(('check', 'set'), commands):
                if not entry >> 63:
                    continue
                phase = (entry >> 40) & 0xFF
                result = (f'card status byte {(entry >> 32) & 0xFF:02X}' if entry & (1 << 48)
                          else f'failed at {COMMAND_FAILURES[phase] if phase < len(COMMAND_FAILURES) else phase}')
                print(f'Reset CMD6 {label}: {result}, error {(entry >> 16) & 0xFFFF:04X}, IRQ {entry & 0xFFFF:04X}')
    elif trace_version >= 11:
        history = LEGACY_SWEEPS.unpack_from(data, LEGACY_SWEEP_OFFSET)
        for i in range(max(0, sweeps - len(history)), sweeps):
            entry = history[i % len(history)]
            clock = '166.40 MHz recovery' if entry & (1 << 16) else 'full rate'
            print(f'Sweep {i + 1} at {clock}: center {entry & 0xFF}, '
                  f'width {(entry >> 8) & 0xFF}, reassertions {(entry >> 17) & 0x7F}, '
                  f'data errors {entry >> 24}')

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
        if trace_version >= 13 and i >= 2:
            names = HANDOFF_REGISTERS
            values = struct.unpack_from('<4I', data, 0x210 + (i - 2) * 16)
        else:
            names = REGISTERS
            values = struct.unpack_from('<24I', data, 0x150 + i * 96)
        if not values[0] & 1:
            continue
        states[label] = dict(zip(names, values))
        print(f'{label} (host accessible: {bool(values[0] & 2)}, Mariko: {bool(values[0] & 4)})')
        for name, val in zip(names[1:], values[1:]):
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
        for name, before in states[left].items():
            after = states[right].get(name)
            if after is not None and before != after:
                print(f'  {name:24s} {before:08X} → {after:08X}')


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        decode(Path(argument))
