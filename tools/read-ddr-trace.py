#!/usr/bin/env python3
"""Read the emuMMC DDR200 diagnostic from an Atmosphère AFE2 report."""

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
                'native SD opened', 'DDR200 reassertion after native close', 'retained DDR200 verified',
                'wake reassertion started', 'waiting for Horizon SD power')
COMMAND_FAILURES = ('unknown', 'clock disabled', 'command inhibit', 'DMA setup',
                    'command response', 'DMA transfer', 'card busy')
DMA_SAMPLES = ('SDMA reference slices', 'ADMA2 primary', 'SDMA 512 KiB boundary',
               'SDMA 256 KiB boundary', 'ADMA2 alternate', 'Last failed read')
DMA_MODES = ('unconfigured DMA', 'SDMA 512 KiB', 'SDMA 256 KiB', 'ADMA2')
DMA_RESULTS = ('unavailable', 'passed', 'command failed', 'data mismatch')
BATCH_REFUSALS = ('eye too narrow or tap outside margin', 'tap pair qualification failed',
                  '512 KiB run-in pending', 'request under 64 KiB', 'non-adjacent request',
                  'stale sweep', 'batching unavailable', 'window or DMA map unavailable')
DROP_REASONS = ('unspecified', 'no passing eye', 'no verified guarded pair', 'verification buffer unavailable',
                'verification command failed', 'verification data differed', 'verification timed out',
                'clock or tap readback failed', 'writes require two guarded taps', 'diagnostic transition',
                'retained-link wake verification', 'native close for sleep', 'card stopped responding',
                'distinct I/O retries exhausted')
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


def print_dma_comparison(data, offset, version):
    print('DDR200 DMA qualification' if version >= 23 else 'DDR200 DMA comparison')
    caps, config = struct.unpack_from('<QQ', data, offset + (len(DMA_SAMPLES) + 1) * 16)
    if config >> 32:
        supported = (config & 0xFF) >= 3 and (caps & 0x80000) and ((config >> 16) & 0x3000) == 0x3000
        print(f'  Host ADMA2 availability {bool(supported)}, capabilities {caps:#010x}, '
              f'Host Control 2 {(config >> 16) & 0xFFFF:#06x}')
    for i, label in enumerate(DMA_SAMPLES):
        if version >= 24 and i not in (1, 4):
            continue
        elapsed, blocks, remaining, mode, status, restarts, tap, left, right, adma_error = struct.unpack_from(
            '<IHHBBHBBBB', data, offset + i * 16)
        result = status & 0x7F
        if not result:
            continue
        outcome = DMA_RESULTS[result] if result < len(DMA_RESULTS) else f'result {result}'
        engine = DMA_MODES[mode] if mode < len(DMA_MODES) else f'mode {mode}'
        eye = f', eye {left}–{right} ({right - left + 1})' if left <= right else ''
        timing = f', {elapsed} us' if elapsed else ''
        progress = f', blocks remaining register {remaining}' if status & 0x80 else ''
        error = f', ADMA error register {adma_error:#04x}' if adma_error else ''
        print(f'  {label}: {outcome}, {engine}, tap {tap}{eye}, '
              f'{blocks / 2:.1f} KiB{timing}, {restarts} SDMA restarts{progress}{error}')
    for slot, label in ((6, 'read commands' if version >= 24 else 'commands'), (2, 'write commands')):
        if slot == 2 and version < 24:
            continue
        elapsed, blocks = struct.unpack_from('<QQ', data, offset + slot * 16)
        if elapsed:
            print(f'  ADMA2 {label} averaged {blocks * 512 / elapsed:.2f} MB/s across {blocks / 2048:.2f} MiB')
    first_check = offset + (16 if version >= 24 else 0)
    if not any(data[first_check:first_check + 16]):
        minimum = 6 if version >= 23 else 8
        print(f'  No same-sector check recorded. It requires ADMA2, a guarded {minimum}+ tap eye and sequential reads.')


def format_drop(source, target, reason, probes, left, right):
    detail = DROP_REASONS[reason] if reason < len(DROP_REASONS) else f'reason {reason}'
    eye = f'{left}–{right}' if left <= right else 'unavailable'
    return f'{RATES[source]} → {RATES[target]}, {detail}, {probes} distinct taps attempted, eye {eye}'


def print_batch_trace(data, offset):
    refused = struct.unpack_from('<8Q', data, offset + 8 * 16)
    print('Batching refusals by request')
    for label, count in zip(BATCH_REFUSALS, refused):
        print(f'  {label}: {count}')
    sequential, served = struct.unpack_from('<QQ', data, offset + 12 * 16)
    if sequential:
        print(f'Batched {served / sequential:.1%} of recognized sequential traffic '
              f'({served / 2048:.2f}/{sequential / 2048:.2f} MiB)')
    restarts, failures = struct.unpack_from('<QQ', data, offset + 13 * 16)
    print(f'SDMA boundary restarts across all emuMMC SD data commands: {restarts}')
    when, drop = struct.unpack_from('<QQ', data, offset + 14 * 16)
    if when:
        print('Latest clock drop ' + format_drop(drop & 3, (drop >> 8) & 3, (drop >> 16) & 0xFF,
              (drop >> 24) & 0xFF, (drop >> 32) & 0xFF, (drop >> 40) & 0xFF))
    if failures > 8:
        print(f'{failures - 8} earlier DMA failure records exceeded capacity')
    for i in range(max(0, failures - 8), failures):
        irq, blocks, remaining, flags = struct.unpack_from('<IHHQ', data, offset + (15 + i % 8) * 16)
        mode = (flags >> 36) & 3
        cmd, tap = (flags >> 46) & 0xFF, (flags >> 28) & 0xFF
        adma_error, progress = (flags >> 38) & 0xFF, (flags >> 19) & 0x1FF
        sampled, descriptors = (flags >> 54) & 1, flags >> 55
        if mode == 3:
            report = f'{progress}/{descriptors} whole descriptors by block count' if sampled else 'progress unavailable'
            report += f', ADMA error {adma_error:#04x}'
        else:
            report = f'{progress} restarts'
        remaining_text = f', {remaining} blocks remaining' if sampled else ''
        print(f'  Failed CMD{cmd}, {DMA_MODES[mode]}, tap {tap}, {blocks} blocks{remaining_text}, '
              f'DMA offset {flags & 0x7FFFF:#07x}, {report}, IRQ {irq:#010x}')


def print_lifecycle(data, offset):
    attempts, passed = struct.unpack_from('<QQ', data, offset)
    print('SD power lifecycle since boot')
    for event, label in enumerate(POWER_EVENTS):
        count = (attempts >> (8 * event)) & 0xFF
        if count:
            print(f'  {label}: {(passed >> (8 * event)) & 0xFF}/{count} successful')
    mapped, unmapped = struct.unpack_from('<QQ', data, offset + 3 * 16)
    for action, entry in (('map', mapped), ('unmap', unmapped)):
        if entry:
            print(f'  Last descriptor {action} DAS {entry >> 32:#010x}, result {entry & 0xFFFFFFFF:#010x}')
    commands, handoff_restarts = struct.unpack_from('<QQ', data, offset + 5 * 16)
    all_restarts, = struct.unpack_from('<Q', data, offset + 13 * 16)
    print(f'  Handoff SDMA commands {commands}, handoff restarts {handoff_restarts}, '
          f'outside handoff {max(0, all_restarts - handoff_restarts)}')


WAKE_STEPS = ('suspend begin', 'handoff CMD6', 'mapping retired', 'native close',
              'native open', 'wake host registers', 'wake power registers', 'wake reassertion',
              'descriptor map', 'ordinary command', 'DMA retry', 'wake failure', 'first failure',
              'wake PLLC4 registers', 'retained DDR link check')
DLL_REGISTERS = ('venclkctl', 'vencapover', 'veniotrimctl', 'vendllcalcfg', 'vendllcalcfgsts')


def print_wake(data, gprs, offset):
    count = gprs[3] >> 48
    if not count:
        return
    first = gprs[13] >> 48
    print(f'Lifecycle breadcrumbs, first failure record {first or "unavailable"}')
    for i in range(max(0, count - 22), count):
        timing, result, detail = struct.unpack_from('<QII', data, offset + (i % 22) * 16)
        step = timing >> 56
        label = WAKE_STEPS[step] if step < len(WAKE_STEPS) else str(step)
        if step == 5:
            fields = f'hostctl {detail >> 16:#04x}, hostctl2 {detail & 0xFFFF:#06x}, clkcon {result >> 16:#06x}, live {bool(result & 1)}, GPIO {(result >> 1) & 0x7fff:#06x}'
        elif step == 6:
            fields = f'CAR {detail:#010x}, pinmux {result:#010x}'
        elif step == 1:
            fields = f'{DMA_MODES[result >> 30]}, argument {detail:#010x}, result {result & 0x3fffffff:#010x}'
        elif step in (9, 10):
            mode = detail & 3
            fields = f'{DMA_MODES[mode]}, detail {detail:#x}, result {result:#010x}'
        elif step == 13:
            fields = f'base {detail:#010x}, misc {result:#010x}, locked {bool(detail & (1 << 27))}'
        elif step == 14:
            fields = (f'{RATES[detail & 3]}, clock configured {bool(result & 1)}, '
                      f'CMD13 ready {bool(result & 2)}, verified eye and data {bool(result & 4)}')
        else:
            fields = f'detail {detail:#x}, result {result:#010x}'
        print(f'  {timing & 0x00FFFFFFFFFFFFFF} us {label}, {fields}')


def print_dll(data, gprs):
    status = ('pending', 'baseline running', 'DLL running', 'DLL inactive', 'calibration timed out',
              'complete', 'link or I/O unavailable', 'cancelled')
    state = gprs[31]
    print(f'DLL matrix {status[state] if state < len(status) else state}')
    matrix_count, other_count = gprs[4] & 0xFFFFFFFF, gprs[4] >> 32
    baseline, calibrated = [], []
    for base, count, capacity in ((0, matrix_count, 26), (26, other_count, 6)):
        previous = None
        for i in range(max(0, count - capacity), count):
            *regs, packed = struct.unpack_from('<6I', data, 0x150 + (base + i % capacity) * 24)
            left, right, kind, mode = packed & 255, (packed >> 8) & 255, (packed >> 16) & 255, packed >> 24
            width = max(0, right - left + 1)
            arm = {32:'baseline idle', 33:'baseline stream', 34:'DLL idle', 35:'DLL stream', 36:'post-calibration registers'}.get(kind, f'sweep {kind}')
            if kind in (32, 33): baseline.append((left, right, width))
            if kind in (34, 35): calibrated.append((left, right, width))
            changed = ''
            if previous and previous[0] > width:
                moved = [name for name, a, b in zip(DLL_REGISTERS, previous[1], regs) if a != b]
                changed = ', narrowed, registers changed ' + (', '.join(moved) or 'none')
            warning = ' (expected 4)' if mode != 4 else ''
            print(f'  {arm}, eye {left}–{right} ({width}), UHS mode {mode}{warning}, ' +
                  ', '.join(f'{name}={value:08X}' for name, value in zip(DLL_REGISTERS, regs)) +
                  f', DLL active {bool(regs[4] & 0x80000000)}' + changed)
            previous = width, regs
    for label, arm in (('baseline', baseline), ('DLL', calibrated)):
        if arm:
            widths = [row[2] for row in arm]
            print(f'  {label} widths {min(widths)}–{max(widths)}, {len(arm)} sweeps')


def print_policy_trace(data, gprs):
    if len(data) < 0x450:
        raise ValueError('report is too short for the DDR200 policy trace')
    version = gprs[0] & 0xFF
    if version >= 26:
        mode = (gprs[28] >> 16) & 7
        print(f'Last sampled UHS mode {mode}' + (' (expected 4)' if mode != 4 else ''))
    if version == 27:
        print_dll(data, gprs)
        return
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
    print(f'CMD6 group 3 supported {(gprs[5] >> 32) & 0xFFFF:#06x}, selected {(gprs[5] >> 16) & 0xFF}, '
          f'group 4 supported {gprs[5] & 0xFFFF:#06x}, selected {(gprs[5] >> 24) & 0xFF}')
    errors = gprs[6] >> 32
    print(f'Data command failures {errors}')
    if (gprs[0] & 0xFF) >= 18:
        print(f'CMD25 write command failures {state >> 48}')
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
    perf_version = gprs[0] & 0xFF
    if perf_version >= 16:
        blocks, usec = gprs[22] & 0xFFFFFFFF, gprs[22] >> 32
        copied, dma_us = gprs[23] & 0xFFFFFFFF, gprs[23] >> 32
        if blocks and usec and dma_us:
            print(f'199.68 MHz reads of at least 256 KiB: {blocks * 512 / usec:.2f} MB/s '
                  f'across {blocks / 2048:.2f} MiB')
            print(f'DMA wait {dma_us / usec:.1%} of command time, DMA-window rate '
                  f'{blocks * 512 / dma_us:.2f} MB/s, bounce-buffer data {copied / blocks:.1%}')
        if perf_version >= 17 and gprs[2] >> 32 and gprs[31] & 0xffffffff:
            commands, restarts = gprs[2] >> 32, gprs[2] & 0xFFFFFFFF
            print(f'{commands} large CMD18 reads, average {blocks / (2 * commands):.1f} KiB, '
                  f'maximum {(gprs[31] & 0xFFFFFFFF) / 2:.1f} KiB, {restarts} SDMA boundary restarts')
        if perf_version >= 19:
            maps, = struct.unpack_from('<I', data, 0x120)
            served, result = struct.unpack_from('<II', data, 0x130)
            print(f'Largest Horizon read at the SD hook {(gprs[31] >> 32) / 2:.1f} KiB')
            print(f'Read-ahead served {served / 2048:.2f} MiB, DMA map attempts {maps}, '
                  f'last result {result:#010x}')
        if perf_version >= 20:
            print(f'Read-ahead fills {(gprs[4] >> 32) & 0xFFFF} completed, {gprs[4] >> 48} failed')
        if perf_version >= 21:
            flags = (state >> 24) & 0xFF
            print(f'Auto CMD23 completed {gprs[5] >> 48} reads, host version field {flags & 0x3F}, '
                  f'card support {bool(flags & 0x40)}, disabled after failure {bool(flags & 0x80)}')
    dma_capacity = 23 if perf_version >= 24 else len(DMA_SAMPLES) + 2 if perf_version >= 22 else 0
    if dma_capacity:
        print_dma_comparison(data, 0x150 + (48 - dma_capacity) * 16, perf_version)
    if perf_version >= 24:
        print_batch_trace(data, 0x150 + (48 - dma_capacity) * 16)
    if perf_version >= 25:
        print_lifecycle(data, 0x150 + (48 - dma_capacity) * 16)
    runtime_capacity = 48 - matrix_capacity - dma_capacity
    if perf_version == 26:
        runtime_capacity -= 22
        print_wake(data, gprs, 0x150 + runtime_capacity * 16)
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
            if kind == 7:
                print(f'  +{usec} us clock drop ' + format_drop(drive >> 6, (drive >> 4) & 3,
                      drive & 15, tap, left, right))
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
    if magic != 0x32454641 or gprs[0] >> 8 != 0x44445232303000 or not 6 <= trace_version <= 27:
        raise ValueError(f'{path} does not contain an emuMMC DDR200 trace')
    if trace_version >= 9 and len(data) < PERF_OFFSET:
        raise ValueError(f'{path} is too short for a GPIO trace')
    if trace_version >= 10 and len(data) < LEGACY_SWEEP_OFFSET:
        raise ValueError(f'{path} is too short for a performance trace')
    if trace_version in (11, 12, 14) and len(data) < LEGACY_SWEEP_OFFSET + LEGACY_SWEEPS.size:
        raise ValueError(f'{path} is too short for a sweep history')
    has_rates = trace_version >= 7
    stage = gprs[1] >> 32
    print(f'{path}\nFS title {title:016X}, reason {reason:08X}')
    if reason == 0xDD200066 and trace_version >= 26:
        print('Automatic sleep/wake failure snapshot')
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
