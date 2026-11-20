/* SPDX-License-Identifier: GPL-2.0-only */
#include <stdatomic.h>
#include <string.h>
#include "ddr_trace.h"
#include "ddr_watchdog.h"
#include "util.h"
#include "../emmc/mmc.h"
#include "../soc/clock.h"
#include "../soc/gpio.h"
#include "../soc/pinmux.h"
#include "../soc/t210.h"

enum {
    HistoryCount = 48,
#ifdef EMUMMC_DDR200_ADMA_PROBE
    WakeCount = 22,
#else
    WakeCount = 0,
#endif
#if defined(EMUMMC_DDR200_MATRIX) && !defined(EMUMMC_DDR200_DLL_DIAG)
    MatrixCount = 40,
#else
    MatrixCount = 0,
#endif
#ifdef EMUMMC_DDR200_ADMA_PROBE
    DmaCount = DdrDmaSamples,
#else
    DmaCount = 0,
#endif
    RuntimeCount = HistoryCount - MatrixCount - DmaCount - WakeCount
};
typedef struct {
    _Atomic u64 timing;
    _Atomic u64 settings;
} sweep_event_t;

static sweep_event_t history[HistoryCount];
#ifdef EMUMMC_DDR200_ADMA_PROBE
static _Atomic u32 wake_events;
static _Atomic u32 first_wake_failure;
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result)
{
    if (step == WakeSuspend)
        first_wake_failure = 0;
    if (step == WakeFirstFailure && first_wake_failure)
        return;
    sweep_event_t *entry = &history[RuntimeCount + wake_events % WakeCount];
    entry->timing = (get_tmr_us() & 0x00ffffffffffffffull) | ((u64)step << 56);
    entry->settings = ((u64)detail << 32) | result;
    ++wake_events;
    if (step == WakeFirstFailure)
        first_wake_failure = wake_events;
}

void ddr_trace_wake_host(const sdmmc_t *host)
{
    bool live = host->regs && clock_sdmmc_is_not_reset_and_enabled(SDMMC_1);
    ddr_trace_wake(WakeHost, live ? ((u32)host->regs->hostctl << 16) | host->regs->hostctl2 : 0,
        (live ? ((u32)host->regs->clkcon << 16) | 1 : 0) |
        (gpio_get_pin_state(GPIO_PORT_E, GPIO_PIN_4) << 1));
    ddr_trace_wake(WakePower, CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1),
        PINMUX_AUX(PINMUX_AUX_DMIC3_CLK));
    ddr_trace_wake(WakePll, CLOCK(CLK_RST_CONTROLLER_PLLC4_BASE), CLOCK(CLK_RST_CONTROLLER_PLLC4_MISC));
}
#endif
#ifdef EMUMMC_DDR200_DLL_DIAG
enum { DllRows = 26, OtherRows = 6 };
static _Atomic u32 dll_rows, other_rows, dll_state;
static _Atomic u32 dll_history[DllRows + OtherRows][6];
void ddr_trace_dll_result(unsigned status)
{
    if (status == 1)
        dll_rows = 0;
    dll_state = status;
}
void ddr_trace_dll_sweep(const sdmmc_t *host, ddr_eye_t eye, u8 kind)
{
    bool experiment = kind >= 32 && kind <= 35;
    unsigned row = experiment ? dll_rows % DllRows : DllRows + other_rows % OtherRows;
    dll_history[row][0] = host->regs->venclkctl;
    dll_history[row][1] = host->regs->vencapover;
    dll_history[row][2] = host->regs->veniotrimctl;
    dll_history[row][3] = host->regs->vendllcalcfg;
    dll_history[row][4] = host->regs->vendllcalcfgsts;
    dll_history[row][5] = eye.left | ((u32)eye.right << 8) | ((u32)kind << 16) |
        ((u32)(host->regs->hostctl2 & 7) << 24);
    if (experiment) ++dll_rows;
    else ++other_rows;
}
#endif
static u64 trace_started;
static _Atomic u32 runtime_events, matrix_events;
static _Atomic u32 reads, writes, reasserts, io_errors, ready_khz;
static _Atomic u32 boot_power_recoveries, waiting_native;
static _Atomic u32 verify_attempts, verify_passed, verify_pair;
static _Atomic u32 recovery_attempts, recovery_passed;
static _Atomic u32 hygiene_attempts, hygiene_passed, hygiene_last;
static _Atomic u32 function_status;
#ifdef EMUMMC_DDR200_COUNTED_READS
static _Atomic u32 counted_reads, counted_read_flags;
#endif
static _Atomic u64 checkpoint, functions, ladder, bytes_since_reassert;
static _Atomic u64 first_failure[2], last_failure[2];
static _Atomic u64 sd_stats[2][2], fs_stats[2][2], matrix_stats[2][2];
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
static _Atomic u64 large_read_blocks, large_read_us, large_read_dma_us, large_read_copied;
static _Atomic u32 large_read_commands, large_read_restarts, largest_read, write_errors;
#endif
#ifdef EMUMMC_DDR200_READ_AHEAD
static _Atomic u32 largest_fs_read, read_ahead_blocks, map_attempts, map_result;
static _Atomic u32 read_ahead_fills[2];
#endif
static _Atomic u64 matrix_status;
#if defined(EMUMMC_DDR200_ADMA_PROBE) && !defined(EMUMMC_DDR200_READ_AHEAD) && !defined(EMUMMC_DDR200_MATRIX)
static _Atomic u32 pad_recalibration;
#endif
static _Atomic u64 bus_registers, clock_registers;
static _Atomic u64 native_before, native_after;
static _Atomic u32 sampled_rate;

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_pad_recalibration(ddr_eye_t before, ddr_eye_t after, bool used_fallback)
{
#if !defined(EMUMMC_DDR200_READ_AHEAD) && !defined(EMUMMC_DDR200_MATRIX)
    pad_recalibration = before.left | 0x80u | ((u32)before.right << 8) |
        ((u32)used_fallback << 15) | ((u32)after.left << 16) | ((u32)after.right << 24);
#else
    (void)before;
    (void)after;
    (void)used_fallback;
#endif
}
#endif

void ddr_trace_stage(enum DdrStage stage, u32 detail)
{
    atomic_store_explicit(&checkpoint, ((u64)stage << 32) | detail, memory_order_relaxed);
    if (stage == DdrReady)
        ready_khz = detail;
}

void ddr_trace_host(enum DdrSnapshot slot, const sdmmc_t *host)
{
    if (slot != DdrBeforeOpen && slot != DdrAfterOpen && slot != DdrReassert)
        return;
    u32 power = gpio_get_pin_state(GPIO_PORT_E, GPIO_PIN_4);
    u32 mux = PINMUX_AUX(PINMUX_AUX_DMIC3_CLK);
    bool live = host->regs && clock_sdmmc_is_not_reset_and_enabled(SDMMC_1);
    u64 state = ((u64)mux << 32) | ((u64)power << 16) | (live ? host->regs->hostctl2 : 0);
    if (slot == DdrAfterOpen)
        native_after = state;
    else
        native_before = state;
}

static void record_event(u8 kind, u32 khz, ddr_eye_t eye, unsigned tap, u8 drive, u64 time)
{
    if (!trace_started)
        trace_started = time;
    bool experimental = kind >= DdrSweepNoise;
    _Atomic u32 *count = experimental ? &matrix_events : &runtime_events;
    unsigned capacity = experimental ? MatrixCount : RuntimeCount;
    if (!capacity)
        return;
    unsigned index = *count % capacity + (experimental ? RuntimeCount : 0);
    u64 elapsed = MIN(time - trace_started, 0xffffffffull);
    u64 bytes = MIN(bytes_since_reassert, 0xffffffffull);
    history[index].timing = elapsed | (bytes << 32);
    history[index].settings = (khz / 10) | ((u64)MIN(reasserts, 255u) << 16) |
        ((u64)kind << 24) | ((u64)eye.left << 32) | ((u64)eye.right << 40) |
        ((u64)(tap < DdrTapCount ? tap : 255) << 48) | ((u64)drive << 56);
    ++*count;
}

void ddr_trace_policy_sweep(const sdmmc_t *host, ddr_eye_t eye, unsigned tap, u8 kind, u8 drive, u64 started)
{
    record_event(kind, host->divisor, eye, tap, drive, started);
#ifdef EMUMMC_DDR200_DLL_DIAG
    ddr_trace_dll_sweep(host, eye, kind);
#endif
    bus_registers = ((u64)host->divisor << 32) | ((u32)host->regs->hostctl2 << 16) | host->regs->clkcon;
    clock_registers = ((u64)CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1) << 32) | host->regs->venclkctl;
}

void ddr_trace_tuning(sdmmc_t *host, const u32 raw[4], const u32 complete[4],
                      u32 count, u32 tap, u32 width, u32 selected)
{
    (void)raw;
    (void)complete;
    (void)count;
    unsigned start = width ? tap - (width - 1) / 2 : DdrTapCount;
    ddr_eye_t eye = {start, width ? start + width - 1 : 0};
    ddr_trace_policy_sweep(host, eye, selected, DdrSweepInitial, 0, get_tmr_us());
}

void ddr_trace_ladder(enum DdrRate rate, bool ready, bool unstable, unsigned upgrades)
{
    ladder = (u64)rate | ((u64)ready << 8) | ((u64)unstable << 9) | ((u64)upgrades << 16);
    record_event(DdrSweepLadder, 0, (ddr_eye_t){rate, ready}, 255,
                 (u8)(unstable ? 0x80 | upgrades : upgrades), get_tmr_us());
}

void ddr_trace_verify(unsigned tap, unsigned second, bool passed)
{
    verify_pair = (tap << 8) | second;
    ++verify_attempts;
    if (passed)
        ++verify_passed;
}

void ddr_trace_hygiene(bool stopped, bool ready, bool reset)
{
    hygiene_last = stopped | ((u32)ready << 1) | ((u32)reset << 2);
    ++hygiene_attempts;
    if (reset)
        ++hygiene_passed;
}

void ddr_trace_functions(u16 drive, u16 power, u8 selected_drive, u8 selected_power)
{
    functions = ((u64)drive << 32) | power | ((u32)selected_drive << 16) | ((u32)selected_power << 24);
}

void ddr_trace_card(const u8 *status, bool selected)
{
    if (selected)
        function_status = ((u32)status[16] << 8) | status[15];
}

void ddr_trace_io(bool write)
{
    if (write)
        ++writes;
    else
        ++reads;
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_dma_begin(void)
{
    sweep_event_t *base = &history[HistoryCount - DmaCount];
    base[DdrDmaBaseline].settings = 0;
    base[DdrDmaAlternate].settings = 0;
}

void ddr_trace_dma_sample(enum DdrDmaSample sample, const sdmmc_t *host, ddr_eye_t eye,
    u64 elapsed, u32 blocks, u32 resumes, enum DdrDmaResult result)
{
    const sdmmc_dma_progress_t *progress = &host->dma_progress;
    sweep_event_t *slot = &history[HistoryCount - DmaCount + sample];
    unsigned tap = (host->regs->venclkctl >> 16) & 0xff;
    slot->timing = MIN(elapsed, 0xffffffffull) | ((u64)MIN(blocks, 0xffffu) << 32) |
        ((u64)progress->remaining << 48);
    slot->settings = progress->mode | ((u64)result << 8) | ((u64)progress->sampled << 15) |
        ((u64)MIN(resumes, 0xffffu) << 16) | ((u64)tap << 32) |
        ((u64)eye.left << 40) | ((u64)eye.right << 48) | ((u64)progress->adma_error << 56);
}

void ddr_trace_sdma_restart(bool handoff)
{
    ++history[HistoryCount - DmaCount + DdrDmaStats].timing;
    if (handoff)
        ++history[HistoryCount - DmaCount + DdrDmaHandoff].settings;
}

void ddr_trace_handoff_sdma(void)
{
    ++history[HistoryCount - DmaCount + DdrDmaHandoff].timing;
}

void ddr_trace_descriptor_mapping(bool unmap, u32 das, Result result)
{
    sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaMappings];
    u64 sample = ((u64)das << 32) | result;
    if (unmap)
        slot->settings = sample;
    else
        slot->timing = sample;
}

void ddr_trace_batch_refused(enum DdrBatchRefusal reason)
{
    sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaRefusals + reason / 2];
    if (reason & 1)
        ++slot->settings;
    else
        ++slot->timing;
}

void ddr_trace_batch_seq(u32 blocks, bool served)
{
    sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaSequential];
    slot->timing += blocks;
    if (served)
        slot->settings += blocks;
}

void ddr_trace_clock_drop(enum DdrRate from, enum DdrRate to, enum DdrDropReason reason,
    u8 probes, ddr_eye_t eye)
{
    u64 now = get_tmr_us();
    sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaLastDrop];
    slot->timing = now;
    slot->settings = from | ((u64)to << 8) | ((u64)reason << 16) | ((u64)probes << 24) |
        ((u64)eye.left << 32) | ((u64)eye.right << 40);
    record_event(DdrSweepClockDrop, 0, eye, probes, (from << 6) | (to << 4) | reason, now);
}
#endif

void ddr_trace_io_error(const sdmmc_t *host, u32 cmd, u32 blocks)
{
    u32 tap = (host->regs->venclkctl >> 16) & 0xff;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    const sdmmc_dma_progress_t *dma = &host->dma_progress;
    unsigned progress = host->dma_resumes;
    if (dma->mode == SdmmcDmaAdma2 && dma->sampled && dma->remaining <= dma->blocks) {
        unsigned completed_bytes = (dma->blocks - dma->remaining) * dma->block_bytes;
        progress = dma->remaining ? completed_bytes / SdmmcAdmaChunk : dma->descriptors;
    }
    unsigned slot_id = HistoryCount - DmaCount + DdrDmaFailures + io_errors % DdrDmaFailureCount;
    history[slot_id].timing = host->command_irq | ((u64)(dma->blocks ? dma->blocks : MIN(blocks, 0xffffu)) << 32) |
        ((u64)dma->remaining << 48);
    history[slot_id].settings = dma->address_low | ((u64)MIN(progress, 0x1ffu) << 19) |
        ((u64)tap << 28) | ((u64)dma->mode << 36) | ((u64)dma->adma_error << 38) |
        ((u64)cmd << 46) | ((u64)dma->sampled << 54) | ((u64)dma->descriptors << 55);
#endif
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    if (cmd == MMC_WRITE_MULTIPLE_BLOCK)
        ++write_errors;
#endif
    last_failure[0] = ((u64)host->divisor << 32) | host->command_irq;
    last_failure[1] = ((u64)host->command_auto_stop_error << 48) |
        ((u64)tap << 40) | ((u64)host->command_failure << 32) | (cmd << 16) | MIN(blocks, 0xffffu);
    if (++io_errors == 1) {
        first_failure[0] = last_failure[0];
        first_failure[1] = last_failure[1];
    }
}

void ddr_trace_read_recovery(const sdmmc_t *host, bool succeeded)
{
    (void)host;
    if (succeeded)
        ++recovery_passed;
    else
        ++recovery_attempts;
}

void ddr_trace_sd_perf(const sdmmc_t *host, bool write, bool succeeded, bool copied,
                       u32 blocks, u64 elapsed, u64 dma)
{
    (void)copied;
    (void)dma;
    u64 previous_time = atomic_fetch_add_explicit(&sd_stats[write][1], elapsed, memory_order_relaxed);
    if (!succeeded)
        return;
    u64 total = atomic_fetch_add_explicit(&sd_stats[write][0], blocks, memory_order_relaxed);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (host->dma_progress.mode == SdmmcDmaAdma2) {
        sweep_event_t *slot = &history[HistoryCount - DmaCount + (write ? DdrDmaWriteTotals : DdrDmaTotals)];
        slot->timing += elapsed;
        slot->settings += blocks;
    }
#endif
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    if (!write && host->divisor == 199680 && (host->regs->hostctl2 & 7) == UHS_DDR50_BUS_SPEED &&
        blocks >= DdrVerifyBlocks) {
        large_read_blocks += blocks;
        large_read_us += elapsed;
        large_read_dma_us += dma;
        ++large_read_commands;
        large_read_restarts += host->dma_resumes;
        if (blocks > largest_read)
            largest_read = blocks;
        if (copied)
            large_read_copied += blocks;
    }
#endif
    atomic_fetch_add_explicit(&bytes_since_reassert, (u64)blocks * 512, memory_order_relaxed);
    if (!previous_time || (total >> 12) != ((total + blocks) >> 12) || sampled_rate != host->divisor) {
        sampled_rate = host->divisor;
        bus_registers = ((u64)host->divisor << 32) | ((u32)host->regs->hostctl2 << 16) | host->regs->clkcon;
        clock_registers = ((u64)CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1) << 32) | host->regs->venclkctl;
#ifdef EMUMMC_DDR200_ADMA_PROBE
        sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaHost];
        slot->timing = host->regs->capareg;
        slot->settings = (1ull << 32) | ((u32)host->regs->hostctl2 << 16) | host->regs->hcver;
#endif
    }
}

void ddr_trace_fs_perf(bool write, u32 blocks, u64 elapsed)
{
    atomic_fetch_add_explicit(&fs_stats[write][0], blocks, memory_order_relaxed);
    atomic_fetch_add_explicit(&fs_stats[write][1], elapsed, memory_order_relaxed);
#ifdef EMUMMC_DDR200_READ_AHEAD
    u32 previous = atomic_load_explicit(&largest_fs_read, memory_order_relaxed);
    while (!write && blocks > previous &&
        !atomic_compare_exchange_weak_explicit(&largest_fs_read, &previous, blocks,
            memory_order_relaxed, memory_order_relaxed)) {
    }
#endif
}

#ifdef EMUMMC_DDR200_COUNTED_READS
void ddr_trace_counted_read(u8 version, bool supported, bool disabled, bool completed)
{
    counted_read_flags = (version & 0x3f) | ((u32)supported << 6) | ((u32)disabled << 7);
    if (completed)
        ++counted_reads;
}
#endif

#ifdef EMUMMC_DDR200_READ_AHEAD
void ddr_trace_read_ahead_map(Result result)
{
    map_result = result;
    ++map_attempts;
}

void ddr_trace_read_ahead(u32 blocks)
{
    atomic_fetch_add_explicit(&read_ahead_blocks, blocks, memory_order_relaxed);
}

void ddr_trace_read_ahead_fill(bool succeeded)
{
    ++read_ahead_fills[succeeded];
}
#endif

void ddr_trace_reassert(void)
{
    ++reasserts;
    bytes_since_reassert = 0;
}

void ddr_trace_boot_power(void) { ddr_trace_stage(DdrBootPower, ++boot_power_recoveries); }
void ddr_trace_wait_native(void) { ++waiting_native; }

void ddr_trace_power(enum DdrPowerEvent event, bool ok)
{
#ifdef EMUMMC_DDR200_ADMA_PROBE
    _Static_assert(DdrPowerWaiting < 8, "power count layout");
    sweep_event_t *slot = &history[HistoryCount - DmaCount + DdrDmaPower];
    unsigned shift = event * 8;
    u64 count = slot->timing;
    if (((count >> shift) & 0xff) != 0xff)
        slot->timing = count + (1ull << shift);
    count = slot->settings;
    if (ok && ((count >> shift) & 0xff) != 0xff)
        slot->settings = count + (1ull << shift);
#endif
    record_event(DdrSweepPower, ready_khz, (ddr_eye_t){event, ok}, 255, 0, get_tmr_us());
}

void ddr_trace_matrix(enum DdrMatrixStage state, unsigned phase, u32 detail)
{
    matrix_status = ((u64)detail << 32) | ((u64)phase << 8) | state;
}

void ddr_trace_matrix_transfer(bool reduced, u32 blocks, u64 elapsed)
{
    matrix_stats[reduced][0] += blocks;
    matrix_stats[reduced][1] += elapsed;
}

void ddr_trace_report(atmosphere_fatal_error_ctx *ctx)
{
    ctx->report_identifier = get_tmr_us();
#if defined(EMUMMC_DDR200_DLL_DIAG)
    ctx->gprs[0] = ctx->error_desc == 0xdd200066u ? 0x444452323030001aull : 0x444452323030001bull;
#elif defined(EMUMMC_DDR200_ADMA_PROBE)
    ctx->gprs[0] = 0x444452323030001aull;
#elif defined(EMUMMC_DDR200_COUNTED_READS)
    ctx->gprs[0] = 0x4444523230300015ull;
#elif defined(EMUMMC_DDR200_READ_AHEAD)
    ctx->gprs[0] = 0x4444523230300014ull;
#elif defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    ctx->gprs[0] = 0x4444523230300012ull;
#else
    ctx->gprs[0] = 0x444452323030000full;
#endif
    ctx->gprs[1] = checkpoint;
    ctx->gprs[2] = ((u64)reads << 32) | writes;
    ctx->gprs[3] = ((u64)reasserts << 32) | runtime_events;
    ctx->gprs[4] = ((u64)matrix_events << 32) | MatrixCount;
#if defined(EMUMMC_DDR200_ADMA_PROBE) && !defined(EMUMMC_DDR200_READ_AHEAD) && !defined(EMUMMC_DDR200_MATRIX)
    /* In ADMA2 reports without a matrix or read-ahead, the upper half is spare. */
    ctx->gprs[4] |= (u64)pad_recalibration << 32;
#endif
    ctx->gprs[5] = functions;
    ctx->gprs[6] = ((u64)io_errors << 32) | ready_khz;
    ctx->gprs[7] = last_failure[1];
    ctx->gprs[8] = last_failure[0];
    ctx->gprs[9] = first_failure[1];
    ctx->gprs[10] = first_failure[0];
    ctx->gprs[11] = ((u64)hygiene_attempts << 32) | ((u64)MIN(hygiene_passed, 0xffffffu) << 8) | hygiene_last;
    ctx->gprs[12] = ((u64)verify_attempts << 32) | ((u64)MIN(verify_passed, 0xffffu) << 16) | verify_pair;
    ctx->gprs[13] = ((u64)waiting_native << 32) | boot_power_recoveries;
    for (unsigned i = 0; i < 2; ++i) {
        ctx->gprs[14 + i * 2] = sd_stats[i][0];
        ctx->gprs[15 + i * 2] = sd_stats[i][1];
        ctx->gprs[18 + i * 2] = fs_stats[i][0];
        ctx->gprs[19 + i * 2] = fs_stats[i][1];
        ctx->gprs[22 + i] = ((u64)MIN(matrix_stats[i][1], 0xffffffffull) << 32) | matrix_stats[i][0];
    }
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    ctx->gprs[22] = (MIN(large_read_us, 0xffffffffull) << 32) | MIN(large_read_blocks, 0xffffffffull);
    ctx->gprs[23] = (MIN(large_read_dma_us, 0xffffffffull) << 32) | MIN(large_read_copied, 0xffffffffull);
#endif
    ddr_watchdog_report(&ctx->gprs[24]);
    ctx->gprs[28] = bus_registers;
    ctx->gprs[29] = clock_registers;
    ctx->gprs[30] = ladder | ((u64)function_status << 32);
    ctx->gprs[31] = matrix_status;
#ifdef EMUMMC_DDR200_COUNTED_READS
    ctx->gprs[5] |= (u64)MIN(counted_reads, 0xffffu) << 48;
    ctx->gprs[30] |= (u64)counted_read_flags << 24;
#endif
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    ctx->gprs[2] = ((u64)large_read_commands << 32) | large_read_restarts;
    ctx->gprs[30] |= (u64)MIN(write_errors, 0xffffu) << 48;
    ctx->gprs[31] = largest_read;
#endif
#ifdef EMUMMC_DDR200_READ_AHEAD
    ctx->gprs[4] = ((u64)MIN(read_ahead_fills[0], 0xffffu) << 48) |
                  ((u64)MIN(read_ahead_fills[1], 0xffffu) << 32);
    ctx->gprs[31] |= (u64)largest_fs_read << 32;
    ctx->pstate = map_attempts;
    ctx->far = ((u64)map_result << 32) | read_ahead_blocks;
#endif
    ctx->afsr0 = native_before >> 16;
    ctx->afsr1 = native_after >> 16;
    ctx->esr = (MIN(recovery_attempts, 0xffffu) << 16) | MIN(recovery_passed, 0xffffu);

    u64 data[HistoryCount][2];
    for (unsigned i = 0; i < HistoryCount; ++i) {
        data[i][0] = history[i].timing;
        data[i][1] = history[i].settings;
    }
#ifdef EMUMMC_DDR200_ADMA_PROBE
    data[HistoryCount - DmaCount + DdrDmaStats][1] = io_errors;
#endif
#ifdef EMUMMC_DDR200_ADMA_PROBE
    ctx->gprs[3] |= (u64)MIN(wake_events, 0xffffu) << 48;
    ctx->gprs[13] |= (u64)MIN(first_wake_failure, 0xffffu) << 48;
#endif
#ifdef EMUMMC_DDR200_DLL_DIAG
    if (ctx->error_desc != 0xdd200066u) {
        ctx->gprs[4] = ((u64)other_rows << 32) | dll_rows;
        ctx->gprs[31] = dll_state;
        u32 rows[DllRows + OtherRows][6];
        for (unsigned i = 0; i < DllRows + OtherRows; ++i)
            for (unsigned j = 0; j < 6; ++j)
                rows[i][j] = dll_history[i][j];
        memcpy(data, rows, sizeof(rows));
        ctx->stack_trace_size = 0;
        ctx->stack_dump_size = 0;
    }
#endif
    _Static_assert(sizeof(data) == sizeof(ctx->stack_trace) + sizeof(ctx->stack_dump) + sizeof(ctx->tls),
        "sweep trace capacity");
    memcpy(ctx->stack_trace, data, sizeof(ctx->stack_trace));
    memcpy(ctx->stack_dump, data + 16, sizeof(ctx->stack_dump));
    memcpy(ctx->tls, data + 32, sizeof(ctx->tls));
}
