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
#ifdef EMUMMC_DDR200_MATRIX
    MatrixCount = 40,
#else
    MatrixCount = 0,
#endif
    RuntimeCount = HistoryCount - MatrixCount
};
typedef struct {
    _Atomic u64 timing;
    _Atomic u64 settings;
} sweep_event_t;

static sweep_event_t history[HistoryCount];
static u64 trace_started;
static _Atomic u32 runtime_events, matrix_events;
static _Atomic u32 reads, writes, reasserts, io_errors, ready_khz;
static _Atomic u32 boot_power_recoveries, waiting_native;
static _Atomic u32 verify_attempts, verify_passed, verify_pair;
static _Atomic u32 recovery_attempts, recovery_passed;
static _Atomic u32 hygiene_attempts, hygiene_passed, hygiene_last;
static _Atomic u32 function_status;
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
static _Atomic u64 bus_registers, clock_registers;
static _Atomic u64 native_before, native_after;
static _Atomic u32 sampled_rate;

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

void ddr_trace_io_error(const sdmmc_t *host, u32 cmd, u32 blocks)
{
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_MATRIX)
    if (cmd == MMC_WRITE_MULTIPLE_BLOCK)
        ++write_errors;
#endif
    u32 tap = (host->regs->venclkctl >> 16) & 0xff;
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
#if defined(EMUMMC_DDR200_READ_AHEAD)
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
    _Static_assert(sizeof(data) == sizeof(ctx->stack_trace) + sizeof(ctx->stack_dump) + sizeof(ctx->tls),
        "sweep trace capacity");
    memcpy(ctx->stack_trace, data, sizeof(ctx->stack_trace));
    memcpy(ctx->stack_dump, data + 16, sizeof(ctx->stack_dump));
    memcpy(ctx->tls, data + 32, sizeof(ctx->tls));
}
