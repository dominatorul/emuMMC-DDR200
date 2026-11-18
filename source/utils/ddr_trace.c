/* SPDX-License-Identifier: GPL-2.0-only */
#include <string.h>
#include <stdatomic.h>
#include "ddr_trace.h"
#include "ddr_watchdog.h"
#include "util.h"
#include "../soc/clock.h"
#include "../soc/gpio.h"
#include "../soc/pinmux.h"
#include "../soc/t210.h"

enum { SnapshotWords = 24, SweepHistoryLength = 8 };
static _Atomic u32 snapshots[DdrSnapshotCount][SnapshotWords];
static _Atomic u32 power_pins[DdrSnapshotCount][3];
static _Atomic u32 first_maps[8], latest_maps[8];
static _Atomic u64 checkpoint;
static _Atomic u32 reads, writes, attempts, sweeps;
static _Atomic u32 first_window, latest_window, first_card[2], latest_card[2];
static _Atomic u32 first_khz, latest_khz, ready_khz, io_errors;
static _Atomic u32 last_full_window, last_reduced_window;
static _Atomic u32 boot_power_recoveries;
static _Atomic u64 first_io_error[3], latest_io_error[3];
static _Atomic u64 first_error_hw, latest_error_hw;
static _Atomic u32 read_recovery_attempts, read_recovery_successes, read_recovery_tap;
static _Atomic u32 sweep_history[SweepHistoryLength];
/* SD commands include both the SD filesystem and emulated NAND. The outer
   FS counters select only SD filesystem I/O, including mutex wait time. */
enum { PerfCalls, PerfBlocks, PerfElapsed, PerfDma, PerfBounce, PerfErrors, PerfFields };
static _Atomic u64 sd_perf[2][PerfFields], fs_perf[2][3];
static _Atomic u32 perf_clock[5];

/* SD operations serialize trace writes. Atomic reads let a watchdog capture
   the current trace even while a worker owns both storage mutexes. */
void ddr_trace_stage(enum DdrStage next, u32 info)
{
    atomic_store_explicit(&checkpoint, ((u64)next << 32) | info, memory_order_relaxed);
    if (next == DdrReady)
        ready_khz = info;
}

void ddr_trace_host(enum DdrSnapshot slot, const sdmmc_t *host)
{
    const bool live = host->regs && clock_sdmmc_is_not_reset_and_enabled(SDMMC_1);
    const t210_sdmmc_t *regs = host->regs;
    const u32 values[SnapshotWords] = {
        1u | (live ? 2u : 0u) | (host->t210b01 ? 4u : 0u),
        CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1),
        CLOCK(CLK_RST_CONTROLLER_PLLC4_BASE),
        CLOCK(CLK_RST_CONTROLLER_PLLC4_MISC),
        CLOCK(CLK_RST_CONTROLLER_RST_DEVICES_L),
        CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_L),
        APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL),
        APB_MISC(APB_MISC_GP_SDMMC1_CLK_LPBK_CONTROL),
        PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK),
        PINMUX_AUX(PINMUX_AUX_SDMMC1_CMD),
        PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT0),
        sdmmc_get_sd_power_enabled(),
        live ? ((u32)regs->hostctl2 << 16) | regs->clkcon : 0,
        live ? ((u32)regs->hostctl << 8) | regs->pwrcon : 0,
        live ? regs->venclkctl : 0,
        live ? regs->ventunctl0 : 0,
        live ? regs->ventunctl1 : 0,
        live ? regs->autocalcfg : 0,
        live ? regs->autocalsts : 0,
        live ? regs->sdmemcmppadctl : 0,
        live ? regs->veniotrimctl : 0,
        live ? regs->iospare : 0,
        live ? regs->venmiscctl : 0,
        live ? ((u32)regs->errintsts << 16) | regs->norintsts : 0,
    };
    for (u32 i = 1; i < SnapshotWords; ++i)
        atomic_store_explicit(&snapshots[slot][i], values[i], memory_order_relaxed);
    power_pins[slot][0] = gpio_get_pin_state(GPIO_PORT_E, GPIO_PIN_4);
    power_pins[slot][1] = PINMUX_AUX(PINMUX_AUX_DMIC3_CLK);
    power_pins[slot][2] = gpio_get_pin_state(GPIO_PORT_Z, GPIO_PIN_1);
    atomic_store_explicit(&snapshots[slot][0], values[0], memory_order_release);
}

void ddr_trace_tuning(const u32 raw[4], const u32 complete[4], u32 count, u32 tap, u32 width, u32 khz)
{
    /* 9 bits for count (128 is valid), then 8 for tap, then width. */
    latest_window = count | (tap << 9) | (width << 17);
    latest_khz = khz;
    if (khz == SDMMC_DDR200_RECOVERY_CARD_KHZ)
        last_reduced_window = latest_window;
    else
        last_full_window = latest_window;
    for (u32 i = 0; i < 4; ++i) {
        atomic_store_explicit(&latest_maps[i], raw[i], memory_order_relaxed);
        atomic_store_explicit(&latest_maps[i + 4], complete[i], memory_order_relaxed);
    }
    /* Each entry includes the reassert and failed-command counts when sampled. */
    sweep_history[sweeps % SweepHistoryLength] = tap | (width << 8) |
        ((u32)(khz == SDMMC_DDR200_RECOVERY_CARD_KHZ) << 16) |
        (MIN(attempts, 127u) << 17) | (MIN(io_errors, 255u) << 24);
    if (++sweeps == 1) {
        first_window = latest_window;
        first_khz = khz;
        for (u32 i = 0; i < 4; ++i) {
            atomic_store_explicit(&first_maps[i], raw[i], memory_order_relaxed);
            atomic_store_explicit(&first_maps[i + 4], complete[i], memory_order_relaxed);
        }
    }
}

void ddr_trace_card(const u8 *status, bool selected)
{
    /* CMD6 neutral query and SET response, one word each. Byte 16 holds
       command-system and access-mode fields. Bytes 10/11 advertise group 2. */
    u32 packed = ((u32)status[16] << 24) | ((u32)status[13] << 16) |
        ((u32)status[10] << 8) | status[11];
    latest_card[selected] = packed;
    if (!sweeps)
        first_card[selected] = packed;
}

void ddr_trace_io(bool write)
{
    if (write) ++writes;
    else ++reads;
}

void ddr_trace_io_error(const sdmmc_t *host, u32 command, u32 blocks)
{
    latest_io_error[0] = ((u64)host->command_failure << 32) | command;
    latest_io_error[1] = ((u64)host->divisor << 32) | host->command_irq;
    latest_io_error[2] = ((u64)host->regs->prnsts << 32) | blocks;
    latest_error_hw = ((u64)host->regs->venclkctl << 32) |
        ((u32)host->regs->hostctl2 << 16) | host->command_auto_stop_error;
    if (++io_errors == 1) {
        for (u32 i = 0; i < 3; ++i)
            first_io_error[i] = latest_io_error[i];
        first_error_hw = latest_error_hw;
    }
}

void ddr_trace_read_recovery(const sdmmc_t *host, bool succeeded)
{
    if (succeeded) {
        read_recovery_tap = (host->regs->venclkctl >> 16) & 0xFF;
        ++read_recovery_successes;
    } else {
        ++read_recovery_attempts;
    }
}

void ddr_trace_sd_perf(const sdmmc_t *host, bool write, bool succeeded, bool copied,
                       u32 blocks, u64 elapsed_us, u64 dma_us)
{
    _Atomic u64 *totals = sd_perf[write];
    u64 calls = atomic_fetch_add_explicit(&totals[PerfCalls], 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&totals[PerfElapsed], elapsed_us, memory_order_relaxed);
    atomic_fetch_add_explicit(&totals[PerfDma], dma_us, memory_order_relaxed);
    if (!succeeded) {
        atomic_fetch_add_explicit(&totals[PerfErrors], 1, memory_order_relaxed);
        return;
    }
    atomic_fetch_add_explicit(&totals[PerfBlocks], blocks, memory_order_relaxed);
    if (copied)
        atomic_fetch_add_explicit(&totals[PerfBounce], blocks, memory_order_relaxed);

    /* Sample concrete clock registers periodically and on clock changes. */
    if (!(calls & 255) || perf_clock[0] != host->divisor) {
        perf_clock[0] = host->divisor;
        perf_clock[1] = CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1);
        perf_clock[2] = CLOCK(CLK_RST_CONTROLLER_PLLC4_BASE);
        perf_clock[3] = ((u32)host->regs->hostctl2 << 16) | host->regs->clkcon;
        perf_clock[4] = host->regs->venclkctl;
    }
}

void ddr_trace_fs_perf(bool write, u32 blocks, u64 elapsed_us)
{
    atomic_fetch_add_explicit(&fs_perf[write][PerfCalls], 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&fs_perf[write][PerfBlocks], blocks, memory_order_relaxed);
    atomic_fetch_add_explicit(&fs_perf[write][PerfElapsed], elapsed_us, memory_order_relaxed);
}

void ddr_trace_reassert(void) { ++attempts; }

void ddr_trace_boot_power(void)
{
    ddr_trace_stage(DdrBootPower, ++boot_power_recoveries);
}

void ddr_trace_report(atmosphere_fatal_error_ctx *ctx)
{
    ctx->report_identifier = get_tmr_us();
    ctx->gprs[0] = 0x444452323030000Bull;
    ctx->gprs[1] = atomic_load_explicit(&checkpoint, memory_order_relaxed);
    ctx->gprs[2] = ((u64)reads << 32) | writes;
    ctx->gprs[3] = ((u64)attempts << 32) | sweeps;
    ctx->gprs[4] = ((u64)first_window << 32) | latest_window;
    ctx->gprs[5] = ((u64)first_card[0] << 32) | first_card[1];
    ctx->gprs[6] = ((u64)latest_card[0] << 32) | latest_card[1];
    ctx->gprs[7] = ((u64)first_khz << 32) | latest_khz;
    for (u32 i = 0; i < 4; ++i) {
        ctx->gprs[8 + i] = ((u64)first_maps[2 * i + 1] << 32) | first_maps[2 * i];
        ctx->gprs[12 + i] = ((u64)latest_maps[2 * i + 1] << 32) | latest_maps[2 * i];
    }
    ctx->gprs[16] = ((u64)io_errors << 32) | ready_khz;
    for (u32 i = 0; i < 3; ++i) {
        ctx->gprs[17 + i] = first_io_error[i];
        ctx->gprs[20 + i] = latest_io_error[i];
    }
    ctx->gprs[23] = ((u64)last_full_window << 32) | last_reduced_window;
    /* Use reserved report space while leaving actual stack lengths zero. */
    ddr_watchdog_report(&ctx->gprs[24]);
    ctx->gprs[28] = boot_power_recoveries;
    ctx->gprs[29] = first_error_hw;
    ctx->gprs[30] = latest_error_hw;
    ctx->gprs[31] = ((u64)read_recovery_tap << 56) |
        ((u64)MIN(read_recovery_successes, 0xFFFFFFu) << 32) | read_recovery_attempts;
    u32 saved[DdrSnapshotCount][SnapshotWords];
    for (u32 slot = 0; slot < DdrSnapshotCount; ++slot) {
        saved[slot][0] = atomic_load_explicit(&snapshots[slot][0], memory_order_acquire);
        for (u32 i = 1; i < SnapshotWords; ++i)
            saved[slot][i] = atomic_load_explicit(&snapshots[slot][i], memory_order_relaxed);
    }
    _Static_assert(sizeof(saved) <= sizeof(ctx->stack_trace) + sizeof(ctx->stack_dump), "trace too large");
    memcpy(ctx->stack_trace, saved, sizeof(ctx->stack_trace));
    memcpy(ctx->stack_dump, (const u8 *)saved + sizeof(ctx->stack_trace),
        sizeof(saved) - sizeof(ctx->stack_trace));
    /* The reserved TLS dump holds GPIO input, drive and pinmux state. */
    u32 pins[DdrSnapshotCount][3];
    for (u32 slot = 0; slot < DdrSnapshotCount; ++slot)
        for (u32 i = 0; i < 3; ++i)
            pins[slot][i] = atomic_load_explicit(&power_pins[slot][i], memory_order_relaxed);
    _Static_assert(sizeof(pins) <= sizeof(ctx->tls), "GPIO trace too large");
    memcpy(ctx->tls, pins, sizeof(pins));
    u64 performance[2 * PerfFields + 2 * 3];
    for (u32 dir = 0; dir < 2; ++dir) {
        for (u32 i = 0; i < PerfFields; ++i)
            performance[dir * PerfFields + i] = atomic_load_explicit(&sd_perf[dir][i], memory_order_relaxed);
        for (u32 i = 0; i < 3; ++i)
            performance[2 * PerfFields + dir * 3 + i] = atomic_load_explicit(&fs_perf[dir][i], memory_order_relaxed);
    }
    u32 clocks[5];
    for (u32 i = 0; i < 5; ++i)
        clocks[i] = atomic_load_explicit(&perf_clock[i], memory_order_relaxed);
    u32 history[SweepHistoryLength];
    for (u32 i = 0; i < SweepHistoryLength; ++i)
        history[i] = atomic_load_explicit(&sweep_history[i], memory_order_relaxed);
    _Static_assert(sizeof(pins) + sizeof(performance) + sizeof(clocks) + sizeof(history)
        <= sizeof(ctx->tls), "performance and sweep trace too large");
    memcpy(ctx->tls + sizeof(pins), performance, sizeof(performance));
    memcpy(ctx->tls + sizeof(pins) + sizeof(performance), clocks, sizeof(clocks));
    memcpy(ctx->tls + sizeof(pins) + sizeof(performance) + sizeof(clocks), history, sizeof(history));
}
