/* SPDX-License-Identifier: GPL-2.0-only */
#include <string.h>
#include "ddr_trace.h"
#include "util.h"
#include "../soc/clock.h"
#include "../soc/pinmux.h"
#include "../soc/t210.h"

enum { SnapshotWords = 24 };
static u32 snapshots[DdrSnapshotCount][SnapshotWords];
static u32 first_maps[8], latest_maps[8];
static u32 stage, detail, reads, writes, attempts, sweeps;
static u32 first_window, latest_window, first_card[2], latest_card[2];
static u32 first_khz, latest_khz, ready_khz, io_errors;
static u32 last_full_window, last_reduced_window;
static u64 first_io_error[3], latest_io_error[3];

/* The SD and NAND mutexes serialize these calls. Fatal reports are captured
   on the failing thread, with no filesystem calls in the reporting path. */
void ddr_trace_stage(enum DdrStage next, u32 info)
{
    stage = next;
    detail = info;
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
    memcpy(snapshots[slot], values, sizeof(values));
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
    memcpy(latest_maps, raw, 4 * sizeof(u32));
    memcpy(latest_maps + 4, complete, 4 * sizeof(u32));
    if (++sweeps == 1) {
        first_window = latest_window;
        first_khz = khz;
        memcpy(first_maps, latest_maps, sizeof(first_maps));
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
    if (++io_errors == 1)
        memcpy(first_io_error, latest_io_error, sizeof(first_io_error));
}

void ddr_trace_reassert(void) { ++attempts; }

void ddr_trace_report(atmosphere_fatal_error_ctx *ctx)
{
    ctx->report_identifier = get_tmr_us();
    ctx->gprs[0] = 0x4444523230300007ull;
    ctx->gprs[1] = ((u64)stage << 32) | detail;
    ctx->gprs[2] = ((u64)reads << 32) | writes;
    ctx->gprs[3] = ((u64)attempts << 32) | sweeps;
    ctx->gprs[4] = ((u64)first_window << 32) | latest_window;
    ctx->gprs[5] = ((u64)first_card[0] << 32) | first_card[1];
    ctx->gprs[6] = ((u64)latest_card[0] << 32) | latest_card[1];
    ctx->gprs[7] = ((u64)first_khz << 32) | latest_khz;
    memcpy(&ctx->gprs[8], first_maps, sizeof(first_maps));
    memcpy(&ctx->gprs[12], latest_maps, sizeof(latest_maps));
    ctx->gprs[16] = ((u64)io_errors << 32) | ready_khz;
    memcpy(&ctx->gprs[17], first_io_error, sizeof(first_io_error));
    memcpy(&ctx->gprs[20], latest_io_error, sizeof(latest_io_error));
    ctx->gprs[23] = ((u64)last_full_window << 32) | last_reduced_window;
    /* Use reserved report space while leaving actual stack lengths zero. */
    _Static_assert(sizeof(snapshots) <= sizeof(ctx->stack_trace) + sizeof(ctx->stack_dump), "trace too large");
    memcpy(ctx->stack_trace, snapshots, sizeof(ctx->stack_trace));
    memcpy(ctx->stack_dump, (const u8 *)snapshots + sizeof(ctx->stack_trace),
        sizeof(snapshots) - sizeof(ctx->stack_trace));
}
