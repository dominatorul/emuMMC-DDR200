/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include "fatal.h"
#include "../emmc/sdmmc_driver.h"

enum DdrSnapshot { DdrInitialTune, DdrLatestTune, DdrBeforeOpen, DdrAfterOpen, DdrReassert, DdrSnapshotCount };
enum DdrStage { DdrBoot, DdrInit, DdrReady, DdrNativeOpen, DdrCheckState,
    DdrReadFunctions, DdrCheckFunction, DdrSetFunction, DdrSetClock, DdrSweep,
    DdrCheckStatus, DdrRead, DdrWrite, DdrSetSdr12, DdrDma, DdrNativePower,
    DdrBootPower };
enum DdrCommandFailure { DdrCommandClock = 1, DdrCommandInhibit, DdrCommandDmaSetup,
    DdrCommandResponse, DdrCommandTransfer, DdrCommandBusy };

void ddr_trace_stage(enum DdrStage stage, u32 detail);
void ddr_trace_host(enum DdrSnapshot slot, const sdmmc_t *host);
void ddr_trace_tuning(const u32 raw[4], const u32 complete[4], u32 count, u32 tap, u32 width, u32 khz);
void ddr_trace_card(const u8 *status, bool selected);
void ddr_trace_io(bool write);
void ddr_trace_io_error(const sdmmc_t *host, u32 command, u32 blocks);
void ddr_trace_read_recovery(const sdmmc_t *host, bool succeeded);
void ddr_trace_sd_perf(const sdmmc_t *host, bool write, bool succeeded, bool copied,
                       u32 blocks, u64 elapsed_us, u64 dma_us);
void ddr_trace_fs_perf(bool write, u32 blocks, u64 elapsed_us);
void ddr_trace_reassert(void);
void ddr_trace_boot_power(void);
void ddr_trace_report(atmosphere_fatal_error_ctx *ctx);
