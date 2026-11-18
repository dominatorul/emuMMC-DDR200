/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include "fatal.h"
#include "../emmc/sdmmc_driver.h"

enum DdrSnapshot { DdrInitialTune, DdrLatestTune, DdrBeforeOpen, DdrAfterOpen, DdrReassert, DdrSnapshotCount };
enum DdrStage { DdrBoot, DdrInit, DdrReady, DdrNativeOpen, DdrCheckState,
    DdrReadFunctions, DdrCheckFunction, DdrSetFunction, DdrSetClock, DdrSweep,
    DdrCheckStatus, DdrRead, DdrWrite, DdrSetSdr12, DdrDma, DdrNativePower,
    DdrBootPower, DdrCheckCardIdentity, DdrCheckDmaBuffers, DdrResetLink, DdrWaitForNative, DdrNativeClose };
enum DdrCommandFailure { DdrCommandClock = 1, DdrCommandInhibit, DdrCommandDmaSetup,
    DdrCommandResponse, DdrCommandTransfer, DdrCommandBusy };

void ddr_trace_stage(enum DdrStage stage, u32 detail);
void ddr_trace_host(enum DdrSnapshot slot, const sdmmc_t *host);
void ddr_trace_tuning(sdmmc_t *host, const u32 raw[4], const u32 complete[4],
                      u32 count, u32 tap, u32 width, u32 selected_tap);
void ddr_trace_card(const u8 *status, bool selected);
void ddr_trace_io(bool write);
void ddr_trace_io_error(const sdmmc_t *host, u32 command, u32 blocks);
void ddr_trace_read_recovery(const sdmmc_t *host, bool succeeded);
void ddr_trace_sd_perf(const sdmmc_t *host, bool write, bool succeeded, bool copied,
                       u32 blocks, u64 elapsed_us, u64 dma_us);
void ddr_trace_fs_perf(bool write, u32 blocks, u64 elapsed_us);
#ifdef EMUMMC_DDR200_READ_AHEAD
void ddr_trace_read_ahead_map(Result result);
void ddr_trace_read_ahead(u32 blocks);
void ddr_trace_read_ahead_fill(bool succeeded);
#endif
void ddr_trace_reassert(void);
void ddr_trace_boot_power(void);
void ddr_trace_report(atmosphere_fatal_error_ctx *ctx);

#include "../emmc/ddr200_policy.h"

enum DdrSweepKind {
    DdrSweepInitial, DdrSweepHandoff, DdrSweepTrack, DdrSweepRecovery, DdrSweepUpgrade, DdrSweepLadder,
    DdrSweepPower,
    DdrSweepNoise = 16, DdrSweepGated, DdrSweepFree,
    DdrSweepSettle0, DdrSweepSettle10, DdrSweepSettle100,
    DdrSweepLoad, DdrSweepIdle, DdrSweepDrive, DdrSweepSettleStart,
};
enum DdrMatrixStage {
    DdrMatrixPending, DdrMatrixRunning, DdrMatrixComplete, DdrMatrixSkipped, DdrMatrixInterrupted,
};
enum DdrPowerEvent {
    DdrPowerClosing, DdrPowerHandback, DdrPowerClosed, DdrPowerOpened, DdrPowerReasserted,
    DdrPowerRetained,
};

void ddr_trace_policy_sweep(const sdmmc_t *host, ddr_eye_t eye, unsigned tap, u8 kind, u8 drive, u64 started);
void ddr_trace_ladder(enum DdrRate rate, bool ready, bool unstable, unsigned upgrades);
void ddr_trace_verify(unsigned tap, unsigned second, bool passed);
void ddr_trace_hygiene(bool stopped, bool ready, bool reset);
void ddr_trace_functions(u16 drive, u16 power, u8 selected_drive, u8 selected_power);
void ddr_trace_wait_native(void);
void ddr_trace_power(enum DdrPowerEvent event, bool ok);
void ddr_trace_matrix(enum DdrMatrixStage state, unsigned phase, u32 detail);
void ddr_trace_matrix_transfer(bool reduced, u32 blocks, u64 elapsed);
