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
#ifdef EMUMMC_DDR200_COUNTED_READS
void ddr_trace_counted_read(u8 version, bool supported, bool disabled, bool completed);
#endif
#ifdef EMUMMC_DDR200_READ_AHEAD
void ddr_trace_read_ahead_map(Result result);
void ddr_trace_read_ahead(u32 blocks);
void ddr_trace_read_ahead_fill(bool succeeded);
#endif
void ddr_trace_reassert(void);
void ddr_trace_boot_power(void);
void ddr_trace_report(atmosphere_fatal_error_ctx *ctx);

#include "../emmc/ddr200_policy.h"

enum DdrBatchRefusal { DdrBatchEye, DdrBatchTapPair, DdrBatchRunIn, DdrBatchSmall,
    DdrBatchNonAdjacent, DdrBatchStale, DdrBatchUnavailable, DdrBatchWindow, DdrBatchRefusals };
enum DdrDropReason { DdrDropUnknown, DdrDropNoEye, DdrDropMargin, DdrDropScratch,
    DdrDropProbeIo, DdrDropMismatch, DdrDropDeadline, DdrDropClockSetup, DdrDropWriteMargin,
    DdrDropDiagnostic, DdrDropWakeCheck, DdrDropSuspend, DdrDropLinkLost, DdrDropRetryLimit };

#ifdef EMUMMC_DDR200_ADMA_PROBE
enum DdrDmaSample { DdrDmaPower, DdrDmaBaseline, DdrDmaWriteTotals,
    DdrDmaMappings, DdrDmaAlternate, DdrDmaHandoff, DdrDmaTotals, DdrDmaHost,
    DdrDmaRefusals, DdrDmaSequential = 12,
    DdrDmaStats, DdrDmaLastDrop, DdrDmaFailures, DdrDmaFailureCount = 8,
    DdrDmaSamples = DdrDmaFailures + DdrDmaFailureCount };
enum DdrDmaResult { DdrDmaOk = 1, DdrDmaFailed, DdrDmaMismatch };

void ddr_trace_dma_begin(void);
void ddr_trace_dma_sample(enum DdrDmaSample sample, const sdmmc_t *host, ddr_eye_t eye,
    u64 elapsed, u32 blocks, u32 resumes, enum DdrDmaResult result);
void ddr_trace_sdma_restart(bool handoff);
void ddr_trace_handoff_sdma(void);
enum DdrWakeStep { WakeSuspend, WakeCmd6, WakeRetire, WakeClose, WakeOpen, WakeHost, WakePower,
    WakeReassert, WakeMap, WakeCommand, WakeRetry, WakeFailed, WakeFirstFailure, WakePll, WakeRetained };
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result);
void ddr_trace_wake_host(const sdmmc_t *host);
#ifdef EMUMMC_DDR200_DLL_DIAG
void ddr_trace_dll_sweep(const sdmmc_t *host, ddr_eye_t eye, u8 kind);
void ddr_trace_dll_result(unsigned status);
#endif
void ddr_trace_descriptor_mapping(bool unmap, u32 das, Result result);
void ddr_trace_batch_refused(enum DdrBatchRefusal reason);
void ddr_trace_batch_seq(u32 blocks, bool served);
void ddr_trace_clock_drop(enum DdrRate from, enum DdrRate to, enum DdrDropReason reason,
    u8 probes, ddr_eye_t eye);
void ddr_trace_pad_recalibration(ddr_eye_t before, ddr_eye_t after, bool used_fallback);
void ddr_trace_short_counted_read(bool succeeded);
#endif

enum DdrSweepKind {
    DdrSweepInitial, DdrSweepHandoff, DdrSweepTrack, DdrSweepRecovery, DdrSweepUpgrade, DdrSweepLadder,
    DdrSweepPower, DdrSweepClockDrop,
    DdrSweepNoise = 16, DdrSweepGated, DdrSweepFree,
    DdrSweepSettle0, DdrSweepSettle10, DdrSweepSettle100,
    DdrSweepLoad, DdrSweepIdle, DdrSweepDrive, DdrSweepSettleStart,
};
enum DdrMatrixStage {
    DdrMatrixPending, DdrMatrixRunning, DdrMatrixComplete, DdrMatrixSkipped, DdrMatrixInterrupted,
};
enum DdrPowerEvent {
    DdrPowerClosing, DdrPowerHandback, DdrPowerClosed, DdrPowerOpened, DdrPowerReasserted,
    DdrPowerRetained, DdrPowerReasserting, DdrPowerWaiting,
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
