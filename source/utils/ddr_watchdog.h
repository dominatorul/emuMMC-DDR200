/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include "types.h"
#include "util.h"

enum DdrOperation { DdrOpRead = 1, DdrOpWrite, DdrOpOpen, DdrOpClose };
enum DdrOpPhase { DdrOpEntered = 1, DdrOpWaitSd, DdrOpWaitNand, DdrOpRunning, DdrOpUnlock };

void ddr_watchdog_start(volatile const u32 *buttons);
u32 ddr_watchdog_enter(uintptr_t thread, enum DdrOperation op, u32 mmc_id);
void ddr_watchdog_phase(uintptr_t thread, enum DdrOpPhase phase);
void ddr_watchdog_leave(u32 slot);
void ddr_watchdog_report(u64 *out);
bool ddr_watchdog_volume_down(void);
#ifdef EMUMMC_DDR200_MATRIX
void ddr_watchdog_arm_matrix(void);
#endif
