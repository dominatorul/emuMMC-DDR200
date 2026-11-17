/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include "../utils/types.h"

static inline u32 sdmmc_tuning_select_window(const u32 *map, u32 count, u32 *size)
{
	u32 run = 0, best_size = 0, best_tap = 0;
	for (u32 i = 0; i < count; ++i) {
		if (map[i / 32] & (1u << (i % 32))) {
			if (++run > best_size) {
				best_size = run;
				best_tap = i - run / 2;
			}
		} else {
			run = 0;
		}
	}
	*size = best_size;
	return best_tap;
}
