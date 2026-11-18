/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include "../utils/types.h"

enum DdrRate { DdrRateUnknown, DdrRateSdr104, DdrRateReduced, DdrRateFull };
enum {
    DdrTapCount = 128,
    DdrRuntimeWidth = 4,
    DdrTapMargin = 3,
    DdrVerifyBlocks = 512,
    DdrTrackBlocks = 512 * 1024,
    DdrTrackBusyUs = 2000000,
    DdrCoolIdleUs = 5000000,
    DdrCoolBlocks = 256 * 1024,
    DdrMaxUpgrades = 6,
};

typedef struct { u8 left, right; } ddr_eye_t;

typedef struct {
    ddr_eye_t eyes[3];
    u8 count;
} ddr_history_t;

typedef struct {
    ddr_history_t history[2];
    enum DdrRate rate;
    bool capable;
    bool ready;
    bool unstable;
    bool matrix_active;
    u8 upgrades;
    u8 drive;
    u16 drive_support;
    u16 power_support;
    u8 power;
    u32 blocks;
    u32 busy_us;
    u64 last_io;
    u64 retry_at;
} ddr_policy_t;

static inline unsigned ddr_eye_width(ddr_eye_t eye)
{
    return eye.left <= eye.right ? eye.right - eye.left + 1u : 0;
}

static inline bool ddr_tap_passes(const u32 map[4], unsigned tap)
{
    return (map[tap / 32] >> (tap % 32)) & 1u;
}

static inline ddr_eye_t ddr_first_eye(const u32 map[4], unsigned first, unsigned end, unsigned minimum)
{
    unsigned start = first;
    for (unsigned tap = first; tap <= end; ++tap) {
        if (tap < end && ddr_tap_passes(map, tap))
            continue;
        if (tap - start >= minimum)
            return (ddr_eye_t){start, tap - 1};
        start = tap + 1;
    }
    return (ddr_eye_t){DdrTapCount, 0};
}

static inline void ddr_remember_eye(ddr_history_t *history, ddr_eye_t eye)
{
    history->eyes[2] = history->eyes[1];
    history->eyes[1] = history->eyes[0];
    history->eyes[0] = eye;
    if (history->count < 3)
        ++history->count;
}

static inline int ddr_pick_tap(const ddr_history_t *history, const u32 tried[4])
{
    ddr_eye_t newest = history->eyes[0];
    if (ddr_eye_width(newest) < 2 * DdrTapMargin + 1)
        return -1;

    int left = newest.left + DdrTapMargin;
    int right = newest.right - DdrTapMargin;
    for (unsigned i = 1; i < history->count; ++i) {
        left = MAX(left, history->eyes[i].left + DdrTapMargin);
        right = MIN(right, history->eyes[i].right - DdrTapMargin);
    }
    if (left > right) {
        left = newest.left + DdrTapMargin;
        right = newest.right - DdrTapMargin;
    }

    /* A receding right edge is a measured trend. Leave an extra guard on
       that side while retaining the margin from each observed edge. */
    if (history->count >= 2) {
        int right_loss = (int)history->eyes[1].right - newest.right;
        if (right_loss >= 3)
            right = MIN(right, MAX(left, newest.right - 2 * right_loss));
    }

    int selected = -1, best_margin = -1;
    for (int tap = left; tap <= right; ++tap) {
        int margin = MIN(tap - left, right - tap);
        if (margin > best_margin && !ddr_tap_passes(tried, tap)) {
            best_margin = margin;
            selected = tap;
        }
    }
    return selected;
}

static inline enum DdrRate ddr_failure_next(enum DdrRate rate)
{
    return rate == DdrRateFull ? DdrRateReduced :
           rate == DdrRateReduced ? DdrRateSdr104 : DdrRateUnknown;
}

static inline u64 ddr_retry_delay(unsigned failures)
{
    return MIN((u64)DdrCoolIdleUs << MIN(failures, 6u), 300000000ull);
}

static inline bool ddr_should_upgrade(const ddr_policy_t *policy, u64 now)
{
    return policy->capable && policy->ready && policy->rate != DdrRateFull &&
        policy->upgrades < DdrMaxUpgrades && now >= policy->retry_at &&
        ((now - policy->last_io >= DdrCoolIdleUs) || policy->blocks >= DdrCoolBlocks);
}
