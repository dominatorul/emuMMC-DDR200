/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include "../utils/types.h"

enum DdrRate { DdrRateUnknown, DdrRateSdr104, DdrRateReduced, DdrRateFull };
enum {
    DdrTapCount = 128,
    DdrRuntimeWidth = 4,
    DdrTapMargin = 3,
    DdrNarrowMargin = 2,
    DdrVerifyBlocks = 512,
    DdrTrackBlocks = 128 * 1024,
    DdrTrackBusyUs = 500000,
    DdrNarrowTrackBlocks = 64 * 1024,
    DdrNarrowTrackBusyUs = 250000,
    DdrNarrowVerifyUs = 2000000,
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
    enum DdrRate resume_rate;
    bool capable;
    bool ready;
    bool narrow;
    bool unstable;
    bool matrix_active;
    bool counted_reads_failed;
    u8 upgrades;
    u8 drive;
    u16 drive_support;
    u16 power_support;
    u8 power;
    u8 narrow_verified_tap;
    u32 blocks;
    u32 busy_us;
    u64 last_io;
    u64 last_write;
    u64 retry_at;
    u64 narrow_verified_at;
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

static inline int ddr_pick_tap_in_range(int left, int right, const u32 tried[4], bool prefer_left)
{
    int selected = -1, best_margin = -1;
    for (int tap = left; tap <= right; ++tap) {
        int score = prefer_left ? right - tap : MIN(tap - left, right - tap);
        if (score > best_margin && !ddr_tap_passes(tried, tap)) {
            best_margin = score;
            selected = tap;
        }
    }
    return selected;
}

static inline int ddr_pick_tap(const ddr_history_t *history, const u32 tried[4],
                               unsigned guard, bool prefer_left)
{
    ddr_eye_t newest = history->eyes[0];
    if (ddr_eye_width(newest) < 2 * guard + 1)
        return -1;

    int left = newest.left + guard;
    int right = newest.right - guard;
    for (unsigned i = 1; i < history->count; ++i) {
        left = MAX(left, history->eyes[i].left + (int)guard);
        right = MIN(right, history->eyes[i].right - (int)guard);
    }
    if (left > right) {
        left = newest.left + guard;
        right = newest.right - guard;
    }

    /* A receding right edge is a measured trend. Leave an extra guard on
       that side while retaining the margin from each observed edge. */
    if (history->count >= 2) {
        int right_loss = (int)history->eyes[1].right - newest.right;
        if (right_loss >= 3)
            right = MIN(right, MAX(left, newest.right - 2 * right_loss));
    }

    int selected = ddr_pick_tap_in_range(left, right, tried, prefer_left);
    if (selected >= 0)
        return selected;

    /* The intersection can contain only the failed tap. A fresh candidate
       still needs the same edge margin and the caller's data verification. */
    return ddr_pick_tap_in_range(newest.left + guard, newest.right - guard, tried, prefer_left);
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
        (!policy->last_write || now - policy->last_write >= DdrCoolIdleUs) &&
        ((now - policy->last_io >= DdrCoolIdleUs) || policy->blocks >= DdrCoolBlocks);
}
