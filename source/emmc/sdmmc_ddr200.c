/* SPDX-License-Identifier: GPL-2.0-only */
#include <string.h>
#include "sdmmc.h"
#include "mmc.h"
#include "sd.h"
#include "../nx/cache.h"
#include "../utils/ddr_trace.h"
#include "../utils/util.h"

enum { DdrVerifyBytes = DdrVerifyBlocks * 512 };
static u8 verify_backup[DdrVerifyBytes];

static u32 tap_readback(const sdmmc_t *host)
{
    return (host->regs->venclkctl >> 16) & 0xff;
}

static void exclude_tap(u32 used[4], unsigned tap)
{
    if (tap < DdrTapCount)
        used[tap / 32] |= 1u << (tap % 32);
}

/* The caller owns both storage mutexes. A probe can temporarily use an FS
   DMA window as long as its complete contents are saved and restored. */
static char *borrow_dma(unsigned blocks, void *backup)
{
    if (!_current_accessor || !_current_accessor->parent)
        return NULL;
    int index = sdmmc_calculate_fitting_dma_index(_current_accessor, blocks);
    sdmmc_dma_buffer_t *pool = &_current_accessor->parent->dmaBuffers[index];
    size_t bytes = blocks * 512u;
    if (!pool->device_addr_buffer || !pool->device_addr_buffer_masked ||
        pool->device_addr_buffer_size < bytes)
        return NULL;
    armDCacheFlush(pool->device_addr_buffer, bytes);
    memcpy(backup, pool->device_addr_buffer, bytes);
    return pool->device_addr_buffer;
}

static void restore_dma(char *dma, const void *backup, size_t bytes)
{
    memcpy(dma, backup, bytes);
    armDCacheFlush(dma, bytes);
}

static u64 begin_budget(sdmmc_t *host, u64 budget)
{
    u64 old = host->deadline_us;
    host->deadline_us = get_tmr_us() + budget;
    if (old)
        host->deadline_us = MIN(old, host->deadline_us);
    return old;
}

static bool probe_read(sdmmc_storage_t *storage, char *dma, u32 sector, unsigned tap, u64 *hash)
{
    u32 done = 0;
    sdmmc_set_ddr200_tap(storage->sdmmc, tap);
    if (tap_readback(storage->sdmmc) != tap ||
        !sdmmc_storage_transfer(storage, &done, sector, DdrVerifyBlocks, dma, 0) ||
        done != DdrVerifyBlocks)
        return false;
    *hash = ddr_data_checksum((const u8 *)dma, DdrVerifyBytes);
    return true;
}

static int verification_neighbor(ddr_eye_t eye, int tap, int guard, const u32 used[4])
{
    int left = eye.left + guard;
    int right = eye.right - guard;
    if (tap - 1 >= left && tap - 1 <= right && !ddr_tap_passes(used, tap - 1))
        return tap - 1;
    if (tap + 1 >= left && tap + 1 <= right && !ddr_tap_passes(used, tap + 1))
        return tap + 1;
    return ddr_pick_tap_in_range(left, right, used, false);
}

static bool verify_tap(sdmmc_storage_t *storage, ddr_eye_t eye, u32 sector, unsigned tap,
    unsigned guard, u32 used[4])
{
    if (!storage->has_sector_access || storage->sec_cnt < DdrVerifyBlocks)
        return false;
    exclude_tap(used, tap);
    int neighbor = verification_neighbor(eye, tap, guard, used);
    /* A seven-tap eye has only one tap with a three-tap guard. The
       secondary data probe can use the next margin, leaving the primary guarded. */
    if (neighbor < 0)
        neighbor = verification_neighbor(eye, tap, guard - 1, used);
    if (neighbor < 0)
        return false;

    char *dma = borrow_dma(DdrVerifyBlocks, verify_backup);
    if (!dma)
        return false;
    sector = MIN(sector, storage->sec_cnt - DdrVerifyBlocks);
    storage->sdmmc->regs->hostctl2 |= SDHCI_CTRL_TUNED_CLK;
    u64 first, second;
    bool ok = probe_read(storage, dma, sector, tap, &first);
    if (ok) {
        exclude_tap(used, neighbor);
        ok = probe_read(storage, dma, sector, neighbor, &second) && first == second;
    }
    restore_dma(dma, verify_backup, sizeof(verify_backup));
    if (ok) {
        sdmmc_set_ddr200_tap(storage->sdmmc, tap);
        ok = tap_readback(storage->sdmmc) == tap;
    }
    ddr_trace_verify(tap, neighbor, ok);
    return ok;
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
/* Only a guarded tap can be installed. Other passing taps can supply the
   independent checksum or expose why the full-rate eye cannot be used. */
static unsigned verify_full_rate(sdmmc_storage_t *storage, const ddr_history_t *history,
    ddr_eye_t eye, u32 sector, unsigned guard, u32 tried[4])
{
    enum { MaxProbes = 6 };
    ddr_policy_t *policy = &storage->ddr;
    sdmmc_t *host = storage->sdmmc;
    char *dma = borrow_dma(DdrVerifyBlocks, verify_backup);
    if (!dma || !storage->has_sector_access || storage->sec_cnt < DdrVerifyBlocks) {
        policy->tune_failure = DdrDropScratch;
        return SDMMC_INVALID_TAP;
    }

    sector = MIN(sector, storage->sec_cnt - DdrVerifyBlocks);
    host->regs->hostctl2 |= SDHCI_CTRL_TUNED_CLK;
    u64 hashes[MaxProbes];
    u8 taps[MaxProbes];
    bool guarded[MaxProbes];
    unsigned good = 0, applied = SDMMC_INVALID_TAP, partner = SDMMC_INVALID_TAP;
    unsigned relaxed = SDMMC_INVALID_TAP, relaxed_partner = SDMMC_INVALID_TAP;
    bool mismatch = false;
    unsigned first_probe = 255, second_probe = 255;
    policy->tune_failure = DdrDropMargin;
    while (policy->tune_probes < MaxProbes && policy->rate == DdrRateFull) {
        if (host->deadline_us && get_tmr_us() >= host->deadline_us) {
            policy->tune_failure = DdrDropDeadline;
            break;
        }

        int tap = good && guarded[0] ? verification_neighbor(eye, taps[0], guard, tried) :
            ddr_pick_tap(history, tried, guard, true);
        if (tap < 0)
            tap = ddr_pick_tap_in_range(eye.left, eye.right, tried, false);
        if (tap < 0)
            break;

        exclude_tap(tried, tap);
        if (policy->tune_probes == 0)
            first_probe = tap;
        else if (policy->tune_probes == 1)
            second_probe = tap;
        ++policy->tune_probes;
        u64 hash;
        if (!probe_read(storage, dma, sector, tap, &hash)) {
            policy->tune_failure = policy->rate == DdrRateUnknown ? DdrDropLinkLost : DdrDropProbeIo;
            continue;
        }
        bool guarded_tap = tap >= eye.left + (int)guard && tap + (int)guard <= eye.right;
        for (unsigned i = 0; i < good; ++i) {
            if (hash != hashes[i]) {
                mismatch = true;
                continue;
            }
            if (guarded_tap && guarded[i]) {
                applied = taps[i];
                partner = tap;
                policy->narrow_pair = true;
                break;
            }
            if ((guarded_tap || guarded[i]) && relaxed == SDMMC_INVALID_TAP) {
                relaxed = guarded[i] ? taps[i] : tap;
                relaxed_partner = guarded[i] ? tap : taps[i];
            }
        }
        if (applied != SDMMC_INVALID_TAP || (relaxed != SDMMC_INVALID_TAP && policy->tune_probes >= 3))
            break;
        hashes[good] = hash;
        taps[good] = tap;
        guarded[good++] = guarded_tap;
    }
    restore_dma(dma, verify_backup, sizeof(verify_backup));
    if (applied == SDMMC_INVALID_TAP && policy->rate == DdrRateFull && policy->tune_failure != DdrDropDeadline) {
        applied = relaxed;
        partner = relaxed_partner;
    }
    if (applied == SDMMC_INVALID_TAP) {
        if (mismatch && policy->tune_failure != DdrDropDeadline && policy->tune_failure != DdrDropLinkLost)
            policy->tune_failure = DdrDropMismatch;
        if (policy->tune_probes)
            ddr_trace_verify(first_probe, second_probe, false);
        return applied;
    }

    sdmmc_set_ddr200_tap(host, applied);
    bool ok = tap_readback(host) == applied;
    ddr_trace_verify(applied, partner, ok);
    policy->tune_failure = ok ? DdrDropUnknown : DdrDropClockSetup;
    return ok ? applied : SDMMC_INVALID_TAP;
}
#endif

static ddr_eye_t scan_eye(sdmmc_storage_t *storage, const ddr_history_t *history,
                          bool local, u32 *raw, u32 *passing, u32 *samples)
{
    sdmmc_t *host = storage->sdmmc;
    unsigned first = 0, end = DdrTapCount;
    ddr_eye_t old = history->count ? history->eyes[0] : (ddr_eye_t){DdrTapCount, 0};
    if (local && ddr_eye_width(old)) {
        int center = (old.left + old.right) / 2;
        first = MAX(0, center - 16);
        end = MIN(DdrTapCount, center + 17);
    }
    *samples = sdmmc_scan_ddr200(host, first, end, raw, passing);
    ddr_eye_t eye = ddr_select_eye(passing, first, first + *samples, DdrRuntimeWidth, old);
    if (first || end < DdrTapCount) {
        if (ddr_eye_width(eye) && eye.left != first && eye.right + 1u < end && *samples == end - first)
            return eye;
        memset(raw, 0, sizeof(u32) * 4);
        memset(passing, 0, sizeof(u32) * 4);
        *samples = sdmmc_scan_ddr200(host, 0, DdrTapCount, raw, passing);
        return ddr_select_eye(passing, 0, *samples, DdrRuntimeWidth, old);
    }
    return eye;
}

static bool train_link(sdmmc_storage_t *storage, enum DdrRate rate, u8 kind, bool local,
                  u32 sector, int failed_tap, const u32 failed[4])
{
    ddr_policy_t *policy = &storage->ddr;
    sdmmc_t *host = storage->sdmmc;
    bool reduced = rate == DdrRateReduced;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    policy->tune_failure = DdrDropLinkLost;
    policy->tune_probes = 0;
    policy->tune_eye = (ddr_eye_t){DdrTapCount, 0};
#endif
    ddr_history_t *history = &policy->history[reduced];
    if (policy->rate == DdrRateUnknown)
        return false;
    bool clock_changed = policy->rate != rate;
    policy->rate = rate;
    if (clock_changed) {
        if (!sdmmc_setup_ddr200_clock(host, reduced)) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
            policy->tune_failure = DdrDropClockSetup;
#endif
            policy->rate = DdrRateUnknown;
            return false;
        }
        local = false;
    }
    u32 raw[4] = {0}, passing[4] = {0};
    u64 started = get_tmr_us();
#ifdef EMUMMC_DDR200_HEKATE
    /* Match Hekate's full sweep, TUNED_CLK pass map and midpoint. Horizon
       reuses a live link, so verify that midpoint before resuming FS I/O. */
    sdmmc_scan_ddr200(host, 0, DdrTapCount, raw, passing);
    ddr_eye_t eye = ddr_hekate_eye(raw);
    unsigned applied = SDMMC_INVALID_TAP;
    *history = (ddr_history_t){0};
    if (ddr_eye_width(eye))
        ddr_remember_eye(history, eye);
    if (ddr_eye_width(eye) >= SDMMC_SAMPLE_WIN_SIZE_MIN) {
        unsigned tap = (eye.left + eye.right) / 2;
        u32 tried[4] = {0};
        if (failed)
            memcpy(tried, failed, sizeof(tried));
        if (failed_tap >= 0)
            exclude_tap(tried, failed_tap);
        if (kind == DdrSweepInitial) {
            sdmmc_set_ddr200_tap(host, tap);
            if (sdmmc_storage_wait_ready(storage, get_tmr_us() + 100000))
                applied = tap;
        } else if (!ddr_tap_passes(tried, tap) &&
                   verify_tap(storage, eye, sector, tap, DdrTapMargin, tried)) {
            applied = tap;
        }
    }
    policy->last_sweep_us = get_tmr_us();
    policy->ready = applied != SDMMC_INVALID_TAP;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    policy->tune_eye = eye;
    policy->tune_failure = !ddr_eye_width(eye) ? DdrDropNoEye :
        ddr_eye_width(eye) < SDMMC_SAMPLE_WIN_SIZE_MIN ? DdrDropMargin : DdrDropProbeIo;
#endif
    ddr_trace_policy_sweep(host, eye, applied, kind, policy->drive, started);
    if (policy->ready)
        ddr_trace_stage(DdrReady, host->divisor);
    return policy->ready;
#else
    u32 samples;
    ddr_eye_t eye;
    if (kind == DdrSweepInitial) {
        *history = (ddr_history_t){0};
        samples = sdmmc_scan_ddr200(host, 0, DdrTapCount, raw, passing);
        eye = ddr_widest_eye(passing, samples);
    } else {
        eye = scan_eye(storage, history, local, raw, passing, &samples);
    }
    ddr_eye_t previous = history->eyes[0];
    bool had_eye = history->count != 0;
    if (ddr_eye_width(eye))
        ddr_remember_eye(history, eye);

    u32 prior_tap = tap_readback(host);
    bool tracking = kind == DdrSweepTrack && !clock_changed;
    bool runtime_full = rate == DdrRateFull && kind != DdrSweepInitial;
    unsigned width = ddr_eye_width(eye);
    if (rate == DdrRateFull)
        policy->last_sweep_us = get_tmr_us();
#ifdef EMUMMC_DDR200_ADMA_PROBE
    policy->tune_eye = eye;
    policy->tune_failure = width ? DdrDropMargin : DdrDropNoEye;
#endif
    bool narrow = runtime_full && width >= 2 * DdrNarrowMargin + 1 && width < 2 * DdrTapMargin + 1;
    unsigned guard = narrow ? DdrNarrowMargin : DdrTapMargin;
    unsigned applied = SDMMC_INVALID_TAP;
    bool verified_narrow = policy->ready && policy->narrow && had_eye &&
        previous.left == eye.left && previous.right == eye.right &&
        prior_tap == policy->narrow_verified_tap &&
        started - policy->narrow_verified_at < DdrNarrowVerifyUs;
    bool reuse = tracking && (!narrow || verified_narrow) && width >= 2 * guard + 1 &&
        prior_tap >= eye.left + guard && prior_tap <= eye.right - guard &&
        (!had_eye || previous.right < eye.right + 3u);
    if (reuse) {
        applied = prior_tap;
    } else if (width >= (kind == DdrSweepInitial ? SDMMC_SAMPLE_WIN_SIZE_MIN : DdrRuntimeWidth)) {
        u32 tried[4] = {0};
        if (failed)
            memcpy(tried, failed, sizeof(tried));
        if (failed_tap >= 0)
            exclude_tap(tried, failed_tap);

#ifdef EMUMMC_DDR200_ADMA_PROBE
        if (runtime_full && !host->handoff_sdma) {
            policy->narrow_pair = false;
            applied = verify_full_rate(storage, history, eye, sector, guard, tried);
        } else
#endif
        for (unsigned attempt = 0; attempt < 2; ++attempt) {
            int tap = ddr_pick_tap(history, tried, guard, runtime_full);
            if (tap < 0 || policy->rate == DdrRateUnknown)
                break;
            if (kind == DdrSweepInitial) {
                sdmmc_set_ddr200_tap(host, tap);
                host->regs->hostctl2 |= SDHCI_CTRL_TUNED_CLK;
                if (tap_readback(host) == (unsigned)tap && sdmmc_storage_wait_ready(storage, get_tmr_us() + 100000))
                    applied = tap;
                break;
            }
            if (verify_tap(storage, eye, sector, tap, guard, tried)) {
                applied = tap;
                break;
            }
        }
    }
    if (narrow && applied != SDMMC_INVALID_TAP && !reuse) {
        policy->narrow_verified_tap = applied;
        policy->narrow_verified_at = get_tmr_us();
    }
    ddr_trace_policy_sweep(host, eye, applied, kind, policy->drive, started);
    policy->ready = applied != SDMMC_INVALID_TAP;
    policy->narrow = policy->ready && narrow;
    if (policy->ready)
        ddr_trace_stage(DdrReady, host->divisor);
    return policy->ready;
#endif
}

static bool train(sdmmc_storage_t *storage, enum DdrRate rate, u8 kind, bool local,
                  u32 sector, int failed_tap)
{
    return train_link(storage, rate, kind, local, sector, failed_tap, NULL);
}

static void remember_functions(sdmmc_storage_t *storage, const u8 *status)
{
    storage->ddr.drive_support = (u16)status[8] << 8 | status[9];
    storage->ddr.power_support = (u16)status[6] << 8 | status[7];
    storage->ddr.drive = status[15] & 15;
    storage->ddr.power = status[15] >> 4;
    ddr_trace_functions(storage->ddr.drive_support, storage->ddr.power_support,
                        storage->ddr.drive, storage->ddr.power);
}

static bool check_ddr_switch(sdmmc_storage_t *storage, u8 status[64], u32 arg)
{
    return sdmmc_sd_switch(storage, status, arg) && (status[16] >> 4) == 14 &&
        ((unsigned)status[0] << 8 | status[1]) <= 800;
}

static bool enter_ddr(sdmmc_storage_t *storage, u8 status[64])
{
    /* Check and set function 14 in command-system group 2. Access group 1
       stays at SDR12. The status block uses the preceding signaling. */
    const u32 no_drive_change = 0x00ffffef;
    u32 arg = no_drive_change;
#if defined(EMUMMC_DDR200_PERF_TRIAL) && !defined(EMUMMC_DDR200_HEKATE)
    /* Try the advertised card drive settings while the bus is still SDR. */
    const u8 drives[] = {
#ifdef EMUMMC_DDR200_DRIVE_C
        SD_SET_DRIVER_TYPE_C,
#endif
        SD_SET_DRIVER_TYPE_A,
    };
    for (unsigned i = 0; i < sizeof(drives); ++i) {
        u8 drive = drives[i];
        u32 candidate = (no_drive_change & ~(15u << 8)) | ((u32)drive << 8);
        if ((storage->ddr.drive_support & (1u << drive)) &&
            check_ddr_switch(storage, status, candidate) && (status[15] & 15) == drive) {
            arg = candidate;
            break;
        }
    }
#endif
    if (arg == no_drive_change && !check_ddr_switch(storage, status, arg))
        return false;
    if (!sdmmc_sd_switch(storage, status, arg | 0x80000000) || (status[16] >> 4) != 14) {
        storage->ddr.rate = DdrRateUnknown;
        return false;
    }
    ddr_trace_card(status, true);
    remember_functions(storage, status);
    bool clock = sdmmc_setup_ddr200_clock(storage->sdmmc, false);
    storage->ddr.rate = clock ? DdrRateFull : DdrRateUnknown;
    return clock;
}

static bool enter_from_sdr104(sdmmc_storage_t *storage, u8 status[64])
{
    if (!sdmmc_setup_clock(storage->sdmmc, SDHCI_TIMING_UHS_SDR12) ||
        !sdmmc_sd_switch(storage, status, 0x80fffff0) || status[16] != 0) {
        storage->ddr.rate = DdrRateUnknown;
        return false;
    }
    return enter_ddr(storage, status);
}

static bool set_sdr104(sdmmc_storage_t *storage, u8 status[64])
{
    /* Even an eye too narrow for production can carry the CMD6 status
       block. Park the host at its center before changing both modes. */
    if (storage->ddr.rate >= DdrRateReduced) {
        const ddr_history_t *history = &storage->ddr.history[storage->ddr.rate == DdrRateReduced];
        if (history->count) {
            ddr_eye_t eye = history->eyes[0];
            sdmmc_set_ddr200_tap(storage->sdmmc, (eye.left + eye.right) / 2);
            storage->sdmmc->regs->hostctl2 |= SDHCI_CTRL_TUNED_CLK;
        }
    }

    /* Select standard command system and SDR104 in one SET. Only a
       CRC-checked reply confirming both groups lets us commit the mode. */
    if (!sdmmc_sd_switch(storage, status, 0x80ffff03) || status[16] != 3) {
        storage->ddr.rate = DdrRateUnknown;
        return false;
    }
    storage->ddr.rate = DdrRateSdr104;
    return sdmmc_setup_clock(storage->sdmmc, SDHCI_TIMING_UHS_SDR104) &&
        sdmmc_tuning_execute(storage->sdmmc, SDHCI_TIMING_UHS_SDR104, MMC_SEND_TUNING_BLOCK) &&
        sdmmc_storage_wait_ready(storage, get_tmr_us() + 100000);
}

static bool fallback(sdmmc_storage_t *storage, enum DdrRate from, u32 sector, u8 status[64])
{
    ddr_policy_t *policy = &storage->ddr;
    policy->ready = false;
#ifndef EMUMMC_FORCE_DDR200
    if (from == DdrRateFull) {
        policy->unstable = true;
        policy->retry_at = get_tmr_us() + ddr_retry_delay(policy->upgrades);
    }
    for (enum DdrRate next = ddr_failure_next(from);
         next != DdrRateUnknown && policy->rate != DdrRateUnknown;
         next = ddr_failure_next(next)) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
        ddr_trace_clock_drop(policy->rate, next, policy->tune_failure, policy->tune_probes, policy->tune_eye);
#endif
        ddr_trace_ladder(next, false, policy->unstable, policy->upgrades);
        if (next == DdrRateReduced) {
            if (!train(storage, next, DdrSweepRecovery, false, sector, -1))
                continue;
        } else {
            if (!set_sdr104(storage, status))
                break;
            policy->ready = true;
            storage->csd.busspeed = 104;
            ddr_trace_stage(DdrReady, storage->sdmmc->divisor);
        }
        policy->blocks = policy->busy_us = 0;
        ddr_trace_ladder(next, true, policy->unstable, policy->upgrades);
        return true;
    }
#endif
    ddr_trace_ladder(policy->rate, false, policy->unstable, policy->upgrades);
    return false;
}

bool sdmmc_ddr200_initial(sdmmc_storage_t *storage, const u8 *functions)
{
    u8 status[64];
    if (!(((u16)functions[10] << 8 | functions[11]) & SD_MODE_UHS_DDR200))
        return false;
    remember_functions(storage, functions);
    if (!enter_ddr(storage, status) ||
        !train(storage, DdrRateFull, DdrSweepInitial, false, 0, -1))
        return false;
    storage->ddr.capable = true;
    storage->csd.busspeed = 200;
    ddr_trace_ladder(DdrRateFull, true, false, 0);
    return true;
}

static bool probe_native_sdr104(sdmmc_storage_t *storage, u8 status[64])
{
    sdmmc_t *host = storage->sdmmc;
    u64 old = begin_budget(host, 120000);
    bool read = sdmmc_sd_switch(storage, status, 0x00ffffff);
    host->deadline_us = old;
    if (read)
        return (status[16] >> 4) == 0;

    /* Native wake can program the SDR104 mode with the tap it sampled
       while the physical link was DDR200. Recheck at a tuned SDR104 tap. */
    old = begin_budget(host, 200000);
    read = sdmmc_setup_clock(host, SDHCI_TIMING_UHS_SDR104) &&
        sdmmc_tuning_execute(host, SDHCI_TIMING_UHS_SDR104, MMC_SEND_TUNING_BLOCK) &&
        sdmmc_sd_switch(storage, status, 0x00ffffff);
    host->deadline_us = old;
    return read && (status[16] >> 4) == 0;
}

static bool train_retained_ddr(sdmmc_storage_t *storage, enum DdrRate rate)
{
    sdmmc_t *host = storage->sdmmc;
    u64 old = begin_budget(host, 500000);
    /* The software rate predates the host reset. Always program the clock. */
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    bool clock = sdmmc_setup_ddr200_clock(host, rate == DdrRateReduced);
    bool ready = clock && sdmmc_storage_wait_ready(storage, get_tmr_us() + 50000);
#ifdef EMUMMC_DDR200_HEKATE
    storage->ddr.rate = ready ? rate : DdrRateUnknown;
#else
    storage->ddr.rate = rate;
#endif
    bool verified = ready && train(storage, rate, DdrSweepHandoff, true, 0, -1);
    host->deadline_us = old;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    ddr_trace_wake(WakeRetained, rate, (clock ? 1 : 0) | (ready ? 2 : 0) | (verified ? 4 : 0));
#endif
    return verified;
}

static bool restore_retained_ddr(sdmmc_storage_t *storage, enum DdrRate requested, u8 status[64])
{
#ifndef EMUMMC_DDR200_HEKATE
    ddr_policy_t *policy = &storage->ddr;
#endif
    /* Horizon may have restored only the controller. Restore the running
       card's last rate before trying the existing reduced-rate recovery. */
    bool ok = train_retained_ddr(storage, requested);
#ifdef EMUMMC_DDR200_HEKATE
    ddr_trace_power(DdrPowerRetained, ok);
    return ok || fallback(storage, requested, 0, status);
#else
    if (!ok && requested == DdrRateFull) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
        ddr_trace_clock_drop(requested, DdrRateReduced, DdrDropWakeCheck, 0, (ddr_eye_t){DdrTapCount, 0});
#endif
        ok = train_retained_ddr(storage, DdrRateReduced);
        if (ok) {
            policy->unstable = true;
            policy->retry_at = get_tmr_us() + ddr_retry_delay(policy->upgrades);
        }
    }
    if (!ok)
        policy->rate = DdrRateUnknown;
    ddr_trace_power(DdrPowerRetained, ok);
    return ok;
#endif
}

bool sdmmc_ddr200_handoff(sdmmc_storage_t *storage)
{
    ddr_policy_t *policy = &storage->ddr;
    sdmmc_t *host = storage->sdmmc;
    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma)
        return false;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    /* Horizon owns the SD address space during the handoff. Use its
       existing DMA buffers until the card and host agree on the mode. */
    bool previous_sdma = host->handoff_sdma;
    host->handoff_sdma = true;
    policy->narrow_pair = false;
#endif
    u64 old = begin_budget(host, 1500000);
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    enum DdrRate requested = policy->resume_rate;
    if (requested == DdrRateUnknown && policy->unstable)
        requested = policy->rate;
    if (requested == DdrRateUnknown)
        requested = DdrRateFull;
    policy->rate = DdrRateSdr104;
    policy->ready = false;
    policy->blocks = policy->busy_us = 0;
    ddr_trace_reassert();

    bool want_ddr = policy->capable && requested >= DdrRateReduced;
    if (want_ddr)
        sdmmc_ddr200_restore_host_cfg(host);
    bool standard = probe_native_sdr104(storage, status);
    bool ok = false;
    if (standard && want_ddr) {
        remember_functions(storage, status);
        sdmmc_sd_set_current_limit(storage, policy->power_support, status);
        if (enter_from_sdr104(storage, status)) {
            /* A fresh CMD6 switch starts a new link. The pre-handoff
               samples can force the tap too far right as the eye contracts. */
            memset(policy->history, 0, sizeof(policy->history));
            ok = train(storage, requested, DdrSweepHandoff, false, 0, -1) ||
                fallback(storage, requested, 0, status);
        }
    } else if (standard) {
        /* Horizon has already completed its native SDR104 init. Reuse the
           standard timing while keeping our serialized emuMMC transport. */
        ok = sdmmc_setup_clock(host, SDHCI_TIMING_UHS_SDR104) &&
             sdmmc_tuning_execute(host, SDHCI_TIMING_UHS_SDR104, MMC_SEND_TUNING_BLOCK) &&
             sdmmc_storage_wait_ready(storage, get_tmr_us() + 100000);
        storage->csd.busspeed = 104;
    } else if (want_ddr) {
        ok = restore_retained_ddr(storage, requested, status);
    }
    if (!ok && standard && policy->rate == DdrRateSdr104)
        ok = set_sdr104(storage, status);
    restore_dma(dma, backup, sizeof(backup));
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_ENABLE);
    host->deadline_us = old;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    host->handoff_sdma = previous_sdma;
#endif
    policy->ready = ok;
    if (ok) {
        storage->csd.busspeed = policy->rate >= DdrRateReduced ? 200 : 104;
        policy->resume_rate = DdrRateUnknown;
    }
    ddr_trace_ladder(policy->rate, ok, policy->unstable, policy->upgrades);
    return ok;
}

bool sdmmc_ddr200_prepare_suspend(sdmmc_storage_t *storage)
{
    ddr_policy_t *policy = &storage->ddr;
    if (!policy->ready)
        return false;
    policy->resume_rate = policy->rate;
    if (policy->rate == DdrRateSdr104)
        return true;
    if (policy->rate < DdrRateReduced)
        return false;

    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma)
        return false;
    sdmmc_t *host = storage->sdmmc;
    u64 old = begin_budget(host, 350000);
    bool powersave = host->powersave_enabled;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    bool previous_sdma = host->handoff_sdma;
    host->handoff_sdma = true;
#endif
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    bool ok = sdmmc_storage_wait_ready(storage, get_tmr_us() + 50000) &&
        set_sdr104(storage, status);
    restore_dma(dma, backup, sizeof(backup));
    sdmmc_card_clock_powersave(host, powersave);
    host->deadline_us = old;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    host->handoff_sdma = previous_sdma;
#endif
    policy->ready = ok;
    if (ok) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
        ddr_trace_clock_drop(policy->resume_rate, DdrRateSdr104, DdrDropSuspend, 0, (ddr_eye_t){DdrTapCount, 0});
#endif
        storage->csd.busspeed = 104;
        ddr_trace_stage(DdrReady, host->divisor);
    }
    return ok;
}

bool sdmmc_ddr200_profile_ready(const sdmmc_storage_t *storage)
{
    return storage->ddr.ready;
}

static bool recover_io(sdmmc_storage_t *storage, u32 sector, bool degrade, const u32 failed[4])
{
    ddr_policy_t *policy = &storage->ddr;
    sdmmc_t *host = storage->sdmmc;
    enum DdrRate rate = policy->rate;
    if (!policy->ready || rate < DdrRateReduced)
        return false;
    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma)
        return false;

    int failed_tap = tap_readback(host);
    u64 old = begin_budget(host, 1500000);
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    ddr_trace_read_recovery(host, false);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (degrade) {
        policy->tune_failure = failed ? DdrDropRetryLimit : DdrDropDiagnostic;
        policy->tune_probes = 0;
        if (failed)
            for (unsigned i = 0; i < 4; ++i)
                policy->tune_probes += __builtin_popcount(failed[i]);
        policy->tune_eye = policy->history[rate == DdrRateReduced].count ?
            policy->history[rate == DdrRateReduced].eyes[0] : (ddr_eye_t){DdrTapCount, 0};
    }
#endif
    bool ok = (!degrade && train_link(storage, rate, DdrSweepRecovery, true, sector, failed_tap, failed)) ||
        fallback(storage, rate, sector, status);
    if (ok)
        ddr_trace_read_recovery(host, true);
    policy->blocks = policy->busy_us = 0;
    restore_dma(dma, backup, sizeof(backup));
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_ENABLE);
    host->deadline_us = old;
    return ok;
}

bool sdmmc_ddr200_recover(sdmmc_storage_t *storage, u32 sector, bool degrade)
{
    return recover_io(storage, sector, degrade, NULL);
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
bool sdmmc_ddr200_retry_io(sdmmc_storage_t *storage, u32 sector, u32 failed[4], bool exhausted)
{
    exclude_tap(failed, tap_readback(storage->sdmmc));
    return recover_io(storage, sector, exhausted, failed);
}
#endif

bool sdmmc_ddr200_before_io(sdmmc_storage_t *storage, u32 sector, bool writing)
{
    ddr_policy_t *policy = &storage->ddr;
#ifdef EMUMMC_DDR200_HEKATE
    return policy->ready;
#else
    sdmmc_t *host = storage->sdmmc;
    if (!policy->capable || !policy->ready)
        return policy->ready;

    u64 now = get_tmr_us();
    if (writing)
        policy->last_write = now;
    /* Narrow eyes have passed two read probes. A lower clock gives writes
       more margin because a partially programmed CMD25 cannot be retried. */
#ifndef EMUMMC_FORCE_DDR200
    bool guard_write = writing;
#else
    bool guard_write = false;
#endif
    bool lower_for_write = guard_write && policy->rate == DdrRateFull && policy->narrow;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    lower_for_write &= !policy->narrow_pair;
#endif
    unsigned track_blocks = policy->narrow ? DdrNarrowTrackBlocks : DdrTrackBlocks;
    unsigned track_time = policy->narrow ? DdrNarrowTrackBusyUs : DdrTrackBusyUs;
    bool track = policy->rate == DdrRateFull &&
        (policy->blocks >= track_blocks || policy->busy_us >= track_time);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    track |= policy->rate == DdrRateFull && ddr_sweep_stale(policy, now);
#endif
    bool upgrade = !writing && !policy->matrix_active && ddr_should_upgrade(policy, now);
    if (!track && !upgrade && !lower_for_write)
        return true;
    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma)
        return !lower_for_write;

    u64 old = begin_budget(host, 1500000);
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    if (track || lower_for_write) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
        if (!train(storage, DdrRateFull, lower_for_write ? DdrSweepRecovery : DdrSweepTrack, true, sector, -1)) {
            fallback(storage, DdrRateFull, sector, status);
        } else if (guard_write && policy->narrow && !policy->narrow_pair) {
            policy->tune_failure = DdrDropWriteMargin;
            fallback(storage, DdrRateFull, sector, status);
        }
#else
        if (lower_for_write || !train(storage, DdrRateFull, DdrSweepTrack, true, sector, -1) ||
            (guard_write && policy->narrow))
            fallback(storage, DdrRateFull, sector, status);
#endif
    } else {
        enum DdrRate was = policy->rate;
        ++policy->upgrades;
        policy->retry_at = get_tmr_us() + ddr_retry_delay(policy->upgrades);
        ddr_trace_ladder(DdrRateFull, false, policy->unstable, policy->upgrades);
        if (was != DdrRateSdr104 || enter_from_sdr104(storage, status)) {
            if (!train(storage, DdrRateFull, DdrSweepUpgrade, false, sector, -1))
                fallback(storage, DdrRateFull, sector, status);
            else
                storage->csd.busspeed = 200;
        } else {
            policy->ready = policy->rate == DdrRateSdr104 && set_sdr104(storage, status);
        }
        ddr_trace_ladder(policy->rate, policy->ready, policy->unstable, policy->upgrades);
    }
    policy->blocks = policy->busy_us = 0;
    restore_dma(dma, backup, sizeof(backup));
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_ENABLE);
    host->deadline_us = old;
    return policy->ready;
#endif
}

void sdmmc_ddr200_count_io(sdmmc_storage_t *storage, u32 blocks, u64 elapsed)
{
    storage->ddr.blocks = MIN((u64)storage->ddr.blocks + blocks, 0xffffffffu);
    storage->ddr.busy_us = MIN(storage->ddr.busy_us + elapsed, 0xffffffffu);
    storage->ddr.last_io = get_tmr_us();
}

#if defined(EMUMMC_DDR200_MATRIX) && !defined(EMUMMC_DDR200_DLL_DIAG)
enum MatrixPhase {
    MatrixNoise, MatrixGated, MatrixFree, MatrixLoad, MatrixIdle, MatrixDrive, MatrixSpeed, MatrixSettle
};
enum {
    MatrixSliceBlocks = 2 * 1024 * 1024 / 512,
    MatrixSweepBlocks = 8 * 1024 * 1024 / 512,
    MatrixLoadBlocks = 64 * 1024 * 1024 / 512,
    MatrixSpeedBlocks = 256 * 1024 * 1024 / 512,
    MatrixIdleUs = 30000000,
    MatrixIdleTimeoutUs = 45000000,
};
enum {
    MatrixNoDma = 1u << 0,
    MatrixFullError = 1u << 1,
    MatrixReducedError = 1u << 2,
    MatrixShortIdle = 1u << 3,
    MatrixDriveError = 1u << 4,
    MatrixUnavailable = 1u << 5,
    MatrixUnfinishedLoad = 1u << 6,
    MatrixSettleUnavailable = 1u << 7,
};

static struct {
    enum MatrixPhase phase;
    enum DdrRate original_rate;
    unsigned noise_count, drive;
    u32 blocks, details;
    u64 idle_begin;
    bool reduced, started;
} matrix;

static bool matrix_sweep(sdmmc_storage_t *storage, u8 kind, bool gated)
{
    sdmmc_t *host = storage->sdmmc;
    u32 raw[4] = {0}, passing[4] = {0}, samples;
    bool powersave = host->powersave_enabled;
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    host->ddr200_scan_gated = gated;
    u64 started = get_tmr_us();
    ddr_eye_t eye = scan_eye(storage, &storage->ddr.history[matrix.reduced],
                              false, raw, passing, &samples);
    host->ddr200_scan_gated = false;

    u32 tap = tap_readback(host);
    ddr_trace_policy_sweep(host, eye, tap, kind, storage->ddr.drive, started);
    sdmmc_card_clock_powersave(host, powersave ? SDMMC_POWER_SAVE_ENABLE : SDMMC_POWER_SAVE_DISABLE);
    return ddr_eye_width(eye) >= DdrRuntimeWidth && tap >= eye.left && tap <= eye.right;
}

static void matrix_settle(sdmmc_storage_t *storage, u64 switched_at)
{
    ddr_trace_policy_sweep(storage->sdmmc, (ddr_eye_t){DdrTapCount, 0},
                           tap_readback(storage->sdmmc), DdrSweepSettleStart,
                           storage->ddr.drive, switched_at);
    static const u32 waits[] = {0, 10000, 100000};
    for (unsigned trial = 0; trial < sizeof(waits) / sizeof(waits[0]); ++trial) {
        u64 now = get_tmr_us();
        if (now - switched_at < waits[trial])
            usleep(waits[trial] - (now - switched_at));
        matrix_sweep(storage, DdrSweepSettle0 + trial, false);
    }
}

static bool matrix_probe_cmd6(sdmmc_storage_t *storage)
{
    ddr_policy_t *policy = &storage->ddr;
    sdmmc_t *host = storage->sdmmc;
    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma) {
        matrix.details |= MatrixSettleUnavailable;
        return policy->ready;
    }

    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    if (sdmmc_storage_wait_ready(storage, get_tmr_us() + 50000)) {
        if (set_sdr104(storage, status) && enter_from_sdr104(storage, status))
            matrix_settle(storage, get_tmr_us());
        else
            matrix.details |= MatrixSettleUnavailable;

        if (policy->rate == DdrRateFull)
            train(storage, DdrRateFull, DdrSweepRecovery, true, 0, -1);
        else
            policy->ready = policy->rate == DdrRateSdr104 && set_sdr104(storage, status);
        if (!policy->ready && policy->rate >= DdrRateReduced)
            fallback(storage, policy->rate, 0, status);
    } else {
        matrix.details |= MatrixSettleUnavailable;
    }
    restore_dma(dma, backup, sizeof(backup));
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_ENABLE);
    return policy->ready;
}

static bool matrix_set_rate(sdmmc_storage_t *storage)
{
    enum DdrRate want = matrix.reduced ? DdrRateReduced : DdrRateFull;
    if (storage->ddr.rate == want && storage->ddr.ready)
        return true;
    if (storage->ddr.rate < DdrRateReduced)
        return false;
    sdmmc_card_clock_powersave(storage->sdmmc, SDMMC_POWER_SAVE_DISABLE);
    bool ready = train(storage, want, matrix.phase == MatrixIdle ? DdrSweepIdle : DdrSweepRecovery,
                       false, 0, -1);
    sdmmc_card_clock_powersave(storage->sdmmc, SDMMC_POWER_SAVE_ENABLE);
    ddr_trace_ladder(storage->ddr.rate, ready, storage->ddr.unstable, storage->ddr.upgrades);
    return ready;
}

static u32 matrix_read_slice(sdmmc_storage_t *storage, u32 count, bool timed)
{
    char *dma = borrow_dma(DdrVerifyBlocks, verify_backup);
    if (!dma || !storage->has_sector_access || storage->sec_cnt < matrix.blocks + count) {
        if (dma)
            restore_dma(dma, verify_backup, sizeof(verify_backup));
        matrix.details |= MatrixNoDma;
        return 0;
    }
    sdmmc_t *host = storage->sdmmc;
    u64 previous = begin_budget(host, 400000);
    u32 blocks = 0;
    while (blocks < count && get_tmr_us() < host->deadline_us) {
        u32 done = 0;
        u64 start = get_tmr_us();
        if (!sdmmc_storage_transfer(storage, &done, matrix.blocks + blocks, DdrVerifyBlocks, dma, 0) ||
            done != DdrVerifyBlocks)
            break;
        if (timed)
            ddr_trace_matrix_transfer(matrix.reduced, done, get_tmr_us() - start);
        blocks += done;
    }
    host->deadline_us = previous;
    restore_dma(dma, verify_backup, sizeof(verify_backup));
    storage->ddr.last_io = get_tmr_us();
    return blocks;
}

static bool matrix_drive_strength(sdmmc_storage_t *storage)
{
    ddr_policy_t *policy = &storage->ddr;
    while (matrix.drive < 4 && !(policy->drive_support & (1u << matrix.drive)))
        ++matrix.drive;
    if (matrix.drive >= 4)
        return false;
    unsigned candidate = matrix.drive++;
    unsigned original = policy->drive;
    u8 backup[512], status[64];
    char *dma = borrow_dma(1, backup);
    if (!dma) {
        matrix.details |= MatrixNoDma;
        return false;
    }
    bool wrote = candidate == original;
    if (!wrote)
        wrote = sdmmc_sd_switch(storage, status, 0x80fff0ff | (candidate << 8)) &&
            (status[15] & 15) == candidate;
    if (wrote) {
        policy->drive = candidate;
        if (!matrix_sweep(storage, DdrSweepDrive, false))
            matrix.details |= MatrixDriveError;
    } else {
        matrix.details |= MatrixDriveError;
    }
    /* A failed SET has an uncertain result, so still restore the original
       strength and let the normal timing policy revalidate if needed. */
    bool restore = candidate == original ||
        (sdmmc_sd_switch(storage, status, 0x80fff0ff | (original << 8)) &&
         (status[15] & 15) == original);
    if (restore) {
        policy->drive = original;
    } else {
        matrix.details |= MatrixDriveError;
        policy->ready = false;
    }
    restore_dma(dma, backup, sizeof(backup));
    return restore;
}

static bool finish_matrix(sdmmc_storage_t *storage, enum DdrMatrixStage state)
{
    ddr_policy_t *policy = &storage->ddr;
    policy->matrix_active = false;
    if (policy->rate == DdrRateReduced && matrix.original_rate == DdrRateFull &&
        !policy->unstable && policy->ready) {
        sdmmc_t *host = storage->sdmmc;
        u64 old = begin_budget(host, 1500000);
        sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
        if (!train(storage, DdrRateFull, DdrSweepUpgrade, false, 0, -1)) {
            u8 backup[512], status[64];
            char *dma = borrow_dma(1, backup);
            if (dma) {
                fallback(storage, DdrRateFull, 0, status);
                restore_dma(dma, backup, sizeof(backup));
            }
        }
        sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_ENABLE);
        host->deadline_us = old;
    }
    ddr_trace_matrix(state, matrix.phase | (matrix.reduced ? 0x80 : 0), matrix.details);
    return false;
}

static bool next_matrix_rate(sdmmc_storage_t *storage)
{
    if (matrix.phase == MatrixDrive || matrix.phase == MatrixSettle)
        return finish_matrix(storage, DdrMatrixComplete);
    if (matrix.reduced) {
        matrix.reduced = false;
        matrix.phase = MatrixDrive;
        return true;
    }
    matrix.reduced = true;
    matrix.phase = MatrixNoise;
    matrix.noise_count = matrix.blocks = 0;
    return true;
}

static bool matrix_failed_io(sdmmc_storage_t *storage)
{
    matrix.details |= matrix.reduced ? MatrixReducedError : MatrixFullError;
    if (storage->ddr.ready)
        sdmmc_ddr200_recover(storage, matrix.blocks, true);
    return storage->ddr.ready;
}

bool sdmmc_ddr200_matrix_step(sdmmc_storage_t *storage)
{
    sdmmc_t *host = storage->sdmmc;
    if (!matrix.started) {
        matrix.started = true;
        matrix.original_rate = storage->ddr.rate;
        storage->ddr.matrix_active = true;
    }
    ddr_trace_matrix(DdrMatrixRunning, matrix.phase | (matrix.reduced ? 0x80 : 0), matrix.details);

    if (matrix.phase == MatrixIdle) {
        u64 now = get_tmr_us();
        u64 last_activity = MAX(matrix.idle_begin, storage->ddr.last_io);
        if (now - last_activity < MatrixIdleUs) {
            if (now - matrix.idle_begin < MatrixIdleTimeoutUs)
                return true;
            matrix.details |= MatrixShortIdle;
        }
    }

    u64 old = begin_budget(host, 250000);
    if (!matrix_set_rate(storage)) {
        matrix.details |= MatrixUnavailable;
        if (storage->ddr.rate >= DdrRateReduced) {
            storage->ddr.ready = true;
            sdmmc_ddr200_recover(storage, 0, true);
        }
        host->deadline_us = old;
        return storage->ddr.ready ? next_matrix_rate(storage) : finish_matrix(storage, DdrMatrixInterrupted);
    }

    bool more = true;
    switch (matrix.phase) {
        case MatrixNoise:
            matrix_sweep(storage, DdrSweepNoise, false);
            if (++matrix.noise_count == 5)
                matrix.phase = MatrixGated;
            break;
        case MatrixGated:
            matrix_sweep(storage, DdrSweepGated, true);
            matrix.phase = MatrixFree;
            break;
        case MatrixFree:
            matrix_sweep(storage, DdrSweepFree, false);
            matrix.phase = MatrixLoad;
            break;
        case MatrixLoad:
        case MatrixSpeed: {
            bool timed = matrix.phase == MatrixSpeed;
            u32 target = timed ? MatrixSpeedBlocks : MatrixLoadBlocks;
            u32 count = MIN(MatrixSliceBlocks, target - matrix.blocks);
            u32 blocks = matrix_read_slice(storage, count, timed);
            matrix.blocks += blocks;
            if (!timed && storage->ddr.ready &&
                (blocks != count || matrix.blocks % MatrixSweepBlocks == 0))
                matrix_sweep(storage, DdrSweepLoad, false);
            if (blocks != count) {
                if (!timed)
                    matrix.details |= MatrixUnfinishedLoad;
                if (!matrix_failed_io(storage))
                    more = finish_matrix(storage, DdrMatrixInterrupted);
            }
            if (!more)
                break;
            if (blocks != count || matrix.blocks == target) {
                if (timed)
                    more = next_matrix_rate(storage);
                else {
                    matrix.phase = MatrixIdle;
                    matrix.idle_begin = get_tmr_us();
                }
                matrix.blocks = 0;
            }
            break;
        }
        case MatrixIdle:
            matrix_sweep(storage, DdrSweepIdle, false);
            matrix.phase = MatrixSpeed;
            break;
        case MatrixDrive:
            if (!matrix_drive_strength(storage))
                matrix.phase = MatrixSettle;
            if (!storage->ddr.ready) {
                storage->ddr.ready = true;
                if (!sdmmc_ddr200_recover(storage, 0, true))
                    more = finish_matrix(storage, DdrMatrixInterrupted);
            }
            break;
        case MatrixSettle:
            more = finish_matrix(storage, matrix_probe_cmd6(storage) ? DdrMatrixComplete : DdrMatrixInterrupted);
            break;
    }
    host->deadline_us = old;
    if (more)
        ddr_trace_matrix(DdrMatrixRunning, matrix.phase | (matrix.reduced ? 0x80 : 0), matrix.details);
    return more;
}

void sdmmc_ddr200_matrix_skip(sdmmc_storage_t *storage)
{
    storage->ddr.matrix_active = false;
    ddr_trace_matrix(DdrMatrixSkipped, matrix.phase | (matrix.reduced ? 0x80 : 0), matrix.details);
}
#endif


#ifdef EMUMMC_DDR200_DLL_DIAG
/* The baseline runs first. Calibration may update internal delay state;
   the writable control registers are restored by the calibration helper. */
static struct {
    unsigned samples, blocks;
    bool started, dll;
} dll_matrix;

void sdmmc_ddr200_matrix_restart(void)
{
    memset(&dll_matrix, 0, sizeof(dll_matrix));
}

static bool dll_matrix_finish(sdmmc_storage_t *storage, unsigned status)
{
    storage->ddr.matrix_active = false;
    ddr_trace_dll_result(status);
    return false;
}

static void dll_matrix_sweep(sdmmc_storage_t *storage, u8 kind)
{
    sdmmc_t *host = storage->sdmmc;
    u32 raw[4] = {0}, passing[4] = {0}, samples;
    bool saved = host->powersave_enabled;
    u32 tune0 = host->regs->ventunctl0, tune1 = host->regs->ventunctl1;
    u32 clock = host->regs->venclkctl;
    sdmmc_card_clock_powersave(host, SDMMC_POWER_SAVE_DISABLE);
    u64 started = get_tmr_us();
    u64 previous = begin_budget(host, 10000);
    ddr_eye_t eye = scan_eye(storage, &storage->ddr.history[0], true, raw, passing, &samples);
    host->deadline_us = previous;
    host->regs->ventunctl0 = tune0;
    host->regs->ventunctl1 = tune1;
    host->regs->venclkctl = clock;
    (void)host->regs->venclkctl;
    sdmmc_card_clock_powersave(host, saved);
    ddr_trace_policy_sweep(host, eye, tap_readback(host), kind, storage->ddr.drive, started);
}

bool sdmmc_ddr200_matrix_step(sdmmc_storage_t *storage)
{
    enum { LoadBlocks = 64 * 1024 * 1024 / 512, SweepBlocks = 8 * 1024 * 1024 / 512 };
    sdmmc_t *host = storage->sdmmc;
    if (storage->ddr.rate != DdrRateFull || !storage->ddr.ready ||
        (host->regs->hostctl2 & 7) != UHS_DDR50_BUS_SPEED ||
        !storage->has_sector_access || storage->sec_cnt < LoadBlocks)
        return dll_matrix_finish(storage, 6);
    if (!dll_matrix.started) {
        dll_matrix.started = true;
        storage->ddr.matrix_active = true;
        ddr_trace_dll_result(1);
    }
    if (dll_matrix.samples < 5) {
        dll_matrix_sweep(storage, dll_matrix.dll ? 34 : 32);
        ++dll_matrix.samples;
        return true;
    }
    char *dma = borrow_dma(DdrVerifyBlocks, verify_backup);
    if (!dma)
        return dll_matrix_finish(storage, 6);
    bool ok = true;
    for (unsigned i = 0; i < 8; ++i) {
        u32 done = 0;
        u64 previous = begin_budget(host, 10000);
        ok = sdmmc_storage_transfer(storage, &done, dll_matrix.blocks, DdrVerifyBlocks, dma, false) &&
            done == DdrVerifyBlocks;
        host->deadline_us = previous;
        if (!ok) break;
        dll_matrix.blocks += done;
    }
    restore_dma(dma, verify_backup, sizeof(verify_backup));
    if (!ok)
        return dll_matrix_finish(storage, 6);
    if (dll_matrix.blocks % SweepBlocks == 0)
        dll_matrix_sweep(storage, dll_matrix.dll ? 35 : 33);
    if (dll_matrix.blocks != LoadBlocks)
        return true;
    if (dll_matrix.dll)
        return dll_matrix_finish(storage, 5);

    int calibrated = sdmmc_ddr200_dll_probe(host);
    ddr_trace_dll_sweep(host, (ddr_eye_t){DdrTapCount, 0}, 36);
    if (calibrated <= 0)
        return dll_matrix_finish(storage, calibrated ? 4 : 3);
    dll_matrix.dll = true;
    dll_matrix.samples = dll_matrix.blocks = 0;
    ddr_trace_dll_result(2);
    return true;
}

void sdmmc_ddr200_matrix_skip(sdmmc_storage_t *storage)
{
    dll_matrix_finish(storage, 7);
}
#endif
