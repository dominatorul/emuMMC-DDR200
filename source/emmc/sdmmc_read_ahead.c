/* SPDX-License-Identifier: GPL-2.0-only */
#ifdef EMUMMC_DDR200_READ_AHEAD
#include <string.h>
#include "sdmmc.h"
#include "../nx/cache.h"
#include "../nx/svc.h"
#include "../utils/ddr_trace.h"
#include "../utils/util.h"

enum {
    ReadAheadBytes = 2 * 1024 * 1024,
    ReadAheadBlocks = ReadAheadBytes / 512,
    ReadAheadRunIn = 512 * 1024 / 512,
#ifdef EMUMMC_DDR200_ADMA_PROBE
    ReadAheadRequest = 64 * 1024 / 512,
    ReadAheadSmallSeed = 16 * 1024 / 512,
    ReadAheadLargeRunIn = 256 * 1024 / 512,
#else
    ReadAheadRequest = 256 * 1024 / 512,
    ReadAheadMinEye = 9,
#endif
    DeviceAlignment = 4 * 1024 * 1024,
    ReadStreams = 4,
    ReadStreamIdleUs = 1000000,
    ReadAheadCooldownUs = 60000000,
    ReadAheadStrikeLimit = 3,
};

static struct { _Alignas(0x1000) u8 data[ReadAheadBytes]; } read_window;
typedef struct { Handle das; u64 dma; bool failed; } dma_window_t;
typedef struct { u64 next, seen; u32 blocks; } read_stream_t;
#ifdef EMUMMC_DDR200_ADMA_PROBE
static union {
    _Alignas(0x1000) u8 page[0x1000];
    sdmmc_adma2_descriptor_t entries[SdmmcAdmaDescriptors];
} descriptor_table;
_Static_assert(sizeof(descriptor_table) == 0x1000, "ADMA table page");
static dma_window_t descriptor_mapping;
extern volatile Handle *sdmmc_das_handle;
#endif
static const Handle current_process = 0xFFFF8001u;

/* Both FS storage mutexes protect the cache and its device mapping. */
static struct {
    dma_window_t mapping;
    read_stream_t streams[ReadStreams];
    u32 first;
    u32 blocks;
    u64 retry_at;
    u8 strikes, reenables;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    bool adma_verified;
    u8 verified_tap[2];
    u8 verified_upgrade;
#endif
} read_ahead;

static bool map_window(dma_window_t *mapping, Handle das, void *buffer, size_t bytes, u32 permission)
{
    if (mapping->dma)
        return mapping->das == das;
    if (!das || mapping->failed)
        return false;

    /* The aligned mapping call requires equal 4 MiB offsets. The kernel
       checks the device aperture and rejects any occupied range. */
    const u64 bases[] = {0x04000000, 0x10000000, 0x20000000};
    u64 address = (uintptr_t)buffer;
    for (unsigned i = 0; i < sizeof(bases) / sizeof(bases[0]); ++i) {
        u64 dma = bases[i] | (address & (DeviceAlignment - 1));
        Result rc = svcMapDeviceAddressSpaceAligned(das, current_process, address, bytes,
            dma, permission);
        ddr_trace_read_ahead_map(rc);
#ifdef EMUMMC_DDR200_ADMA_PROBE
        if (mapping == &descriptor_mapping) {
            ddr_trace_descriptor_mapping(false, das, rc);
            ddr_trace_wake(WakeMap, das, rc);
        }
#endif
        if (!rc) {
            mapping->das = das;
            mapping->dma = dma;
            return true;
        }
    }
    mapping->failed = true;
    return false;
}

static void unmap_window(dma_window_t *mapping, void *buffer, size_t bytes)
{
    if (!mapping->dma)
        return;
    Result rc = svcUnmapDeviceAddressSpace(mapping->das, current_process, (uintptr_t)buffer,
        bytes, mapping->dma);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (mapping == &descriptor_mapping)
        ddr_trace_descriptor_mapping(true, mapping->das, rc);
#endif
#ifdef EMUMMC_DDR200_ADMA_PROBE
    ddr_trace_wake(WakeRetire, mapping == &read_ahead.mapping ? 0 : 1, rc);
#endif
    mapping->dma = 0;
    mapping->failed = rc != 0;
    if (!rc)
        mapping->das = 0;
}

u64 sdmmc_read_ahead_dma(const void *buf, u32 blocks)
{
    uintptr_t offset = (uintptr_t)buf - (uintptr_t)read_window.data;
    if (!read_ahead.mapping.dma || !blocks || offset >= ReadAheadBytes ||
        (u64)blocks * 512 > ReadAheadBytes - offset)
        return 0;
    return read_ahead.mapping.dma + offset;
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
bool sdmmc_adma_remap(void)
{
    unmap_window(&descriptor_mapping, &descriptor_table, sizeof(descriptor_table));
    if (!sdmmc_das_handle || !*sdmmc_das_handle)
        return false;
    /* A new live DAS gets its own validated mapping. A failed unmap on
       the same handle leaves the old mapping quarantined. */
    if (descriptor_mapping.das == 0 || descriptor_mapping.das != *sdmmc_das_handle)
        descriptor_mapping.failed = false;
    return map_window(&descriptor_mapping, *sdmmc_das_handle,
        &descriptor_table, sizeof(descriptor_table), Perm_R);
}

u64 sdmmc_adma_table(u64 dma, u32 bytes)
{
    if (!dma || !bytes || (dma & 7) || bytes > SdmmcAdmaBytes || dma + bytes < dma ||
        !sdmmc_das_handle ||
        (descriptor_mapping.failed && descriptor_mapping.das != *sdmmc_das_handle ?
            !sdmmc_adma_remap() : !map_window(&descriptor_mapping, *sdmmc_das_handle,
            &descriptor_table, sizeof(descriptor_table), Perm_R)))
        return 0;

    unsigned count = 0;
    while (bytes) {
        u16 length = MIN(bytes, SdmmcAdmaChunk);
        descriptor_table.entries[count++] = (sdmmc_adma2_descriptor_t){
            .attributes = SdmmcAdmaTransferValid | (bytes == length ? SdmmcAdmaEnd : 0),
            .length = length, .address_low = (u32)dma, .address_high = (u32)(dma >> 32),
        };
        bytes -= length;
        dma += length;
    }
    armDCacheClean(descriptor_table.entries, count * sizeof(descriptor_table.entries[0]));
    return descriptor_mapping.dma;
}
#endif

void sdmmc_read_ahead_invalidate(u32 sector, u32 blocks)
{
    if (read_ahead.blocks && blocks &&
        (u64)sector + blocks > read_ahead.first &&
        sector < (u64)read_ahead.first + read_ahead.blocks) {
        read_ahead.blocks = 0;
        memset(read_ahead.streams, 0, sizeof(read_ahead.streams));
    }
}

void sdmmc_read_ahead_reset(void)
{
    read_ahead.blocks = 0;
    memset(read_ahead.streams, 0, sizeof(read_ahead.streams));
#ifdef EMUMMC_DDR200_ADMA_PROBE
    read_ahead.adma_verified = false;
    unmap_window(&descriptor_mapping, &descriptor_table, sizeof(descriptor_table));
    if (!descriptor_mapping.das)
        descriptor_mapping.failed = false;
#endif
    unmap_window(&read_ahead.mapping, &read_window, sizeof(read_window));
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
static unsigned active_tap(const sdmmc_t *host)
{
    return (host->regs->venclkctl >> 16) & 0xff;
}

static unsigned adma_guard(ddr_eye_t eye)
{
    return ddr_eye_width(eye) >= 2 * DdrTapMargin + 2 ? DdrTapMargin : DdrNarrowMargin;
}

static bool guarded_tap(ddr_eye_t eye, unsigned tap, unsigned guard)
{
    return tap >= eye.left + guard && tap + guard <= eye.right;
}

static bool transfer(sdmmc_storage_t *storage, u32 sector, u64 *elapsed)
{
    sdmmc_t *host = storage->sdmmc;
    bool previous = host->prefetch;
    u32 done = 0;
    u64 start = get_tmr_us();
    host->prefetch = true;
    bool ok = sdmmc_storage_transfer(storage, &done, sector, ReadAheadBlocks, read_window.data, false);
    host->prefetch = previous;
    *elapsed = get_tmr_us() - start;
    if (ok && done == ReadAheadBlocks) {
        sdmmc_ddr200_count_io(storage, done, *elapsed);
        return true;
    }
    return false;
}

static bool data_command_failed(const sdmmc_t *host)
{
    u16 errors = host->command_irq >> 16;
    return host->command_failure >= DdrCommandResponse &&
        (errors ? (errors & ~SDHCI_ERR_INT_ADMA_ERROR) != 0 : !(host->command_irq & SDHCI_INT_DMA_END));
}

static bool qualify_adma(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye, unsigned tap, unsigned alternate)
{
    sdmmc_t *host = storage->sdmmc;
    u64 elapsed;
    ddr_trace_dma_begin();
    memset(read_window.data, 0xa5, ReadAheadBytes);
    bool ok = transfer(storage, sector, &elapsed);
    ddr_trace_dma_sample(DdrDmaBaseline, host, eye, elapsed, ReadAheadBlocks, 0,
        ok ? DdrDmaOk : DdrDmaFailed);
    if (!ok) {
        if (data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }
    u64 hash = ddr_data_checksum(read_window.data, ReadAheadBytes);
    memset(read_window.data, (u8)~read_window.data[0], ReadAheadBytes);
    sdmmc_set_ddr200_tap(host, alternate);
    if (active_tap(host) != alternate) {
        sdmmc_set_ddr200_tap(host, tap);
        return false;
    }
    ok = transfer(storage, sector, &elapsed);
    enum DdrDmaResult result = !ok ? DdrDmaFailed :
        (ddr_data_checksum(read_window.data, ReadAheadBytes) == hash ? DdrDmaOk : DdrDmaMismatch);
    ddr_trace_dma_sample(DdrDmaAlternate, host, eye, elapsed, ReadAheadBlocks, 0, result);
    if (result != DdrDmaOk) {
        if (result == DdrDmaFailed && data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        else
            sdmmc_set_ddr200_tap(host, tap);
        return false;
    }

    sdmmc_set_ddr200_tap(host, tap);
    if (active_tap(host) != tap)
        return false;
    read_ahead.adma_verified = true;
    read_ahead.verified_tap[0] = tap;
    read_ahead.verified_tap[1] = alternate;
    read_ahead.verified_upgrade = storage->ddr.upgrades;
    return true;
}

static bool fill_adma(sdmmc_storage_t *storage, u32 sector)
{
    ddr_eye_t eye = storage->ddr.history[0].eyes[0];
    unsigned tap = active_tap(storage->sdmmc);
    unsigned guard = adma_guard(eye);
    unsigned a = read_ahead.verified_tap[0], b = read_ahead.verified_tap[1];
    if (!read_ahead.adma_verified || read_ahead.verified_upgrade != storage->ddr.upgrades ||
        (tap != a && tap != b) || !guarded_tap(eye, a, guard) || !guarded_tap(eye, b, guard)) {
        unsigned alternate = guarded_tap(eye, tap - 1, guard) ? tap - 1 : tap + 1;
        bool ok = qualify_adma(storage, sector, eye, tap, alternate);
        if (!ok)
            ddr_trace_batch_refused(DdrBatchTapPair);
        return ok;
    }

    u64 elapsed;
    if (transfer(storage, sector, &elapsed))
        return true;
    if (data_command_failed(storage->sdmmc))
        sdmmc_ddr200_recover(storage, sector, false);
    return false;
}
#endif

static bool read_ahead_eye_ready(const sdmmc_storage_t *storage)
{
    const ddr_history_t *history = &storage->ddr.history[0];
    const sdmmc_t *host = storage->sdmmc;
    if (storage->ddr.rate != DdrRateFull || host->divisor != 199680 || !history->count)
        return false;
    ddr_eye_t eye = history->eyes[0];
#ifdef EMUMMC_DDR200_ADMA_PROBE
    unsigned guard = adma_guard(eye);
    return sdmmc_adma2_supported(host) && ddr_eye_width(eye) >= 2 * guard + 2 &&
        guarded_tap(eye, active_tap(host), guard);
#else
    return ddr_eye_width(eye) >= ReadAheadMinEye;
#endif
}

#ifndef EMUMMC_DDR200_ADMA_PROBE
static bool fill_sdma(sdmmc_storage_t *storage, u32 sector)
{
    sdmmc_t *host = storage->sdmmc;
    u32 done = 0;
    u64 start = get_tmr_us();
    u64 previous_deadline = host->deadline_us;
    host->deadline_us = previous_deadline ? MIN(previous_deadline, start + 200000) : start + 200000;
    bool ok = sdmmc_storage_transfer(storage, &done, sector, ReadAheadBlocks, read_window.data, false);
    host->deadline_us = previous_deadline;
    if (!ok) {
        sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }
    if (done != ReadAheadBlocks)
        return false;
    sdmmc_ddr200_count_io(storage, done, get_tmr_us() - start);
    return true;
}
#endif

static bool refuse_batch(enum DdrBatchRefusal reason)
{
#ifdef EMUMMC_DDR200_ADMA_PROBE
    ddr_trace_batch_refused(reason);
#else
    (void)reason;
#endif
    return false;
}

static u32 advance_stream(u32 sector, u32 blocks, u64 now)
{
    read_stream_t *oldest = &read_ahead.streams[0];
    for (unsigned i = 0; i < ReadStreams; ++i) {
        read_stream_t *stream = &read_ahead.streams[i];
        if (stream->blocks && stream->next == sector && now - stream->seen < ReadStreamIdleUs) {
            u32 preceding = stream->blocks;
            *stream = (read_stream_t){(u64)sector + blocks, now, MIN((u64)preceding + blocks, ReadAheadRunIn)};
            return preceding;
        }
        if (stream->seen < oldest->seen)
            oldest = stream;
    }
    bool can_seed = blocks >= ReadAheadRequest;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    can_seed |= blocks >= ReadAheadSmallSeed && (!oldest->blocks || now - oldest->seen >= ReadStreamIdleUs);
#endif
    if (can_seed)
        *oldest = (read_stream_t){(u64)sector + blocks, now, MIN(blocks, ReadAheadRunIn)};
    return 0;
}

static bool serve_batch(sdmmc_storage_t *storage, Handle das, u32 sector, u32 blocks, void *out, u32 preceding)
{
    if (storage->ddr.rate != DdrRateFull || storage->sdmmc->divisor != 199680 ||
        !storage->ddr.ready || read_ahead.strikes >= ReadAheadStrikeLimit)
        return refuse_batch(DdrBatchUnavailable);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (!sdmmc_adma2_supported(storage->sdmmc))
        return refuse_batch(DdrBatchUnavailable);
#endif

    bool covered = read_ahead.blocks && read_ahead.mapping.das == das &&
        sector >= read_ahead.first && (u64)sector + blocks <= (u64)read_ahead.first + read_ahead.blocks;
    if (!covered) {
        if (!preceding)
            return refuse_batch(blocks < ReadAheadRequest ? DdrBatchSmall : DdrBatchNonAdjacent);
        unsigned run_in = ReadAheadRunIn;
#ifdef EMUMMC_DDR200_ADMA_PROBE
        if (blocks >= ReadAheadLargeRunIn)
            run_in = ReadAheadLargeRunIn;
#endif
        if (preceding < run_in)
            return refuse_batch(blocks < ReadAheadRequest ? DdrBatchSmall : DdrBatchRunIn);
        if ((u64)sector + ReadAheadBlocks > storage->sec_cnt || blocks > ReadAheadBlocks)
            return refuse_batch(DdrBatchWindow);

#ifdef EMUMMC_DDR200_ADMA_PROBE
        bool stale = ddr_sweep_stale(&storage->ddr, get_tmr_us());
        if (!sdmmc_ddr200_before_io(storage, sector, false) ||
            ddr_sweep_stale(&storage->ddr, get_tmr_us()))
            return refuse_batch(stale ? DdrBatchStale : DdrBatchUnavailable);
#else
        if (!sdmmc_ddr200_before_io(storage, sector, false))
            return refuse_batch(DdrBatchUnavailable);
#endif
        if (!read_ahead_eye_ready(storage))
            return refuse_batch(DdrBatchEye);
        if (read_ahead.retry_at) {
            ddr_eye_t eye = storage->ddr.history[0].eyes[0];
            ddr_eye_t verified = storage->ddr.last_verified_eye;
            if (get_tmr_us() < read_ahead.retry_at || storage->ddr.last_verified_us < read_ahead.retry_at ||
                eye.left != verified.left || eye.right != verified.right)
                return refuse_batch(DdrBatchUnavailable);
        }
        if (!map_window(&read_ahead.mapping, das, &read_window, sizeof(read_window), Perm_W))
            return refuse_batch(DdrBatchWindow);

        if (read_ahead.retry_at) {
            read_ahead.retry_at = 0;
            ddr_trace_read_ahead_retry(read_ahead.strikes, ++read_ahead.reenables);
        }
        read_ahead.blocks = 0;
#ifdef EMUMMC_DDR200_ADMA_PROBE
        bool ok = fill_adma(storage, sector);
#else
        bool ok = fill_sdma(storage, sector);
#endif
        ddr_trace_read_ahead_fill(ok);
        if (!ok) {
            ++read_ahead.strikes;
            read_ahead.retry_at = get_tmr_us() + ReadAheadCooldownUs;
            ddr_trace_read_ahead_retry(read_ahead.strikes, read_ahead.reenables);
#ifdef EMUMMC_DDR200_ADMA_PROBE
            read_ahead.adma_verified = false;
#endif
            memset(read_ahead.streams, 0, sizeof(read_ahead.streams));
            return false;
        }

        read_ahead.first = sector;
        read_ahead.blocks = ReadAheadBlocks;
    }

    memcpy(out, read_window.data + (sector - read_ahead.first) * 512u, blocks * 512u);
    ddr_trace_read_ahead(blocks);
    return true;
}

bool sdmmc_read_ahead(sdmmc_storage_t *storage, Handle das, u32 sector, u32 blocks, void *out)
{
    if (!out || !blocks || !storage->has_sector_access || storage->sdmmc->id != SDMMC_1 ||
        (u64)sector + blocks > storage->sec_cnt)
        return false;
    if (!sdmmc_get_sd_inserted()) {
        sdmmc_read_ahead_reset();
        return false;
    }

    u32 preceding = advance_stream(sector, blocks, get_tmr_us());
    bool served = serve_batch(storage, das, sector, blocks, out, preceding);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (preceding)
        ddr_trace_batch_seq(blocks, served);
#endif
    return served;
}
#endif
