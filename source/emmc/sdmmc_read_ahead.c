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
    ReferenceSlices = ReadAheadBlocks / DdrVerifyBlocks,
#else
    ReadAheadRequest = 256 * 1024 / 512,
    ReadAheadMinEye = 9,
#endif
    DeviceAlignment = 4 * 1024 * 1024,
};

static struct {
    _Alignas(0x1000) u8 data[ReadAheadBytes];
#ifdef EMUMMC_DDR200_ADMA_PROBE
    _Alignas(0x1000) sdmmc_adma2_descriptor_t table[ReadAheadBytes / SdmmcAdmaChunk];
#endif
} read_window;
#ifdef EMUMMC_DDR200_ADMA_PROBE
_Static_assert(offsetof(typeof(read_window), table) == ReadAheadBytes, "ADMA2 table layout");
#endif
static const Handle current_process = 0xFFFF8001u;

/* Both FS storage mutexes protect the cache and its device mapping. */
static struct {
    Handle das;
    u64 dma;
    u64 next_sector;
    u32 sequential_blocks;
    u32 first;
    u32 blocks;
    bool mapping_failed;
    bool fill_failed;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    bool adma_verified;
    u8 verified_tap[2];
    u8 verified_upgrade;
#endif
} read_ahead;

static bool map_read_buffer(Handle das)
{
    if (read_ahead.dma)
        return read_ahead.das == das;
    if (!das || read_ahead.mapping_failed)
        return false;

    /* The aligned mapping call requires equal 4 MiB offsets. The kernel
       checks the device aperture and rejects any occupied range. */
    const u64 bases[] = {0x04000000, 0x10000000, 0x20000000};
    u64 address = (uintptr_t)&read_window;
    u32 permission = Perm_W;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    permission = Perm_Rw;
#endif
    for (unsigned i = 0; i < sizeof(bases) / sizeof(bases[0]); ++i) {
        u64 dma = bases[i] | (address & (DeviceAlignment - 1));
        Result rc = svcMapDeviceAddressSpaceAligned(das, current_process, address, sizeof(read_window),
            dma, permission);
        ddr_trace_read_ahead_map(rc);
        if (!rc) {
            read_ahead.das = das;
            read_ahead.dma = dma;
            return true;
        }
    }
    read_ahead.mapping_failed = true;
    return false;
}

u64 sdmmc_read_ahead_dma(const void *buf, u32 blocks)
{
    uintptr_t offset = (uintptr_t)buf - (uintptr_t)read_window.data;
    if (!read_ahead.dma || !blocks || offset >= ReadAheadBytes ||
        (u64)blocks * 512 > ReadAheadBytes - offset)
        return 0;
    return read_ahead.dma + offset;
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
u64 sdmmc_read_ahead_adma_table(u64 dma, u32 bytes)
{
    u64 offset = dma - read_ahead.dma;
    if (!read_ahead.dma || !bytes || (dma & 7) || offset >= ReadAheadBytes ||
        bytes > ReadAheadBytes - offset)
        return 0;

    unsigned count = 0;
    while (bytes) {
        u16 length = MIN(bytes, SdmmcAdmaChunk);
        read_window.table[count++] = (sdmmc_adma2_descriptor_t){
            .attributes = SdmmcAdmaTransferValid | (bytes == length ? SdmmcAdmaEnd : 0),
            .length = length, .address_low = (u32)dma, .address_high = (u32)(dma >> 32),
        };
        bytes -= length;
        dma += length;
    }
    armDCacheClean(read_window.table, count * sizeof(read_window.table[0]));
    return read_ahead.dma + ReadAheadBytes;
}
#endif

void sdmmc_read_ahead_invalidate(u32 sector, u32 blocks)
{
    if (read_ahead.blocks && blocks &&
        (u64)sector + blocks > read_ahead.first &&
        sector < (u64)read_ahead.first + read_ahead.blocks) {
        read_ahead.blocks = 0;
        read_ahead.sequential_blocks = 0;
    }
}

void sdmmc_read_ahead_reset(void)
{
    read_ahead.blocks = 0;
    read_ahead.sequential_blocks = 0;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    read_ahead.adma_verified = false;
#endif
    if (!read_ahead.dma)
        return;
    Result rc = svcUnmapDeviceAddressSpace(read_ahead.das, current_process, (uintptr_t)&read_window,
        sizeof(read_window), read_ahead.dma);
    read_ahead.dma = 0;
    /* If unmapping failed, the address space may still pin these pages. */
    read_ahead.mapping_failed |= rc != 0;
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

static bool transfer(sdmmc_storage_t *storage, enum SdmmcDmaMode mode, u32 sector,
    u32 blocks, u8 *buffer, u64 *elapsed)
{
    sdmmc_t *host = storage->sdmmc;
    enum SdmmcDmaMode previous = host->read_ahead_dma;
    u32 done = 0;
    u64 start = get_tmr_us();
    host->read_ahead_dma = mode;
    bool ok = sdmmc_storage_transfer(storage, &done, sector, blocks, buffer, false);
    host->read_ahead_dma = previous;
    *elapsed = get_tmr_us() - start;
    if (ok && done == blocks) {
        sdmmc_ddr200_count_io(storage, blocks, *elapsed);
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

static bool read_reference(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye,
    u64 hashes[ReferenceSlices], u8 *poison)
{
    enum { BoundaryBytes = 512 * 1024, SliceBytes = DdrVerifyBlocks * 512 };
    _Static_assert(SliceBytes < BoundaryBytes, "SDMA reference must end before its boundary");
    u8 *slice = read_window.data + (-read_ahead.dma & (BoundaryBytes - 1));
    unsigned i = 0, resumes = 0;
    u64 total_us = 0;
    for (; i < ReferenceSlices; ++i) {
        /* Reuse one aligned 256 KiB slice so even the end of a reference
           command cannot raise a 512 KiB boundary interrupt. */
        u64 elapsed;
        bool ok = transfer(storage, SdmmcDmaSdma512, sector + i * DdrVerifyBlocks,
            DdrVerifyBlocks, slice, &elapsed);
        total_us += elapsed;
        resumes += storage->sdmmc->dma_resumes;
        if (!ok || storage->sdmmc->dma_resumes)
            break;
        hashes[i] = ddr_data_checksum(slice, SliceBytes);
        if (!i)
            *poison = (u8)~slice[0];
    }
    bool ok = i == ReferenceSlices;
    ddr_trace_dma_sample(DdrDmaReference, storage->sdmmc, eye, total_us,
        ReadAheadBlocks, resumes, ok ? DdrDmaOk : DdrDmaFailed);
    return ok;
}

static enum DdrDmaResult compare_read(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye,
    const u64 hashes[ReferenceSlices], u8 poison, enum DdrDmaSample sample)
{
    /* A transfer that leaves the previous bytes in memory must fail validation. */
    memset(read_window.data, poison, ReadAheadBytes);
    u64 elapsed;
    enum DdrDmaResult result = DdrDmaFailed;
    if (transfer(storage, SdmmcDmaAdma2, sector, ReadAheadBlocks, read_window.data, &elapsed)) {
        enum { SliceBytes = DdrVerifyBlocks * 512 };
        result = DdrDmaOk;
        for (unsigned i = 0; i < ReferenceSlices; ++i) {
            if (ddr_data_checksum(read_window.data + i * SliceBytes, SliceBytes) != hashes[i]) {
                result = DdrDmaMismatch;
                break;
            }
        }
    }
    ddr_trace_dma_sample(sample, storage->sdmmc, eye, elapsed,
        ReadAheadBlocks, storage->sdmmc->dma_resumes, result);
    return result;
}

static bool qualify_adma(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye, unsigned tap, unsigned alternate)
{
    sdmmc_t *host = storage->sdmmc;
    u64 hashes[ReferenceSlices];
    u8 poison = 0;
    ddr_trace_dma_begin();
    if (!read_reference(storage, sector, eye, hashes, &poison)) {
        if (data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }
    enum DdrDmaResult result = compare_read(storage, sector, eye, hashes, poison, DdrDmaBaseline);
    if (result != DdrDmaOk) {
        if (result == DdrDmaFailed && data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }

    sdmmc_set_ddr200_tap(host, alternate);
    if (active_tap(host) != alternate) {
        sdmmc_set_ddr200_tap(host, tap);
        return false;
    }
    result = compare_read(storage, sector, eye, hashes, poison, DdrDmaAlternate);
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
        unsigned alternate = guarded_tap(eye, tap + 1, guard) ? tap + 1 : tap - 1;
        return qualify_adma(storage, sector, eye, tap, alternate);
    }

    u64 elapsed;
    if (transfer(storage, SdmmcDmaAdma2, sector, ReadAheadBlocks, read_window.data, &elapsed))
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

bool sdmmc_read_ahead(sdmmc_storage_t *storage, Handle das, u32 sector, u32 blocks, void *out)
{
    sdmmc_t *host = storage->sdmmc;
    u64 end = (u64)sector + blocks;
    if (!out || !storage->has_sector_access || host->id != SDMMC_1 ||
        storage->ddr.rate != DdrRateFull || host->divisor != 199680 || !storage->ddr.ready ||
        blocks >= ReadAheadBlocks || end > storage->sec_cnt) {
        read_ahead.sequential_blocks = 0;
        return false;
    }
    if (!sdmmc_get_sd_inserted()) {
        sdmmc_read_ahead_reset();
        return false;
    }
    /* Interleaved metadata reads do not break a run of large adjacent reads. */
    if (blocks < ReadAheadRequest)
        return false;

    u32 preceding = sector == read_ahead.next_sector ? read_ahead.sequential_blocks : 0;
    read_ahead.sequential_blocks = MIN(preceding + blocks, ReadAheadRunIn);
    read_ahead.next_sector = end;
    bool covered = read_ahead.blocks && read_ahead.das == das &&
        sector >= read_ahead.first && end <= (u64)read_ahead.first + read_ahead.blocks;
    if (!covered) {
        if (read_ahead.fill_failed || preceding < ReadAheadRunIn ||
            (u64)sector + ReadAheadBlocks > storage->sec_cnt ||
            !read_ahead_eye_ready(storage) ||
            !sdmmc_ddr200_before_io(storage, sector, false) ||
            !read_ahead_eye_ready(storage) || !map_read_buffer(das))
            return false;
        read_ahead.blocks = 0;
#ifdef EMUMMC_DDR200_ADMA_PROBE
        bool ok = fill_adma(storage, sector);
#else
        bool ok = fill_sdma(storage, sector);
#endif
        ddr_trace_read_ahead_fill(ok);
        if (!ok) {
            read_ahead.fill_failed = true;
            read_ahead.sequential_blocks = 0;
            return false;
        }

        read_ahead.first = sector;
        read_ahead.blocks = ReadAheadBlocks;
    }

    memcpy(out, read_window.data + (sector - read_ahead.first) * 512u, blocks * 512u);
    ddr_trace_read_ahead(blocks);
    return true;
}
#endif
