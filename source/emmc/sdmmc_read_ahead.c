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
    ReadAheadRequest = 256 * 1024 / 512,
    ReadAheadRunIn = 512 * 1024 / 512,
    ReadAheadMinEye = 9,
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
    bool compared;
    bool compared_at_eight;
    ddr_eye_t verified_eight;
    u8 verified_tap;
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
    read_ahead.verified_eight = (ddr_eye_t){0};
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

static bool read_reference(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye, bool compare)
{
    enum { BoundaryBlocks = 512 * 1024 / 512 };
    u32 done = 0, resumes = 0;
    u64 total_us = 0;
    while (done < ReadAheadBlocks) {
        /* Every reference command ends at or before its next SDMA boundary. */
        u32 before_boundary = BoundaryBlocks - (read_ahead.dma / 512 + done) % BoundaryBlocks;
        u32 count = MIN(ReadAheadBlocks - done, before_boundary);
        u64 elapsed;
        bool ok = transfer(storage, SdmmcDmaSdma512, sector + done, count,
            read_window.data + done * 512u, &elapsed);
        total_us += elapsed;
        resumes += storage->sdmmc->dma_resumes;
        if (!ok)
            break;
        done += count;
    }
    bool ok = done == ReadAheadBlocks;
    if (compare)
        ddr_trace_dma_sample(DdrDmaReference, storage->sdmmc, eye, total_us,
            ReadAheadBlocks, resumes, ok ? DdrDmaOk : DdrDmaFailed);
    return ok;
}

static enum DdrDmaResult compare_read(sdmmc_storage_t *storage, enum SdmmcDmaMode mode,
    u32 sector, ddr_eye_t eye, u64 reference, u8 poison, bool log, enum DdrDmaSample sample)
{
    /* A transfer that leaves the previous bytes in memory must fail validation. */
    memset(read_window.data, poison, ReadAheadBytes);
    u64 elapsed;
    enum DdrDmaResult result = DdrDmaFailed;
    if (transfer(storage, mode, sector, ReadAheadBlocks, read_window.data, &elapsed))
        result = ddr_data_checksum(read_window.data, ReadAheadBytes) == reference ? DdrDmaOk : DdrDmaMismatch;
    if (log)
        ddr_trace_dma_sample(sample, storage->sdmmc, eye, elapsed,
            ReadAheadBlocks, storage->sdmmc->dma_resumes, result);
    return result;
}

static bool qualify_adma(sdmmc_storage_t *storage, u32 sector, ddr_eye_t eye, unsigned tap, bool eight)
{
    sdmmc_t *host = storage->sdmmc;
    bool compare = !read_ahead.compared || (eight && !read_ahead.compared_at_eight);
    if (compare) {
        ddr_trace_dma_begin();
        read_ahead.compared = true;
        read_ahead.compared_at_eight |= eight;
    }

    if (!read_reference(storage, sector, eye, compare)) {
        if (data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }
    u64 reference = ddr_data_checksum(read_window.data, ReadAheadBytes);
    u8 poison = (u8)~read_window.data[0];
    enum DdrDmaResult result = compare_read(storage, SdmmcDmaAdma2, sector, eye,
        reference, poison, compare, DdrDmaBaseline);
    if (result != DdrDmaOk) {
        if (result == DdrDmaFailed && data_command_failed(host))
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }

    bool failed_tap = false;
    if (compare) {
        result = compare_read(storage, SdmmcDmaSdma512, sector, eye,
            reference, poison, true, DdrDmaSdma512);
        if (result == DdrDmaOk)
            result = compare_read(storage, SdmmcDmaSdma256, sector, eye,
                reference, poison, true, DdrDmaSdma256);
        if (result == DdrDmaMismatch || !storage->ddr.ready)
            return false;
        failed_tap = result == DdrDmaFailed;
    }

    unsigned alternate = tap < eye.right - DdrTapMargin ? tap + 1 : tap - 1;
    sdmmc_set_ddr200_tap(host, alternate);
    if (active_tap(host) != alternate) {
        sdmmc_set_ddr200_tap(host, tap);
        if (failed_tap)
            sdmmc_ddr200_recover(storage, sector, false);
        return false;
    }
    result = compare_read(storage, SdmmcDmaAdma2, sector, eye,
        reference, poison, compare, DdrDmaAlternate);
    if (result != DdrDmaOk) {
        if (failed_tap || (result == DdrDmaFailed && data_command_failed(host)))
            sdmmc_ddr200_recover(storage, sector, failed_tap);
        else
            sdmmc_set_ddr200_tap(host, tap);
        return false;
    }

    /* A control failure rules out its tap for the current link setup. */
    if (!failed_tap)
        sdmmc_set_ddr200_tap(host, tap);
    unsigned selected = failed_tap ? alternate : tap;
    if (active_tap(host) != selected)
        return false;
    read_ahead.adma_verified = true;
    if (eight) {
        read_ahead.verified_eight = eye;
        read_ahead.verified_tap = selected;
    }
    return true;
}

static bool fill_adma(sdmmc_storage_t *storage, u32 sector)
{
    ddr_eye_t eye = storage->ddr.history[0].eyes[0];
    unsigned tap = active_tap(storage->sdmmc);
    bool eight = ddr_eye_width(eye) == 2 * DdrTapMargin + 2;
    if (!eight)
        read_ahead.verified_eight = (ddr_eye_t){0};
    if (!read_ahead.adma_verified || (eight && (read_ahead.verified_eight.left != eye.left ||
        read_ahead.verified_eight.right != eye.right || read_ahead.verified_tap != tap)))
        return qualify_adma(storage, sector, eye, tap, eight);

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
    if (sdmmc_adma2_supported(host)) {
        unsigned tap = active_tap(host);
        return ddr_eye_width(eye) >= 2 * DdrTapMargin + 2 &&
            tap >= eye.left + DdrTapMargin && tap <= eye.right - DdrTapMargin;
    }
#endif
    return ddr_eye_width(eye) >= ReadAheadMinEye;
}

static bool fill_sdma(sdmmc_storage_t *storage, u32 sector)
{
    sdmmc_t *host = storage->sdmmc;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    u64 elapsed;
    bool ok = transfer(storage, SdmmcDmaSdma512, sector, ReadAheadBlocks, read_window.data, &elapsed);
    if (!ok && data_command_failed(host))
        sdmmc_ddr200_recover(storage, sector, false);
    return ok;
#else
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
#endif
}

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
        bool ok = sdmmc_adma2_supported(host) ? fill_adma(storage, sector) : fill_sdma(storage, sector);
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
