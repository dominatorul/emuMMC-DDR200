/* SPDX-License-Identifier: GPL-2.0-only */
#ifdef EMUMMC_DDR200_READ_AHEAD
#include <string.h>
#include "sdmmc.h"
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

static _Alignas(0x1000) u8 read_buffer[ReadAheadBytes];
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
    u64 address = (uintptr_t)read_buffer;
    for (unsigned i = 0; i < sizeof(bases) / sizeof(bases[0]); ++i) {
        u64 dma = bases[i] | (address & (DeviceAlignment - 1));
        Result rc = svcMapDeviceAddressSpaceAligned(das, current_process, address, sizeof(read_buffer),
            dma, Perm_W);
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
    return buf == read_buffer && blocks <= ReadAheadBlocks ? read_ahead.dma : 0;
}

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
    if (!read_ahead.dma)
        return;
    Result rc = svcUnmapDeviceAddressSpace(read_ahead.das, current_process, (uintptr_t)read_buffer,
        sizeof(read_buffer), read_ahead.dma);
    read_ahead.dma = 0;
    /* If unmapping failed, the address space may still pin these pages. */
    read_ahead.mapping_failed |= rc != 0;
}

static bool read_ahead_eye_ready(const ddr_policy_t *policy)
{
    const ddr_history_t *history = &policy->history[0];
    return policy->rate == DdrRateFull && history->count &&
        ddr_eye_width(history->eyes[0]) >= ReadAheadMinEye;
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
            !read_ahead_eye_ready(&storage->ddr) ||
            !sdmmc_ddr200_before_io(storage, sector, false) ||
            !read_ahead_eye_ready(&storage->ddr) || !map_read_buffer(das))
            return false;
        read_ahead.blocks = 0;
        u32 done = 0;
        u64 start = get_tmr_us();
        u64 previous_deadline = host->deadline_us;
        host->deadline_us = previous_deadline ? MIN(previous_deadline, start + 200000) : start + 200000;
        bool ok = sdmmc_storage_transfer(storage, &done, sector, ReadAheadBlocks, read_buffer, false);
        host->deadline_us = previous_deadline;
        ddr_trace_read_ahead_fill(ok && done == ReadAheadBlocks);
        if (!ok || done != ReadAheadBlocks) {
            read_ahead.fill_failed = true;
            read_ahead.sequential_blocks = 0;
            if (!ok)
                sdmmc_ddr200_recover(storage, sector, false);
            return false;
        }

        sdmmc_ddr200_count_io(storage, done, get_tmr_us() - start);
        read_ahead.first = sector;
        read_ahead.blocks = done;
    }

    memcpy(out, read_buffer + (sector - read_ahead.first) * 512u, blocks * 512u);
    ddr_trace_read_ahead(blocks);
    return true;
}
#endif
