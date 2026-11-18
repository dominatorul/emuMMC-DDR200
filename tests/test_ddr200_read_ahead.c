/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/emmc/sdmmc_read_ahead.c"

static u8 output[512 * 1024];
static sdmmc_t host = {.id = SDMMC_1, .divisor = 199680};
static sdmmc_storage_t card = {
    .sdmmc = &host, .has_sector_access = 1, .sec_cnt = 16384,
    .ddr = {.rate = DdrRateFull, .ready = true},
};
static bool map_ok = true, transfer_ok = true, inserted = true;
static unsigned maps, unmaps, transfers, recoveries, served, mapped_sectors;
static u64 time_us = 1234;

Result svcMapDeviceAddressSpaceAligned(Handle das, Handle process, u64 address, u64 size,
    u64 device_address, u32 permission)
{
    ++maps;
    assert(das && process == current_process && address == (uintptr_t)read_buffer);
    assert(size == ReadAheadBytes && (address & 4095) == 0);
    assert(((address ^ device_address) & (DeviceAlignment - 1)) == 0 && permission == Perm_W);
    return map_ok ? 0 : 0xd401;
}

Result svcUnmapDeviceAddressSpace(Handle das, Handle process, u64 address, u64 size,
    u64 device_address)
{
    ++unmaps;
    assert(das == read_ahead.das && process == current_process &&
        address == (uintptr_t)read_buffer && size == ReadAheadBytes && device_address == read_ahead.dma);
    return 0;
}

void ddr_trace_read_ahead_map(Result result) { assert((result == 0) == map_ok); }
void ddr_trace_read_ahead(u32 blocks) { served += blocks; }
bool sdmmc_get_sd_inserted(void) { return inserted; }
u64 get_tmr_us(void) { return ++time_us; }

bool sdmmc_ddr200_before_io(sdmmc_storage_t *storage, u32 sector, bool writing)
{
    assert(storage == &card && sector < card.sec_cnt && !writing);
    return true;
}

int sdmmc_storage_transfer(sdmmc_storage_t *storage, u32 *done, u32 sector,
    u32 blocks, void *buf, u32 write)
{
    assert(storage == &card && buf == read_buffer && blocks == ReadAheadBlocks && !write);
    assert(sdmmc_read_ahead_dma(buf, blocks) != 0 && host.deadline_us > time_us);
    ++transfers;
    if (!transfer_ok)
        return 0;
    for (unsigned i = 0; i < blocks; ++i)
        memset((u8 *)buf + i * 512, ((sector + i) / 512) & 0xff, 512);
    *done = blocks;
    return 1;
}

void sdmmc_ddr200_count_io(sdmmc_storage_t *storage, u32 blocks, u64 elapsed)
{
    assert(storage == &card && blocks == ReadAheadBlocks && elapsed != 0);
    mapped_sectors += blocks;
}

bool sdmmc_ddr200_recover(sdmmc_storage_t *storage, u32 sector, bool degrade)
{
    assert(storage == &card && sector < card.sec_cnt && !degrade);
    ++recoveries;
    return true;
}

static bool read_sectors(Handle handle, u32 sector)
{
    return sdmmc_read_ahead(&card, handle, sector, sizeof(output) / 512, output);
}

int main(void)
{
    assert(!read_sectors(1, 0) && !read_sectors(1, 1024));
    assert(read_sectors(1, 2048) && transfers == 1 && maps == 1);
    assert(output[0] == 4 && output[sizeof(output) - 1] == 5);
    sdmmc_read_ahead_invalidate(10240, 512);
    assert(read_sectors(1, 3072) && output[0] == 6);
    assert(transfers == 1 && served == 2048 && mapped_sectors == ReadAheadBlocks);

    sdmmc_read_ahead_invalidate(4097, 1);
    assert(!read_sectors(1, 4096));
    assert(!read_sectors(1, 5120));
    transfer_ok = false;
    assert(!read_sectors(1, 6144) && recoveries == 1 && transfers == 2 && !host.deadline_us);
    assert(!read_sectors(1, 7168));

    sdmmc_read_ahead_reset();
    assert(unmaps == 1 && !sdmmc_read_ahead_dma(read_buffer, ReadAheadBlocks));
    transfer_ok = true;
    assert(!read_sectors(2, 0) && !read_sectors(2, 1024));
    assert(read_sectors(2, 2048) && maps == 2 && transfers == 3);
    inserted = false;
    assert(!read_sectors(2, 3072));
    inserted = true;

    sdmmc_read_ahead_reset();
    assert(!read_sectors(3, 13312) && !read_sectors(3, 14336));
    assert(!read_sectors(3, 15360) && maps == 2);
    map_ok = false;
    assert(!read_sectors(3, 0) && !read_sectors(3, 1024));
    assert(!read_sectors(3, 2048) && maps == 5 && transfers == 3);
    assert(!read_sectors(3, 3072) && maps == 5);
    puts("DDR200 read-ahead preserves sector data, invalidates writes and wake, and falls back on mapping or transfer errors");
}
