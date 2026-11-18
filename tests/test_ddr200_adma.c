/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/emmc/sdmmc.h"
#define sdmmc_get_sd_inserted driver_get_sd_inserted
#include "../source/emmc/sdmmc_driver.c"
#undef sdmmc_get_sd_inserted
#include "../source/emmc/sdmmc_read_ahead.c"

static t210_sdmmc_t regs;
static sdmmc_t host = {.regs = &regs, .id = SDMMC_1, .divisor = 199680};
static sdmmc_storage_t card = {.sdmmc = &host, .has_sector_access = true, .sec_cnt = 32768};
static mmc_obj_t device;
static sdmmc_accessor_t accessor = {.parent = &device};
sdmmc_accessor_t *_current_accessor = &accessor;
bool sdmmc_memcpy_buf;
static u64 time_us;
static u8 output[512 * 1024];

static struct {
    enum SdmmcDmaMode mode;
    unsigned tap;
    unsigned blocks;
} commands[40];
static enum DdrDmaResult samples[DdrDmaSamples];
static unsigned sample_taps[DdrDmaSamples];
static unsigned command_count, compares, maps, unmaps, recoveries, filled[2];
static enum SdmmcDmaMode fault_mode;
static unsigned fault_tap;
static u16 fault_irq;
static bool stale_data, stop_failure;

u64 get_tmr_us(void) { return ++time_us; }
bool sdmmc_get_sd_inserted(void) { return true; }
void ddr_trace_read_ahead(unsigned blocks) { assert(blocks == sizeof(output) / 512); }
void ddr_trace_read_ahead_map(Result result) { assert(!result); }
void ddr_trace_read_ahead_fill(bool succeeded) { ++filled[succeeded]; }
void ddr_trace_dma_begin(void)
{
    ++compares;
    memset(samples, 0, sizeof(samples));
}
void ddr_trace_dma_sample(enum DdrDmaSample sample, const sdmmc_t *target, ddr_eye_t eye,
    u64 elapsed, u32 blocks, u32 resumes, enum DdrDmaResult result)
{
    assert(target == &host && ddr_eye_width(eye) >= 8 && elapsed && blocks == ReadAheadBlocks);
    assert(sample != DdrDmaReference || resumes == 0);
    samples[sample] = result;
    sample_taps[sample] = active_tap(target);
}

Result svcMapDeviceAddressSpaceAligned(Handle handle, Handle process, u64 address,
    u64 size, u64 dma, u32 permission)
{
    assert(handle && process == current_process && address == (uintptr_t)&read_window);
    assert(size == ReadAheadBytes + 4096 && permission == Perm_Rw);
    assert(((address ^ dma) & (DeviceAlignment - 1)) == 0);
    ++maps;
    return 0;
}

Result svcUnmapDeviceAddressSpace(Handle handle, Handle process, u64 address, u64 size, u64 dma)
{
    assert(handle == read_ahead.das && process == current_process && address == (uintptr_t)&read_window);
    assert(size == sizeof(read_window) && dma == read_ahead.dma);
    ++unmaps;
    return 0;
}

intptr_t sdmmc_calculate_dma_addr(sdmmc_accessor_t *source, void *buf, unsigned int blocks)
{
    assert(source == &accessor && buf == output && blocks <= sizeof(output) / 512);
    return 0x12340000;
}

void armDCacheClean(void *buf, size_t size)
{
    assert(buf == read_window.table && size && size <= sizeof(read_window.table));
}

int sdmmc_storage_transfer(sdmmc_storage_t *storage, u32 *done, u32 sector, u32 blocks, void *buf, u32 write)
{
    assert(storage == &card && !write && command_count < sizeof(commands) / sizeof(commands[0]));
    u64 dma = sdmmc_read_ahead_dma(buf, blocks);
    assert(dma && !host.deadline_us);
    unsigned tap = active_tap(&host);
    enum SdmmcDmaMode mode = host.read_ahead_dma;
    commands[command_count++] = (typeof(commands[0])){mode, tap, blocks};
    sdmmc_req_t request = {.buf = buf, .blksize = 512, .num_sectors = blocks, .is_multi_block = true,
        .auto_command = SdmmcAutoStop};
    host.dma_progress = (sdmmc_dma_progress_t){0};
    host.command_failure = 0;
    host.command_irq = 0;
    host.dma_resumes = 0;
    assert(_sdmmc_config_dma(&host, done, &request));
    assert(*done == blocks && !sdmmc_memcpy_buf && regs.blkcnt == blocks);
    assert((regs.trnmod & (SDHCI_TRNS_DMA | SDHCI_TRNS_AUTO_CMD12)) ==
        (SDHCI_TRNS_DMA | SDHCI_TRNS_AUTO_CMD12));
    assert(host.dma_progress.blocks == blocks && host.dma_progress.mode == mode);
    if (mode == SdmmcDmaAdma2) {
        assert((regs.hostctl & SDHCI_CTRL_DMA_MASK) == SDHCI_CTRL_ADMA32);
        assert(!host.dma_boundaries_left && regs.admaaddr == read_ahead.dma + ReadAheadBytes);
        assert(!regs.admaaddr_hi && blocks == ReadAheadBlocks);
        for (unsigned i = 0; i < sizeof(read_window.table) / sizeof(read_window.table[0]); ++i) {
            const sdmmc_adma2_descriptor_t *entry = &read_window.table[i];
            unsigned end = i + 1 == sizeof(read_window.table) / sizeof(read_window.table[0]) ? SdmmcAdmaEnd : 0;
            assert(entry->attributes == (SdmmcAdmaTransferValid | end) && entry->length == SdmmcAdmaChunk);
            assert(entry->address_low == dma + i * SdmmcAdmaChunk && !entry->address_high && !entry->reserved);
        }
        /* Emulate the command wrapper's restore before storage cleanup. */
        regs.hostctl &= ~SDHCI_CTRL_DMA_MASK;
    } else {
        assert((regs.hostctl & SDHCI_CTRL_DMA_MASK) == SDHCI_CTRL_SDMA);
        u32 boundary = mode == SdmmcDmaSdma256 ? 256 * 1024 : 512 * 1024;
        assert(regs.admaaddr == dma && !regs.admaaddr_hi && host.dma_boundary_bytes == boundary);
        assert(regs.blksize == (512 | ((boundary == 512 * 1024 ? 7 : 6) << 12)));
        assert(host.dma_addr_next == ((dma + boundary) & ~((u64)boundary - 1)));
        host.dma_resumes = ((dma & (boundary - 1)) + 512 * blocks - 1) / boundary;
        if (blocks < ReadAheadBlocks)
            assert(!host.dma_resumes);
    }
    if (mode == fault_mode && tap == fault_tap && blocks == ReadAheadBlocks) {
        if (stale_data)
            return 1;
        host.command_failure = DdrCommandTransfer;
        host.command_irq = (u32)fault_irq << 16 | SDHCI_INT_ERROR;
        card.ddr.ready = !stop_failure;
        return 0;
    }
    memset(buf, 0, blocks * 512);
    for (u32 offset = 0; offset < blocks; ++offset)
        ((u8 *)buf)[offset * 512] = ((sector + offset) / 128) & 0xff;
    return 1;
}

bool sdmmc_ddr200_before_io(sdmmc_storage_t *storage, u32 sector, bool writing)
{
    assert(storage == &card && sector < card.sec_cnt && !writing);
    return card.ddr.ready;
}

void sdmmc_ddr200_count_io(sdmmc_storage_t *storage, u32 blocks, u64 elapsed)
{
    assert(storage == &card && blocks && blocks <= ReadAheadBlocks && elapsed);
}

bool sdmmc_ddr200_recover(sdmmc_storage_t *storage, u32 sector, bool degrade)
{
    assert(storage == &card && sector < card.sec_cnt && !degrade);
    assert(host.read_ahead_dma == SdmmcDmaDefault && fault_irq != SDHCI_ERR_INT_ADMA_ERROR);
    assert(active_tap(&host) == fault_tap);
    ++recoveries;
    return true;
}

static void reset(ddr_eye_t eye, unsigned tap)
{
    sdmmc_read_ahead_reset();
    read_ahead = (typeof(read_ahead)){0};
    card.ddr = (ddr_policy_t){.rate = DdrRateFull, .ready = true};
    ddr_remember_eye(&card.ddr.history[0], eye);
    regs.capareg = SDHCI_CAN_ADMA2;
    regs.hcver = 3;
    regs.hostctl2 = SDHCI_HOST_VERSION_4_EN | SDHCI_ADDRESSING_64BIT_EN | UHS_DDR50_BUS_SPEED;
    regs.hostctl = SDHCI_CTRL_4BITBUS;
    sdmmc_set_ddr200_tap(&host, tap);
    command_count = compares = maps = unmaps = recoveries = filled[0] = filled[1] = 0;
    fault_mode = SdmmcDmaDefault;
    fault_irq = SDHCI_ERR_INT_DATA_CRC;
    stale_data = stop_failure = false;
}

static bool read_next(Handle handle, u32 sector)
{
    return sdmmc_read_ahead(&card, handle, sector, sizeof(output) / 512, output);
}

int main(void)
{
    const ddr_eye_t eight = {29, 36};
    reset(eight, 32);
    assert(!read_next(1, 0));
    assert(read_next(1, 1024) && compares == 1 && filled[true] == 1 && !recoveries);
    assert(output[0] == 8 && output[128 * 512] == 9);
    for (unsigned i = DdrDmaReference; i <= DdrDmaAlternate; ++i)
        assert(samples[i] == DdrDmaOk && sample_taps[i] == (i == DdrDmaAlternate ? 33 : 32));
    assert(active_tap(&host) == 32 && host.read_ahead_dma == SdmmcDmaDefault);
    assert(!sdmmc_read_ahead_dma(read_window.data + ReadAheadBytes, 1));
    assert(!sdmmc_read_ahead_dma(read_window.data + ReadAheadBytes - 512, 2));
    assert(!sdmmc_read_ahead_adma_table(read_ahead.dma + ReadAheadBytes, 512));
    assert(!sdmmc_read_ahead_adma_table(read_ahead.dma, ReadAheadBytes + 512));
    unsigned qualified = command_count;
    assert(read_next(1, 2048) && read_next(1, 3072) && read_next(1, 4096) && command_count == qualified);
    assert(read_next(1, 5120) && command_count == qualified + 1 && commands[qualified].mode == SdmmcDmaAdma2);
    sdmmc_read_ahead_reset();
    assert(unmaps == 1 && !read_ahead.adma_verified && !read_ahead.dma);
    assert(!read_next(2, 0) && read_next(2, 1024));
    assert(compares == 1 && maps == 2 && read_ahead.adma_verified);

    reset((ddr_eye_t){30, 36}, 33);
    assert(!read_next(1, 0) && !read_next(1, 1024) && !maps && !command_count && card.ddr.ready);
    reset(eight, 31);
    assert(!read_next(1, 0) && !read_next(1, 1024) && !maps && !command_count);
    reset(eight, 32);
    regs.capareg = 0;
    assert(!read_next(1, 0) && !read_next(1, 1024) && !maps && !command_count);

    reset(eight, 32);
    fault_mode = SdmmcDmaSdma512;
    fault_tap = 32;
    assert(!read_next(1, 0) && read_next(1, 1024));
    assert(samples[DdrDmaBaseline] == DdrDmaOk && samples[DdrDmaSdma512] == DdrDmaFailed);
    assert(!samples[DdrDmaSdma256] && samples[DdrDmaAlternate] == DdrDmaOk && !recoveries);
    assert(active_tap(&host) == 33 && host.read_ahead_dma == SdmmcDmaDefault);

    reset(eight, 32);
    fault_mode = SdmmcDmaSdma512;
    fault_tap = 32;
    stop_failure = true;
    assert(!read_next(1, 0) && !read_next(1, 1024) && !card.ddr.ready && !read_ahead.blocks);
    assert(!samples[DdrDmaAlternate] && !recoveries);

    reset(eight, 32);
    fault_mode = SdmmcDmaAdma2;
    fault_tap = 33;
    fault_irq = SDHCI_ERR_INT_ADMA_ERROR;
    assert(!read_next(1, 0) && !read_next(1, 1024));
    assert(samples[DdrDmaAlternate] == DdrDmaFailed && card.ddr.ready && active_tap(&host) == 32);
    assert(!recoveries && read_ahead.fill_failed && !read_ahead.blocks);

    reset(eight, 32);
    fault_mode = SdmmcDmaAdma2;
    fault_tap = 32;
    assert(!read_next(1, 0) && !read_next(1, 1024) && recoveries == 1);
    assert(!read_ahead.blocks && read_ahead.fill_failed);

    reset(eight, 32);
    fault_mode = SdmmcDmaAdma2;
    fault_tap = 33;
    stale_data = true;
    memset(output, 0x7f, sizeof(output));
    assert(!read_next(1, 0) && !read_next(1, 1024));
    assert(samples[DdrDmaAlternate] == DdrDmaMismatch && active_tap(&host) == 32);
    assert(output[0] == 0x7f && !read_ahead.blocks && read_ahead.fill_failed);

    reset((ddr_eye_t){29, 43}, 32);
    assert(!read_next(1, 0) && read_next(1, 1024));
    ddr_remember_eye(&card.ddr.history[0], eight);
    assert(read_next(1, 2048) && read_next(1, 3072) && read_next(1, 4096) && read_next(1, 5120));
    assert(compares == 2 && samples[DdrDmaAlternate] == DdrDmaOk);
    puts("DDR200 ADMA2 qualifies both guarded taps, compares SDMA boundaries and withholds unverified cache data");
}
