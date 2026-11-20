/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/emmc/sdmmc.h"
#define sdmmc_get_sd_inserted driver_get_sd_inserted
#include "../source/emmc/sdmmc_driver.c"
#undef sdmmc_get_sd_inserted
#include "../source/emmc/sdmmc_read_ahead.c"

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result)
{ (void)step; (void)detail; (void)result; }
void ddr_trace_wake_host(const sdmmc_t *host) { (void)host; }
#endif

static t210_sdmmc_t regs;
static sdmmc_t host = {.regs = &regs, .id = SDMMC_1, .divisor = 199680};
static sdmmc_storage_t card = {.sdmmc = &host, .has_sector_access = true, .sec_cnt = 32768};
static mmc_obj_t device;
static sdmmc_accessor_t accessor = {.parent = &device};
sdmmc_accessor_t *_current_accessor = &accessor;
bool sdmmc_memcpy_buf;
static Handle das = 1;
volatile Handle *sdmmc_das_handle = &das;
static u64 time_us = 10;
static _Alignas(8) u8 direct[512 * 1024], bounce[512 * 1024];
static u8 output[512 * 1024];

static enum DdrDmaResult samples[DdrDmaSamples];
static unsigned sample_taps[DdrDmaSamples], refusals[DdrBatchRefusals];
static unsigned commands, compares, maps, unmaps, recoveries, resets, restarts;
static u32 sequential, served;
static int fault_tap;
static u16 fault_irq;
static bool stale_data, mapping_ok = true;

u64 get_tmr_us(void) { return ++time_us; }
u64 get_tmr_ms(void)
{
    if (regs.swrst) {
        assert(regs.swrst == (SDHCI_RESET_CMD | SDHCI_RESET_DATA));
        ++resets;
        regs.swrst = 0;
    }
    return ++time_us;
}

bool sdmmc_get_sd_inserted(void) { return true; }
void ddr_trace_read_ahead(unsigned blocks) { assert(blocks && blocks <= sizeof(output) / 512); }
void ddr_trace_read_ahead_map(Result result) { assert((result == 0) == mapping_ok); }
void ddr_trace_read_ahead_fill(bool succeeded) { (void)succeeded; }
void ddr_trace_read_ahead_retry(unsigned strikes, unsigned reenables) { (void)strikes; (void)reenables; }
void ddr_trace_batch_refused(enum DdrBatchRefusal reason) { ++refusals[reason]; }
void ddr_trace_sdma_restart(bool handoff) { (void)handoff; ++restarts; }
void ddr_trace_descriptor_mapping(bool unmap, u32 handle, Result result)
{
    assert((unmap ? handle == descriptor_mapping.das : handle == das) &&
        (!result || (!unmap && !mapping_ok)));
}
void ddr_trace_batch_seq(u32 blocks, bool hit)
{
    sequential += blocks;
    if (hit)
        served += blocks;
}
void ddr_trace_dma_begin(void)
{
    ++compares;
    memset(samples, 0, sizeof(samples));
}
void ddr_trace_dma_sample(enum DdrDmaSample sample, const sdmmc_t *target, ddr_eye_t eye,
    u64 elapsed, u32 blocks, u32 resumes, enum DdrDmaResult result)
{
    assert(target == &host && ddr_eye_width(eye) >= 6 && elapsed && blocks == ReadAheadBlocks);
    assert(!resumes);
    samples[sample] = result;
    sample_taps[sample] = active_tap(target);
}

Result svcMapDeviceAddressSpaceAligned(Handle handle, Handle process, u64 address,
    u64 size, u64 dma, u32 permission)
{
    assert(handle && process == current_process);
    assert(((address ^ dma) & (DeviceAlignment - 1)) == 0);
    if (address == (uintptr_t)&descriptor_table)
        assert(size == 0x1000 && permission == Perm_R);
    else
        assert(address == (uintptr_t)&read_window && size == ReadAheadBytes && permission == Perm_W);
    ++maps;
    return mapping_ok ? 0 : 1;
}

Result svcUnmapDeviceAddressSpace(Handle handle, Handle process, u64 address, u64 size, u64 dma)
{
    assert(handle && process == current_process);
    if (address == (uintptr_t)&descriptor_table)
        assert(size == 0x1000 && dma == descriptor_mapping.dma);
    else
        assert(address == (uintptr_t)&read_window && size == ReadAheadBytes && dma == read_ahead.mapping.dma);
    ++unmaps;
    return 0;
}

intptr_t sdmmc_calculate_dma_addr(sdmmc_accessor_t *source, void *buf, unsigned int blocks)
{
    assert(source == &accessor);
    uintptr_t offset = (uintptr_t)buf - (uintptr_t)direct;
    if (offset >= sizeof(direct) || blocks * 512 > sizeof(direct) - offset)
        return 0;
    sdmmc_memcpy_buf = false;
    return 0x1237ffe0 + offset;
}

void armDCacheClean(void *buf, size_t size)
{
    assert(buf == descriptor_table.entries && size && size <= sizeof(descriptor_table.entries));
}

static void assert_chain(u64 dma, unsigned bytes)
{
    unsigned count = (bytes + SdmmcAdmaChunk - 1) / SdmmcAdmaChunk;
    for (unsigned i = 0; i < count; ++i) {
        const sdmmc_adma2_descriptor_t *entry = &descriptor_table.entries[i];
        unsigned end = i + 1 == count ? SdmmcAdmaEnd : 0;
        assert(entry->attributes == (SdmmcAdmaTransferValid | end));
        assert(entry->length == MIN(bytes, SdmmcAdmaChunk) && entry->length);
        assert(entry->address_low == (u32)dma && entry->address_high == dma >> 32 && !entry->reserved);
        bytes -= entry->length;
        dma += entry->length;
    }
    assert(bytes == 0);
}

int sdmmc_storage_transfer(sdmmc_storage_t *storage, u32 *done, u32 sector, u32 blocks, void *buf, u32 write)
{
    assert(storage == &card && !write && host.prefetch && blocks == ReadAheadBlocks);
    u64 dma = sdmmc_read_ahead_dma(buf, blocks);
    assert(dma && !host.deadline_us);
    unsigned tap = active_tap(&host);
    ++commands;
    sdmmc_req_t request = {.buf = buf, .blksize = 512, .num_sectors = blocks, .is_multi_block = true,
        .auto_command = SdmmcAutoStop};
    host.dma_progress = (sdmmc_dma_progress_t){0};
    host.command_failure = host.command_irq = host.dma_resumes = 0;
    assert(_sdmmc_config_dma(&host, done, &request));
    assert(*done == blocks && !sdmmc_memcpy_buf && regs.blkcnt == blocks);
    assert((regs.trnmod & (SDHCI_TRNS_DMA | SDHCI_TRNS_AUTO_CMD12)) ==
        (SDHCI_TRNS_DMA | SDHCI_TRNS_AUTO_CMD12));
    assert(host.dma_progress.blocks == blocks && host.dma_progress.mode == SdmmcDmaAdma2);
    assert((regs.hostctl & SDHCI_CTRL_DMA_MASK) == SDHCI_CTRL_ADMA32);
    assert(!host.dma_boundaries_left && regs.admaaddr == descriptor_mapping.dma);
    assert_chain(dma, ReadAheadBytes);
    regs.hostctl &= ~SDHCI_CTRL_DMA_MASK;
    if ((int)tap == fault_tap) {
        if (stale_data)
            return 1;
        host.command_failure = DdrCommandTransfer;
        host.command_irq = (u32)fault_irq << 16 | SDHCI_INT_ERROR;
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
    assert(storage == &card && blocks == ReadAheadBlocks && elapsed);
}

bool sdmmc_ddr200_recover(sdmmc_storage_t *storage, u32 sector, bool degrade)
{
    assert(storage == &card && sector < card.sec_cnt && !degrade && !host.prefetch);
    assert(fault_irq != SDHCI_ERR_INT_ADMA_ERROR && (int)active_tap(&host) == fault_tap);
    ++recoveries;
    return true;
}

static void reset(ddr_eye_t eye, unsigned tap)
{
    sdmmc_read_ahead_reset();
    read_ahead = (typeof(read_ahead)){0};
    descriptor_mapping = (dma_window_t){0};
    das = 1;
    card.ddr = (ddr_policy_t){.rate = DdrRateFull, .ready = true, .last_sweep_us = time_us};
    ddr_remember_eye(&card.ddr.history[0], eye);
    regs = (t210_sdmmc_t){.capareg = SDHCI_CAN_ADMA2, .hcver = 3,
        .hostctl2 = SDHCI_HOST_VERSION_4_EN | SDHCI_ADDRESSING_64BIT_EN | UHS_DDR50_BUS_SPEED,
        .hostctl = SDHCI_CTRL_4BITBUS};
    sdmmc_set_ddr200_tap(&host, tap);
    commands = compares = maps = unmaps = recoveries = resets = restarts = 0;
    sequential = served = 0;
    memset(refusals, 0, sizeof(refusals));
    fault_tap = -1;
    fault_irq = SDHCI_ERR_INT_DATA_CRC;
    stale_data = false;
    mapping_ok = true;
}

static bool read_next(u32 sector)
{
    return sdmmc_read_ahead(&card, das, sector, sizeof(output) / 512, output);
}

static void test_dma(void)
{
    reset((ddr_eye_t){29, 36}, 32);
    u64 address = 0x11237ffe8ull;
    assert(sdmmc_adma_table(address, 2 * 1024 * 1024));
    assert_chain(address, SdmmcAdmaBytes);
    assert(sdmmc_adma_table(address, 64 * 1024));
    assert_chain(address, 64 * 1024);
    assert(descriptor_table.entries[1].length == 512);
    assert(!sdmmc_adma_table(address - 1, 4096));
    assert(!sdmmc_adma_table(address, SdmmcAdmaBytes + 1));

    sdmmc_req_t request = {.buf = direct + 8, .blksize = 512, .num_sectors = 32, .is_multi_block = true};
    u32 blocks;
    assert(_sdmmc_config_dma(&host, &blocks, &request));
    assert(blocks == 32 && host.dma_progress.mode == SdmmcDmaAdma2 && !host.dma_boundaries_left);
    assert_chain(0x1237ffe8, 32 * 512);
    assert(host.dma_progress.address_low == 0x7ffe8 && !sdmmc_memcpy_buf);
    regs.blkcnt = 24;
    regs.norintsts = SDHCI_INT_ERROR;
    regs.errintsts = SDHCI_ERR_INT_ADMA_ERROR;
    regs.admaerr = 3;
    assert(!_sdmmc_update_dma(&host));
    assert(resets == 1 && !restarts && host.dma_progress.sampled &&
        host.dma_progress.remaining == 24 && host.dma_progress.adma_error == 3);
    assert(host.command_irq >> 16 == SDHCI_ERR_INT_ADMA_ERROR);

    device.dmaBuffers[0] = (sdmmc_dma_buffer_t){(char *)bounce, sizeof(bounce), (char *)0x20000000};
    request.buf = output;
    request.is_write = true;
    assert(_sdmmc_config_dma(&host, &blocks, &request));
    assert(sdmmc_memcpy_buf && !(regs.trnmod & SDHCI_TRNS_READ));
    assert_chain(0x20000000, 32 * 512);
    request.buf = direct + 1;
    assert(_sdmmc_config_dma(&host, &blocks, &request) && sdmmc_memcpy_buf);
    device.dmaBuffers[0] = (sdmmc_dma_buffer_t){0};

    request.buf = direct;
    regs.capareg = 0;
    regs.hostctl &= ~SDHCI_CTRL_DMA_MASK;
    assert(_sdmmc_config_dma(&host, &blocks, &request));
    assert(host.dma_progress.mode == SdmmcDmaSdma512 && host.dma_boundaries_left == 1);
    reset((ddr_eye_t){29, 36}, 32);
    mapping_ok = false;
    assert(!_sdmmc_config_dma(&host, &blocks, &request));
    assert(host.dma_progress.mode == SdmmcDmaAdma2);

    reset((ddr_eye_t){29, 36}, 32);
    host.handoff_sdma = true;
    regs.hostctl |= SDHCI_CTRL_ADMA32;
    mapping_ok = false;
    request.num_sectors = 1;
    request.blksize = 64;
    request.buf = direct + 32; /* IOVA starts on the next SDMA boundary. */
    assert(_sdmmc_config_dma(&host, &blocks, &request));
    assert(host.dma_progress.mode == SdmmcDmaSdma512 && !maps && !host.dma_boundaries_left);
    assert((regs.hostctl & SDHCI_CTRL_DMA_MASK) == SDHCI_CTRL_SDMA);
    host.handoff_sdma = false;
    mapping_ok = true;
    assert(_sdmmc_config_dma(&host, &blocks, &request));
    assert(host.dma_progress.mode == SdmmcDmaAdma2 && maps == 1);
    ++das;
    assert(sdmmc_adma_remap() && descriptor_mapping.das == das);
    assert(maps == 2 && unmaps == 1);
}

int main(void)
{
    test_dma();

    reset((ddr_eye_t){29, 36}, 32);
    assert(!read_next(0) && read_next(1024));
    assert(samples[DdrDmaBaseline] == DdrDmaOk && sample_taps[DdrDmaBaseline] == 32);
    assert(samples[DdrDmaAlternate] == DdrDmaOk && sample_taps[DdrDmaAlternate] == 33);
    assert(commands == 2 && maps == 2 && !host.prefetch && output[0] == 8);
    assert(!sdmmc_read_ahead_dma(read_window.data + ReadAheadBytes, 1));
    assert(read_next(2048) && read_next(3072) && read_next(4096) && commands == 2);
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){31, 36});
    sdmmc_set_ddr200_tap(&host, 33);
    assert(read_next(5120) && sample_taps[DdrDmaAlternate] == 34 && compares == 2);
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){30, 36});
    assert(read_next(6144) && read_next(7168) && read_next(8192) && read_next(9216));
    assert(compares == 2 && commands == 5);
    sdmmc_read_ahead_reset();
    assert(unmaps == 2 && !read_ahead.adma_verified && !descriptor_mapping.dma);
    ++das;
    assert(!read_next(0) && read_next(1024) && compares == 3 && maps == 4);

    reset((ddr_eye_t){32, 36}, 34);
    assert(!read_next(0) && !read_next(1024) && refusals[DdrBatchEye] == 1);
    assert(!maps && !commands && card.ddr.ready);
    reset((ddr_eye_t){31, 36}, 33);
    fault_tap = 34;
    fault_irq = SDHCI_ERR_INT_ADMA_ERROR;
    assert(!read_next(0) && !read_next(1024));
    assert(samples[DdrDmaAlternate] == DdrDmaFailed && refusals[DdrBatchTapPair] == 1);
    assert(card.ddr.ready && active_tap(&host) == 33 && !recoveries && !read_ahead.blocks);
    reset((ddr_eye_t){31, 36}, 33);
    fault_tap = 33;
    assert(!read_next(0) && !read_next(1024) && recoveries == 1);
    reset((ddr_eye_t){31, 36}, 33);
    fault_tap = 34;
    stale_data = true;
    memset(output, 0x7f, sizeof(output));
    assert(!read_next(0) && !read_next(1024));
    assert(samples[DdrDmaAlternate] == DdrDmaMismatch && output[0] == 0x7f && !read_ahead.blocks);

    reset((ddr_eye_t){31, 36}, 33);
    const u32 small = 64 * 1024 / 512;
    assert(!sdmmc_read_ahead(&card, das, 20000, 1, output));
    for (u32 sector = 0; sector < ReadAheadRunIn; sector += small) {
        assert(!sdmmc_read_ahead(&card, das, sector, small, output));
        assert(!sdmmc_read_ahead(&card, das, 12000 + sector, small, output));
    }
    assert(!commands && refusals[DdrBatchSmall] == 1 && refusals[DdrBatchNonAdjacent] == 2);
    assert(refusals[DdrBatchRunIn] == 14);
    time_us = card.ddr.last_sweep_us + DdrSweepFreshUs;
    /* Keep an active stream while its last sweep expires. */
    for (u32 sector = 0; sector < ReadAheadRunIn; sector += small)
        assert(!sdmmc_read_ahead(&card, das, sector, small, output));
    assert(!sdmmc_read_ahead(&card, das, ReadAheadRunIn, small, output));
    assert(refusals[DdrBatchStale] == 1);
    card.ddr.last_sweep_us = time_us;
    assert(sdmmc_read_ahead(&card, das, ReadAheadRunIn + small, small, output));
    assert(sdmmc_read_ahead(&card, das, ReadAheadRunIn + 2 * small, 1, output));
    assert(output[0] == 10 && commands == 2 && served == small + 1 && sequential >= served);
    sdmmc_read_ahead_invalidate(ReadAheadRunIn + 2 * small + 1, 1);
    assert(!sdmmc_read_ahead(&card, das, ReadAheadRunIn + 2 * small + 1, small, output));
    assert(commands == 2);

    reset((ddr_eye_t){29, 40}, 34);
    assert(!sdmmc_read_ahead(&card, das, 0, ReadAheadLargeRunIn, output));
    assert(sdmmc_read_ahead(&card, das, ReadAheadLargeRunIn, ReadAheadLargeRunIn, output));
    assert(commands == 2 && sample_taps[DdrDmaAlternate] == 33);

    reset((ddr_eye_t){29, 40}, 34);
    for (u32 sector = 0; sector < ReadAheadRunIn; sector += ReadAheadSmallSeed)
        assert(!sdmmc_read_ahead(&card, das, sector, ReadAheadSmallSeed, output));
    assert(!commands && sdmmc_read_ahead(&card, das, ReadAheadRunIn, ReadAheadSmallSeed, output));
    assert(commands == 2 && output[0] == 8);
    puts("ADMA2 chains, resets, guarded batching, stream interleaving and refusal counters passed");
}
