/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../source/emuMMC/emummc.h"
#define armGetTls() ((void *)0)
#include "../source/emuMMC/emummc.c"
#include "../source/emmc/sdmmc_read_ahead.c"

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result)
{ (void)step; (void)detail; (void)result; }
void ddr_trace_wake_host(const sdmmc_t *host) { (void)host; }
#endif

static bool powered;
static t210_sdmmc_t registers;
static sdmmc_accessor_t sd_accessor, nand_accessor;
static unsigned locks, closed, opened, reasserted, handbacks;
static unsigned desc_maps, window_maps, desc_unmaps, window_unmaps, wake_started;
static Handle das = 1;
static u32 pllc4_base = 0x58006804;
static bool fail_handback, expect_snapshot;
static jmp_buf snapshot;
static char sd_lock, nand_lock;
volatile emuMMC_ctx_t emuMMC_ctx = {.EMMC_Type = emuMMC_SD_Raw};
sdmmc_accessor_t *_current_accessor;

u64 get_tmr_us(void) { return 11000000; }
int sdmmc_get_sd_power_enabled(void) { return powered; }
int clock_sdmmc_is_not_reset_and_enabled(u32 id) { assert(id == SDMMC_1); return powered; }
int sdmmc_get_io_power(sdmmc_t *host) { assert(host == &sd_sdmmc); return SDMMC_POWER_1_8; }
u32 sdmmc_get_bus_width(sdmmc_t *host) { assert(host == &sd_sdmmc); return SDMMC_BUS_WIDTH_4; }
void clock_sdmmc_invalidate_clock_source(u32 id) { assert(id == SDMMC_1 && powered); }
void i2c_release_boot_bus(void) { assert(false); }
void sdmmc_end(sdmmc_t *host) { (void)host; assert(false); }
bool nx_sd_initialize(bool cycle) { (void)cycle; assert(false); return false; }
void ddr_trace_boot_power(void) { assert(false); }
void ddr_trace_reassert(void) { assert(false); }
void ddr_trace_stage(enum DdrStage stage, u32 detail) { (void)stage; (void)detail; }
void ddr_trace_host(enum DdrSnapshot slot, const sdmmc_t *host) { (void)slot; (void)host; }
void ddr_trace_power(enum DdrPowerEvent event, bool ok)
{
    assert(ok || (event == DdrPowerHandback && fail_handback));
    if (event == DdrPowerReasserting)
        ++wake_started;
}
void ddr_trace_wait_native(void) {}
void ddr_watchdog_phase(uintptr_t thread, enum DdrOpPhase phase) { (void)thread; (void)phase; }
void fatal_abort(enum FatalReason reason)
{
    assert(expect_snapshot && reason == Fatal_DdrWake);
    longjmp(snapshot, 1);
}
static bool private_mapping(void)
{
    return descriptor_mapping.dma || read_ahead.mapping.dma;
}

Result svcMapDeviceAddressSpaceAligned(Handle handle, Handle process, u64 address, u64 size,
    u64 device_address, u32 permission)
{
    assert(handle == das && process == current_process);
    assert(((address ^ device_address) & (DeviceAlignment - 1)) == 0);
    if (address == (uintptr_t)&descriptor_table) {
        assert(size == sizeof(descriptor_table) && permission == Perm_R);
        ++desc_maps;
    } else {
        assert(address == (uintptr_t)&read_window && size == sizeof(read_window) && permission == Perm_W);
        ++window_maps;
    }
    return 0;
}

Result svcUnmapDeviceAddressSpace(Handle handle, Handle process, u64 address, u64 size, u64 device_address)
{
    assert(handle == das && process == current_process && locks == 2 && powered);
    assert((registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
    if (address == (uintptr_t)&descriptor_table) {
        assert(size == sizeof(descriptor_table) && device_address == descriptor_mapping.dma);
        ++desc_unmaps;
    } else {
        assert(address == (uintptr_t)&read_window && size == sizeof(read_window));
        assert(device_address == read_ahead.mapping.dma);
        ++window_unmaps;
    }
    return 0;
}

void armDCacheClean(void *buf, size_t bytes)
{
    assert(buf == descriptor_table.entries && bytes == sizeof(descriptor_table.entries[0]));
}

void ddr_trace_read_ahead_map(Result result) { assert(!result); }
void ddr_trace_descriptor_mapping(bool unmap, u32 handle, Result result)
{
    (void)unmap;
    assert(handle == das && !result);
}

static void map_first_commands(void)
{
    /* The first write maps only descriptors. A subsequent qualified read
       maps the window, reusing the descriptor page for its transfer. */
    unsigned previous_descriptors = desc_maps, previous_windows = window_maps;
    assert(!private_mapping() && !read_ahead.blocks && !read_ahead.adma_verified);
    assert(sdmmc_adma_table(0x8000, 512) && desc_maps == previous_descriptors + 1);
    assert(window_maps == previous_windows && descriptor_mapping.das == das);
    assert(map_window(&read_ahead.mapping, das, &read_window, sizeof(read_window), Perm_W));
    assert(sdmmc_adma_table(0x8000, 512) && desc_maps == previous_descriptors + 1);
    assert(window_maps == previous_windows + 1 && read_ahead.mapping.das == das);
    read_ahead.blocks = ReadAheadBlocks;
    read_ahead.adma_verified = true;
}

FRESULT f_close(FIL *file) { (void)file; assert(false); return FR_INT_ERR; }
FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE option)
{
    (void)fs; (void)path; (void)option;
    assert(false);
    return FR_INT_ERR;
}

static void acquire(void *mutex)
{
    assert((locks == 0 && mutex == &sd_lock) || (locks == 1 && mutex == &nand_lock));
    ++locks;
}

static void release(void *mutex)
{
    assert((locks == 2 && mutex == &nand_lock) || (locks == 1 && mutex == &sd_lock));
    --locks;
}

static sdmmc_accessor_t *get_sd(void) { return &sd_accessor; }
static sdmmc_accessor_t *get_nand(void) { return &nand_accessor; }

static u64 native_open(void *accessor)
{
    assert(accessor == &sd_accessor && !locks && native_sd_transitions);
    assert(!private_mapping());
    assert(!_ddr200_reassert(FS_SDMMC_EMMC));
    powered = true;
    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    ++das;
    pllc4_base = opened & 1 ? 0x58006804 : 0x58003402;
    ++opened;
    return 0;
}

static u64 native_close(void *accessor)
{
    assert(!locks);
    if (accessor == &sd_accessor) {
        assert(native_sd_transitions && wait_for_native);
        assert((registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
        assert(!private_mapping());
        ++closed;
        powered = false;
    } else {
        assert(accessor == &nand_accessor);
    }
    return 0;
}

bool sdmmc_ddr200_profile_ready(const sdmmc_storage_t *card) { return card->ddr.ready; }

bool sdmmc_ddr200_prepare_suspend(sdmmc_storage_t *card)
{
    assert(locks == 2 && _current_accessor == &sd_accessor && powered);
    assert(card->ddr.rate == DdrRateFull);
    assert(private_mapping());
    card->ddr.resume_rate = card->ddr.rate;
    card->ddr.rate = DdrRateSdr104;
    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    ++handbacks;
    return !fail_handback;
}

bool sdmmc_ddr200_handoff(sdmmc_storage_t *card)
{
    assert(powered && (registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
    assert(!private_mapping());
    assert(clock_pllc4_ddr200_compatible(pllc4_base));
    ++reasserted;
    card->ddr.rate = DdrRateFull;
    card->ddr.resume_rate = DdrRateUnknown;
    registers.hostctl2 = UHS_DDR50_BUS_SPEED;
    return true;
}

int main(void)
{
    assert(clock_pllc4_ddr200_compatible(0x58006804));
    assert(clock_pllc4_ddr200_compatible(0x58003402));
    assert(!clock_pllc4_ddr200_compatible(0x58003502));
    assert(!clock_pllc4_ddr200_compatible(0x58083402));
    assert(!clock_pllc4_ddr200_compatible(0x50003402));
    assert(!clock_pllc4_ddr200_compatible(0x58000000));

    static sdmmc_accessor_vt_t vtable = {
        .sdmmc_accessor_controller_open = native_open, .sdmmc_accessor_controller_close = native_close,
    };
    sd_accessor.vtab = nand_accessor.vtab = &vtable;
    sdmmc_accessor_sd = get_sd;
    sdmmc_accessor_nand = get_nand;
    sd_mutex = &sd_lock;
    nand_mutex = &nand_lock;
    lock_mutex = acquire;
    unlock_mutex = release;
    sd_storage.initialized = sd_storage.ddr.capable = sd_storage.ddr.ready = true;
    sd_storage.ddr.rate = DdrRateFull;
    sd_sdmmc.regs = &registers;
    sdmmc_das_handle = &das;
    registers.hostctl2 = UHS_DDR50_BUS_SPEED;
    native_sd_opened = boot_power_window_closed = powered = true;

    map_first_commands();
    for (unsigned cycle = 1; cycle <= 3; ++cycle) {
        assert(!controller_close(FS_SDMMC_SD) && closed == cycle - 1);
        assert(!controller_close(FS_SDMMC_EMMC) && closed == cycle && handbacks == cycle);
        assert(desc_unmaps == cycle && window_unmaps == cycle && !private_mapping());
        assert(!_ddr200_reassert(FS_SDMMC_EMMC) && !powered);
        assert(!controller_open(FS_SDMMC_SD) && opened == cycle);
        assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == cycle && wake_started == cycle);
        assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == cycle);
        assert(sd_sdmmc.wake_commands == 8 && !read_ahead.blocks);
        map_first_commands();
        assert(desc_maps == cycle + 1 && window_maps == cycle + 1);
    }

    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    acquire(&sd_lock);
    acquire(&nand_lock);
    assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == 4 && opened == 3);
    release(&nand_lock);
    release(&sd_lock);
    assert(desc_unmaps == 4 && window_unmaps == 4);
    map_first_commands();
    fail_handback = expect_snapshot = true;
    if (!setjmp(snapshot)) {
        controller_close(FS_SDMMC_EMMC);
        assert(false);
    }
    assert(!private_mapping() && closed == 3 && desc_unmaps == 5 && window_unmaps == 5);
    puts("Three suspend/wake cycles and one host-only wake retired and remapped both ADMA windows exactly once");
}
