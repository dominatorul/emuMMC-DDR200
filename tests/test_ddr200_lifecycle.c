/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../source/emuMMC/emummc.h"
#define armGetTls() ((void *)0)
#include "../source/emuMMC/emummc.c"

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result)
{ (void)step; (void)detail; (void)result; }
void ddr_trace_wake_host(const sdmmc_t *host) { (void)host; }
#endif

static bool powered;
static t210_sdmmc_t registers;
static sdmmc_accessor_t sd_accessor, nand_accessor;
static unsigned locks, closed, opened, reasserted, handbacks;
static unsigned unmapped, wake_started;
static bool private_mapping, fail_handback, expect_snapshot;
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
void sdmmc_read_ahead_reset(void)
{
    if (private_mapping) {
        assert(locks == 2 && powered && native_sd_transitions);
        assert((registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
        private_mapping = false;
        ++unmapped;
    }
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
    assert(!private_mapping);
    assert(!_ddr200_reassert(FS_SDMMC_EMMC));
    powered = true;
    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    ++opened;
    return 0;
}

static u64 native_close(void *accessor)
{
    assert(!locks);
    if (accessor == &sd_accessor) {
        assert(native_sd_transitions && wait_for_native);
        assert((registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
        assert(!private_mapping);
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
    assert(private_mapping);
    card->ddr.resume_rate = card->ddr.rate;
    card->ddr.rate = DdrRateSdr104;
    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    ++handbacks;
    return !fail_handback;
}

bool sdmmc_ddr200_handoff(sdmmc_storage_t *card)
{
    assert(powered && (registers.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
    assert(!private_mapping);
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
    registers.hostctl2 = UHS_DDR50_BUS_SPEED;
    native_sd_opened = boot_power_window_closed = powered = true;

    for (unsigned cycle = 1; cycle <= 3; ++cycle) {
        private_mapping = true; /* Ordinary I/O before sleep. */
        assert(!controller_close(FS_SDMMC_SD) && closed == cycle - 1);
        assert(!controller_close(FS_SDMMC_EMMC) && closed == cycle && handbacks == cycle && unmapped == cycle);
        assert(!_ddr200_reassert(FS_SDMMC_EMMC) && !powered);
        assert(!controller_open(FS_SDMMC_SD) && opened == cycle);
        assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == cycle && wake_started == cycle);
        assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == cycle);
    }

    registers.hostctl2 = UHS_SDR104_BUS_SPEED;
    assert(_ddr200_reassert(FS_SDMMC_EMMC) && reasserted == 4 && opened == 3);
    private_mapping = fail_handback = expect_snapshot = true;
    if (!setjmp(snapshot)) {
        controller_close(FS_SDMMC_EMMC);
        assert(false);
    }
    assert(!private_mapping && closed == 3 && unmapped == 4);
    puts("Three paired suspend/wake cycles and a host-only wake passed in the lifecycle model");
}
