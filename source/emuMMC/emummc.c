/*
 * Copyright (c) 2019 m4xw <m4x@m4xw.net>
 * Copyright (c) 2019 Atmosphere-NX
 * Copyright (c) 2019 CTCaer
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "emummc.h"
#include "emummc_ctx.h"
#include "../utils/fatal.h"
#include "../utils/ddr_trace.h"
#include "../utils/ddr_watchdog.h"
#include "../utils/log.h"
#include "../nx/cache.h"
#include "../soc/clock.h"
#include "../soc/i2c.h"
#include "../libs/fatfs/diskio.h"
#include "../soc/gpio.h"
#include "../emmc/sd.h"

static bool sdmmc_first_init = false;
static bool storageSDinitialized = false;

// hekate sdmmmc vars
sdmmc_t sdmmc;
sdmmc_storage_t storage;
sdmmc_t sd_sdmmc;
sdmmc_storage_t sd_storage;

// init vars
bool init_done = false;
bool custom_driver = true;

// FS funcs
_sdmmc_accessor_gc sdmmc_accessor_gc;
_sdmmc_accessor_sd sdmmc_accessor_sd;
_sdmmc_accessor_nand sdmmc_accessor_nand;
_lock_mutex lock_mutex;
_unlock_mutex unlock_mutex;

// FS misc
void *sd_mutex;
void *nand_mutex;
volatile int *active_partition;
volatile Handle *sdmmc_das_handle;

// FatFS
file_based_ctxt f_emu;
static bool fat_mounted = false;
static bool native_sd_opened;

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
extern int _sd_storage_enable_DDR200(sdmmc_storage_t *storage, u8 *buf);
extern int _sd_storage_switch_get(sdmmc_storage_t *storage, void *buf);
extern int _sd_storage_switch(sdmmc_storage_t *storage, void *buf, int mode, int group, u32 arg);
extern void _sd_storage_set_current_limit(sdmmc_storage_t *storage, u16 current_limit, u8 *buf);

static bool sdmmc_ddr200_reassert_enabled = true;

static bool _ddr200_power_ready(void)
{
    return sdmmc_get_sd_power_enabled() &&
        clock_sdmmc_is_not_reset_and_enabled(SDMMC_1) &&
        sdmmc_get_io_power(&sd_sdmmc) == SDMMC_POWER_1_8 &&
        sdmmc_get_bus_width(&sd_sdmmc) == SDMMC_BUS_WIDTH_4;
}

#ifdef EMUMMC_FORCE_DDR200
/* Storage mutexes are held. Open uses the native FS regulator service, so
   its power changes are serialized with Horizon's other I2C transactions. */
static int _ddr200_native_reopen(void)
{
    sdmmc_accessor_t *accessor = sdmmc_accessor_get(FS_SDMMC_SD);
    if (!accessor)
        return 0;

    ddr_trace_stage(DdrNativePower, 0);
    if (native_sd_opened) {
        u64 result = accessor->vtab->sdmmc_accessor_controller_close(accessor);
        if (result) {
            ddr_trace_stage(DdrNativePower, result);
            return 0;
        }
        native_sd_opened = false;
    }

    ddr_trace_host(DdrBeforeOpen, &sd_sdmmc);
    u64 result = accessor->vtab->sdmmc_accessor_controller_open(accessor);
    ddr_trace_stage(DdrNativePower, result);
    ddr_trace_host(DdrAfterOpen, &sd_sdmmc);
    native_sd_opened = result == 0;
    return native_sd_opened && _ddr200_power_ready();
}
#endif

static void _ddr200_reassert(int mmc_id)
{
    static u8 ddr200_switch_buf[512];
    static bool reasserting;
    u8 dma_save[512];
    void *dma_scratch = NULL;

    if (!sdmmc_ddr200_reassert_enabled || !sd_storage.initialized)
        return;

    if (nx_sd_mode_get() != SD_UHS_DDR208)
        return;

    ddr_trace_stage(DdrCheckState, mmc_id);
    if (reasserting)
        fatal_abort(Fatal_DdrNativeHost);
    const bool cold = !_ddr200_power_ready();
    if (!cold && (sd_sdmmc.regs->hostctl2 & 0x7) == UHS_DDR50_BUS_SPEED)
        return;

    ddr_trace_reassert();
    ddr_trace_host(DdrReassert, &sd_sdmmc);
    if (_current_accessor != NULL)
    {
        int dma_idx = sdmmc_calculate_fitting_dma_index(_current_accessor, 1);
        sdmmc_dma_buffer_t *dma = &_current_accessor->parent->dmaBuffers[dma_idx];
        if (!dma->device_addr_buffer || dma->device_addr_buffer_size < sizeof(dma_save))
            fatal_abort(Fatal_DdrReassert);
        dma_scratch = dma->device_addr_buffer;
        armDCacheFlush(dma_scratch, sizeof(dma_save));
        memcpy(dma_save, dma_scratch, sizeof(dma_save));
    }

    reasserting = true;
#ifdef EMUMMC_FORCE_DDR200
    int ok = !cold || _ddr200_native_reopen();
    /* Native FS ordinarily reopens in SDR104. If timing is uncertain,
       avoid issuing a CMD6 data transfer using a mismatched host. */
    if (ok && cold && (sd_sdmmc.regs->hostctl2 & 7) == UHS_DDR50_BUS_SPEED) {
        ddr_trace_stage(DdrNativePower, 1);
        ok = 0;
    }
    if (ok)
#else
    int ok;
    if (cold)
    {
        sdmmc_end(&sd_sdmmc);
        sd_storage.initialized = 0;
        clock_sdmmc_invalidate_clock_source(SDMMC_1);
        ok = nx_sd_initialize(false);
    }
    else
#endif
    {
        sdmmc_card_clock_powersave(&sd_sdmmc, SDMMC_POWER_SAVE_DISABLE);
        sdmmc_ddr200_restore_host_cfg(&sd_sdmmc);
        clock_sdmmc_invalidate_clock_source(SDMMC_1);

        ddr_trace_stage(DdrReadFunctions, mmc_id);
        ok = _sd_storage_switch_get(&sd_storage, ddr200_switch_buf);
        if (ok)
        {
            ddr_trace_card(ddr200_switch_buf, false);
            u16 current_limit = ddr200_switch_buf[7] | (ddr200_switch_buf[6] << 8);
            _sd_storage_set_current_limit(&sd_storage, current_limit, ddr200_switch_buf);

            /* The vendor DDR200 switch in Hekate and L4T starts with Access
               Mode at SDR12. Native FS may have selected SDR104. */
            ddr_trace_stage(DdrSetSdr12, mmc_id);
            ok = sdmmc_setup_clock(&sd_sdmmc, SDHCI_TIMING_UHS_SDR12) &&
                _sd_storage_switch(&sd_storage, ddr200_switch_buf, SD_SWITCH_SET, 0, UHS_SDR12_BUS_SPEED) &&
                (ddr200_switch_buf[16] & 0xF) == UHS_SDR12_BUS_SPEED;
            if (ok)
                ok = _sd_storage_enable_DDR200(&sd_storage, ddr200_switch_buf);
        }

        sd_storage.csd.busspeed = ok ? 200 : 0;
        sdmmc_card_clock_powersave(&sd_sdmmc, SDMMC_POWER_SAVE_ENABLE);
    }

    if (dma_scratch != NULL)
    {
        memcpy(dma_scratch, dma_save, sizeof(dma_save));
        armDCacheFlush(dma_scratch, sizeof(dma_save));
    }
    reasserting = false;

    if (!ok)
    {
#ifdef EMUMMC_FORCE_DDR200
        fatal_abort(Fatal_DdrReassert);
#else
        sd_storage.csd.busspeed = 104;
        sdmmc_ddr200_reassert_enabled = false;
#endif
    }

    Log("Reasserting\n");
}
#else
#define _ddr200_reassert(mmc_id) ((void)(mmc_id))
#endif


static void _sdmmc_ensure_device_attached(void)
{
    // This ensures that the sd device address space handle is always attached,
    // even if FS hasn't attached it
    static bool did_attach = false;
    if (!did_attach)
    {
        svcAttachDeviceAddressSpace(DeviceName_SDMMC1A, *sdmmc_das_handle);
        did_attach = true;
    }
}

static void _sdmmc_ensure_initialized(void)
{
    // First Initial init
    if (!sdmmc_first_init)
    {
        sdmmc_initialize();
#ifdef EMUMMC_FORCE_DDR200
        i2c_release_boot_bus();
#endif
        sdmmc_first_init = true;
    }
#ifndef EMUMMC_FORCE_DDR200
    else
    {
        // The boot sysmodule will eventually kill power to SD.
        // Detect this, and reinitialize when it happens.
        if (!init_done)
        {
            if (sdmmc_get_sd_power_enabled() == 0)
            {
                sdmmc_finalize();
                sdmmc_initialize();
                init_done = true;
            }
        }
    }
#endif
}

static void _file_based_update_filename(char *outFilename, unsigned int sd_path_len, unsigned int part_idx)
{
    snprintf(outFilename + sd_path_len, 3, "%02d", part_idx);
}

static void _file_based_emmc_finalize(void)
{
    if ((emuMMC_ctx.EMMC_Type == emuMMC_SD_File) && fat_mounted)
    {
        // Close all open handles.
        f_close(&f_emu.fp_boot0);
        f_close(&f_emu.fp_boot1);

        for (int i = 0; i < f_emu.parts; i++)
            f_close(&f_emu.fp_gpp[i]);

        // Force unmount FAT volume.
        f_mount(NULL, "", 1);

        fat_mounted = false;
    }
}

static void _nand_patrol_ensure_integrity(void)
{
    fs_nand_patrol_t nand_patrol;
    static bool nand_patrol_checked = false;

    if (!nand_patrol_checked)
    {
        if (emuMMC_ctx.EMMC_Type == emuMMC_SD_Raw)
        {
            unsigned int nand_patrol_sector = emuMMC_ctx.EMMC_StoragePartitionOffset + NAND_PATROL_SECTOR;
            if (!sdmmc_storage_read(&sd_storage, nand_patrol_sector, 1, &nand_patrol))
                goto out;

            // Clear nand patrol if last offset exceeds storage.
            if (nand_patrol.offset > sd_storage.sec_cnt)
            {
                memset(&nand_patrol, 0, sizeof(fs_nand_patrol_t));
                sdmmc_storage_write(&sd_storage, nand_patrol_sector, 1, &nand_patrol);
            }
        }
        else if (emuMMC_ctx.EMMC_Type == emuMMC_SD_File && fat_mounted)
        {
            FIL *fp = &f_emu.fp_boot0;
            if (f_lseek(fp, NAND_PATROL_OFFSET) != FR_OK)
                goto out;

            if (f_read_fast(fp, &nand_patrol, sizeof(fs_nand_patrol_t)) != FR_OK)
                goto out;

            // Clear nand patrol if last offset exceeds total file based size.
            if (nand_patrol.offset > f_emu.total_sect)
            {
                memset(&nand_patrol, 0, sizeof(fs_nand_patrol_t));

                if (f_lseek(fp, NAND_PATROL_OFFSET) != FR_OK)
                    goto out;

                if (f_write_fast(fp, &nand_patrol, sizeof(fs_nand_patrol_t)) != FR_OK)
                    goto out;

                f_sync(fp);
            }
        }

out:
        nand_patrol_checked = true;
    }
}

void sdmmc_finalize(void)
{
    if (!sdmmc_storage_end(&sd_storage))
        fatal_abort(Fatal_InitSD);

    storageSDinitialized = false;
}

static void _file_based_emmc_initialize(void)
{
    char path[sizeof(emuMMC_ctx.storagePath) + 0x20];
    memset(&path, 0, sizeof(path));

    memcpy(path, (void *)emuMMC_ctx.storagePath, sizeof(emuMMC_ctx.storagePath));
    strcat(path, "/eMMC/");
    int path_len = strlen(path);

    // Open BOOT0 physical partition.
    memcpy(path + path_len, "BOOT0", 6);
    if (f_open(&f_emu.fp_boot0, path, FA_READ | FA_WRITE) != FR_OK)
        fatal_abort(Fatal_FatfsFileOpen);
    if (!f_expand_cltbl(&f_emu.fp_boot0, EMUMMC_FP_CLMT_COUNT, f_emu.clmt_boot0, f_size(&f_emu.fp_boot0)))
        fatal_abort(Fatal_FatfsMemExhaustion);

    // Open BOOT1 physical partition.
    memcpy(path + path_len, "BOOT1", 6);
    if (f_open(&f_emu.fp_boot1, path, FA_READ | FA_WRITE) != FR_OK)
        fatal_abort(Fatal_FatfsFileOpen);
    if (!f_expand_cltbl(&f_emu.fp_boot1, EMUMMC_FP_CLMT_COUNT, f_emu.clmt_boot1, f_size(&f_emu.fp_boot1)))
        fatal_abort(Fatal_FatfsMemExhaustion);

    // Open handles for GPP physical partition files.
    _file_based_update_filename(path, path_len, 00);

    if (f_open(&f_emu.fp_gpp[0], path, FA_READ | FA_WRITE) != FR_OK)
        fatal_abort(Fatal_FatfsFileOpen);
    if (!f_expand_cltbl(&f_emu.fp_gpp[0], EMUMMC_FP_CLMT_COUNT, &f_emu.clmt_gpp[0], f_size(&f_emu.fp_gpp[0])))
        fatal_abort(Fatal_FatfsMemExhaustion);

    f_emu.part_size = (uint64_t)f_size(&f_emu.fp_gpp[0]) >> 9;
    f_emu.total_sect = f_emu.part_size;

    // Iterate folder for split parts and stop if next doesn't exist.
    for (f_emu.parts = 1; f_emu.parts < EMUMMC_FILE_MAX_PARTS; f_emu.parts++)
    {
        _file_based_update_filename(path, path_len, f_emu.parts);

        if (f_open(&f_emu.fp_gpp[f_emu.parts], path, FA_READ | FA_WRITE) != FR_OK)
        {
            // Check if single file.
            if (f_emu.parts == 1)
                f_emu.parts = 0;

            return;
        }

        if (!f_expand_cltbl(&f_emu.fp_gpp[f_emu.parts], EMUMMC_FP_CLMT_COUNT,
            &f_emu.clmt_gpp[f_emu.parts * EMUMMC_FP_CLMT_COUNT], f_size(&f_emu.fp_gpp[f_emu.parts])))
        {
            fatal_abort(Fatal_FatfsMemExhaustion);
        }

        f_emu.total_sect += (uint64_t)f_size(&f_emu.fp_gpp[f_emu.parts]) >> 9;
    }
}

bool sdmmc_initialize(void)
{
    if (!storageSDinitialized)
    {
#ifdef EMUMMC_FORCE_DDR200
        int retries = 1;
#else
        int retries = 3;
#endif
        while (retries)
        {
            if (nx_sd_initialize(false))
            {
                storageSDinitialized = true;

                // Init file based emummc.
                if ((emuMMC_ctx.EMMC_Type == emuMMC_SD_File) && !fat_mounted)
                {
                    if (f_mount(&f_emu.sd_fs, "", 1) != FR_OK)
                        fatal_abort(Fatal_InitSD);
                    else
                        fat_mounted = true;

                    _file_based_emmc_initialize();
                }

                // Check if nand patrol offset is inside limits.
                _nand_patrol_ensure_integrity();

                break;
            }

            retries--;
        }

        if (!storageSDinitialized)
            fatal_abort(Fatal_InitSD);
    }

    return storageSDinitialized;
}

sdmmc_accessor_t *sdmmc_accessor_get(int mmc_id)
{
    sdmmc_accessor_t *_this;
    switch (mmc_id)
    {
    case FS_SDMMC_EMMC:
        _this = sdmmc_accessor_nand();
        break;
    case FS_SDMMC_SD:
        _this = sdmmc_accessor_sd();
        break;
    case FS_SDMMC_GC:
        _this = sdmmc_accessor_gc();
        break;
    default:
        fatal_abort(Fatal_InvalidAccessor);
    }

    return _this;
}

void mutex_lock_handler(int mmc_id)
{
    uintptr_t thread = (uintptr_t)armGetTls();
    if (custom_driver) {
        ddr_watchdog_phase(thread, DdrOpWaitSd);
        lock_mutex(sd_mutex);
    }

    ddr_watchdog_phase(thread, DdrOpWaitNand);
    lock_mutex(nand_mutex);
    ddr_watchdog_phase(thread, DdrOpRunning);
}

void mutex_unlock_handler(int mmc_id)
{
    ddr_watchdog_phase((uintptr_t)armGetTls(), DdrOpUnlock);
    unlock_mutex(nand_mutex);

    if (custom_driver)
        unlock_mutex(sd_mutex);
}

int sdmmc_nand_get_active_partition_index()
{
    switch (*active_partition)
    {
    case FS_EMMC_PARTITION_GPP:
        return 2;
    case FS_EMMC_PARTITION_BOOT1:
        return 1;
    case FS_EMMC_PARTITION_BOOT0:
        return 0;
    }

    fatal_abort(Fatal_InvalidAccessor);
}

static uint64_t emummc_read_write_inner(void *buf, unsigned int sector, unsigned int num_sectors, bool is_write)
{
    if (emuMMC_ctx.EMMC_Type == emuMMC_SD_Raw)
    {
        // raw partition sector offset: emuMMC_ctx.EMMC_StoragePartitionOffset.
        sector += emuMMC_ctx.EMMC_StoragePartitionOffset;
        // Set physical partition offset.
        sector += (sdmmc_nand_get_active_partition_index() * BOOT_PARTITION_SIZE);

        if (__builtin_expect(sector + num_sectors > sd_storage.sec_cnt, 0))
            return 0; // Out of bounds. Can only happen with Nand Patrol if resized.

        if (!is_write)
            return sdmmc_storage_read(&sd_storage, sector, num_sectors, buf);
        else
            return sdmmc_storage_write(&sd_storage, sector, num_sectors, buf);
    }

    // File based emummc.
    FIL *fp = NULL;
    switch (*active_partition)
    {
    case FS_EMMC_PARTITION_GPP:
        if (f_emu.parts)
        {
            if (__builtin_expect(sector + num_sectors > f_emu.total_sect, 0))
                return 0; // Out of bounds. Can only happen with Nand Patrol if resized.

            fp = &f_emu.fp_gpp[sector / f_emu.part_size];
            sector = sector % f_emu.part_size;

            // Special handling for reads/writes which cross file-boundaries.
            if (__builtin_expect(sector + num_sectors > f_emu.part_size, 0))
            {
                unsigned int remaining = num_sectors;
                while (remaining > 0) {
                    const unsigned int cur_sectors = MIN(remaining, f_emu.part_size - sector);

                    if (f_lseek(fp, (uint64_t)sector << 9) != FR_OK)
                        return 0; // Out of bounds.

                    if (!is_write)
                    {
                        if (f_read_fast(fp, buf, (uint64_t)cur_sectors << 9) != FR_OK)
                            return 0;
                    }
                    else
                    {
                        if (f_write_fast(fp, buf, (uint64_t)cur_sectors << 9) != FR_OK)
                            return 0;
                    }

                    buf = (char *)buf + ((uint64_t)cur_sectors << 9);
                    remaining -= cur_sectors;
                    sector = 0;
                    ++fp;
                }

                return 1;
            }
        }
        else
            fp = &f_emu.fp_gpp[0];
        break;

    case FS_EMMC_PARTITION_BOOT1:
        fp = &f_emu.fp_boot1;
        break;

    case FS_EMMC_PARTITION_BOOT0:
        fp = &f_emu.fp_boot0;
        break;
    }

    if (f_lseek(fp, (uint64_t)sector << 9) != FR_OK)
        return 0; // Out of bounds. Can only happen with Nand Patrol if resized.

    if (!is_write)
        return !f_read_fast(fp, buf, (uint64_t)num_sectors << 9);
    else
        return !f_write_fast(fp, buf, (uint64_t)num_sectors << 9);
}

// Controller open wrapper
static uint64_t controller_open(int mmc_id)
{
    uint64_t result;
    sdmmc_accessor_t *_this;
    _this = sdmmc_accessor_get(mmc_id);

    if (_this != NULL)
    {
        // Lock eMMC xfer while SD card is being initialized by FS.
        if (mmc_id == FS_SDMMC_SD)
            mutex_lock_handler(FS_SDMMC_EMMC); // Recursive Mutex, handler will lock SD as well if custom_driver

        bool trace_open = mmc_id == FS_SDMMC_SD && sd_storage.initialized;
        if (trace_open) {
            ddr_trace_stage(DdrNativeOpen, mmc_id);
            ddr_trace_host(DdrBeforeOpen, &sd_sdmmc);
        }
        result = _this->vtab->sdmmc_accessor_controller_open(_this);
        if (!result && mmc_id == FS_SDMMC_SD)
            native_sd_opened = true;
        if (trace_open) {
            ddr_trace_stage(DdrNativeOpen, result);
            ddr_trace_host(DdrAfterOpen, &sd_sdmmc);
        }

        // Unlock eMMC.
        if (mmc_id == FS_SDMMC_SD)
            mutex_unlock_handler(FS_SDMMC_EMMC);

        return result;
    }

    fatal_abort(Fatal_OpenAccessor);
}

// Controller close wrapper
static uint64_t controller_close(int mmc_id)
{
    sdmmc_accessor_t *_this;
    _this = sdmmc_accessor_get(mmc_id);

    if (_this != NULL)
    {
        if (mmc_id == FS_SDMMC_SD)
            return 0;

        if (mmc_id == FS_SDMMC_EMMC)
        {
            // Close file handles and unmount
            _file_based_emmc_finalize();

            // Close SD
            sdmmc_accessor_t *sd = sdmmc_accessor_get(FS_SDMMC_SD);
            if (!sd->vtab->sdmmc_accessor_controller_close(sd))
                native_sd_opened = false;

            // Close eMMC
            return _this->vtab->sdmmc_accessor_controller_close(_this);
        }

        return _this->vtab->sdmmc_accessor_controller_close(_this);
    }

    fatal_abort(Fatal_CloseAccessor);
}

// FS read wrapper.
static uint64_t controller_read(void *buf, uint64_t bufSize, int mmc_id, unsigned int sector, unsigned int num_sectors)
{
    sdmmc_accessor_t *_this;
    uint64_t read_res;

    _this = sdmmc_accessor_get(mmc_id);

    if (_this != NULL)
    {
        if (mmc_id == FS_SDMMC_EMMC || mmc_id == FS_SDMMC_SD)
        {
            mutex_lock_handler(mmc_id);
            // Assign FS accessor to the SDMMC driver
            _current_accessor = _this;
            // Make sure we're attached to the device address space.
            _sdmmc_ensure_device_attached();
            // Make sure we're still initialized if boot killed sd card power.
            _sdmmc_ensure_initialized();
            _ddr200_reassert(mmc_id);
            ddr_trace_stage(DdrRead, mmc_id);
        }

        if (mmc_id == FS_SDMMC_EMMC)
        {
            // Call hekates driver.
            if (emummc_read_write_inner(buf, sector, num_sectors, false))
            {
                ddr_trace_io(false);
                mutex_unlock_handler(mmc_id);
                return 0;
            }

#ifdef EMUMMC_FORCE_DDR200
            fatal_abort(Fatal_DdrRead);
#endif
            mutex_unlock_handler(mmc_id);
            return FS_READ_WRITE_ERROR;
        }

        if (mmc_id == FS_SDMMC_SD)
        {
            static bool first_sd_read = true;
            if (first_sd_read)
            {
                first_sd_read = false;
                if (emuMMC_ctx.EMMC_Type == emuMMC_SD_Raw && !nx_sd_is_ddr200())
                {
                    // Because some SD cards have issues with emuMMC's driver
                    // we currently swap to FS's driver after first SD read
                    // for raw based emuMMC
                    custom_driver = false;
                    // FS will handle sd mutex w/o custom driver from here on
                    unlock_mutex(sd_mutex);
                }
            }

            // Call hekate's driver.
            if (sdmmc_storage_read(&sd_storage, sector, num_sectors, buf))
            {
                ddr_trace_io(false);
                mutex_unlock_handler(mmc_id);
                return 0;
            }

#ifdef EMUMMC_FORCE_DDR200
            fatal_abort(Fatal_DdrRead);
#endif
            mutex_unlock_handler(mmc_id);
            return FS_READ_WRITE_ERROR;
        }

        read_res = _this->vtab->read_write(_this, sector, num_sectors, buf, bufSize, 1);
        return read_res;
    }

    fatal_abort(Fatal_ReadNoAccessor);
}

// FS write wrapper.
static uint64_t controller_write(int mmc_id, unsigned int sector, unsigned int num_sectors, void *buf, uint64_t bufSize)
{
    sdmmc_accessor_t *_this;
    uint64_t write_res;

    _this = sdmmc_accessor_get(mmc_id);

    if (_this != NULL)
    {
        if (mmc_id == FS_SDMMC_EMMC)
        {
            mutex_lock_handler(mmc_id);
            _current_accessor = _this;
            _sdmmc_ensure_device_attached();
            if (!sdmmc_first_init)
                fatal_abort(Fatal_DdrWrite);
            _ddr200_reassert(mmc_id);
            ddr_trace_stage(DdrWrite, mmc_id);

            // Call hekates driver.
            if (emummc_read_write_inner(buf, sector, num_sectors, true))
            {
                ddr_trace_io(true);
                mutex_unlock_handler(mmc_id);
                return 0;
            }

#ifdef EMUMMC_FORCE_DDR200
            fatal_abort(Fatal_DdrWrite);
#endif
            mutex_unlock_handler(mmc_id);
            return FS_READ_WRITE_ERROR;
        }

        if (mmc_id == FS_SDMMC_SD)
        {
            mutex_lock_handler(mmc_id);
            _current_accessor = _this;
            _sdmmc_ensure_device_attached();

            if (!sdmmc_first_init)
                fatal_abort(Fatal_DdrWrite);
            _ddr200_reassert(mmc_id);
            ddr_trace_stage(DdrWrite, mmc_id);

            // Call hekates driver.
            if (sdmmc_storage_write(&sd_storage, sector, num_sectors, buf))
            {
                ddr_trace_io(true);
                mutex_unlock_handler(mmc_id);
                return 0;
            }

#ifdef EMUMMC_FORCE_DDR200
            fatal_abort(Fatal_DdrWrite);
#endif
            mutex_unlock_handler(mmc_id);
            return FS_READ_WRITE_ERROR;
        }

        write_res = _this->vtab->read_write(_this, sector, num_sectors, buf, bufSize, 0);
        return write_res;
    }

    fatal_abort(Fatal_WriteNoAccessor);
}

uint64_t sdmmc_wrapper_controller_open(int mmc_id)
{
    u32 slot = ddr_watchdog_enter((uintptr_t)armGetTls(), DdrOpOpen, mmc_id);
    u64 result = controller_open(mmc_id);
    ddr_watchdog_leave(slot);
    return result;
}

uint64_t sdmmc_wrapper_controller_close(int mmc_id)
{
    u32 slot = ddr_watchdog_enter((uintptr_t)armGetTls(), DdrOpClose, mmc_id);
    u64 result = controller_close(mmc_id);
    ddr_watchdog_leave(slot);
    return result;
}

uint64_t sdmmc_wrapper_read(void *buf, uint64_t bufSize, int mmc_id, unsigned int sector, unsigned int num_sectors)
{
    u32 slot = ddr_watchdog_enter((uintptr_t)armGetTls(), DdrOpRead, mmc_id);
    u64 result = controller_read(buf, bufSize, mmc_id, sector, num_sectors);
    ddr_watchdog_leave(slot);
    return result;
}

uint64_t sdmmc_wrapper_write(int mmc_id, unsigned int sector, unsigned int num_sectors, void *buf, uint64_t bufSize)
{
    u32 slot = ddr_watchdog_enter((uintptr_t)armGetTls(), DdrOpWrite, mmc_id);
    u64 result = controller_write(mmc_id, sector, num_sectors, buf, bufSize);
    ddr_watchdog_leave(slot);
    return result;
}
