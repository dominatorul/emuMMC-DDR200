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
static unsigned native_sd_transitions;

enum { BootPowerRecoveryLimit = 2, BootPowerWindowUs = 10000000 };
static bool boot_power_window_closed;
static bool wait_for_native;
static u64 boot_power_deadline;
static u32 boot_power_recoveries;

#ifdef EMUMMC_DDR200_MATRIX
static u64 matrix_next_step;
static bool matrix_finished;
#endif

static bool _ddr200_power_ready(void)
{
    return sdmmc_get_sd_power_enabled() &&
        clock_sdmmc_is_not_reset_and_enabled(SDMMC_1) &&
        sdmmc_get_io_power(&sd_sdmmc) == SDMMC_POWER_1_8 &&
        sdmmc_get_bus_width(&sd_sdmmc) == SDMMC_BUS_WIDTH_4;
}

static void _ddr200_close_boot_power_window(void)
{
    if (!boot_power_window_closed) {
        boot_power_window_closed = true;
        i2c_release_boot_bus();
    }
}

static bool _ddr200_recover_boot_power(void)
{
    ++boot_power_recoveries;
    ddr_trace_reassert();
    ddr_trace_boot_power();
    sdmmc_end(&sd_sdmmc);
    sd_storage.initialized = 0;
    clock_sdmmc_invalidate_clock_source(SDMMC_1);
    int ok = nx_sd_initialize(false);
    if (!ok || boot_power_recoveries == BootPowerRecoveryLimit)
        _ddr200_close_boot_power_window();
    return ok;
}

static bool _ddr200_reassert(int mmc_id)
{
    if (native_sd_transitions)
        return false;
    if (!sd_storage.initialized)
        return false;
    if (!boot_power_window_closed && get_tmr_us() >= boot_power_deadline)
        _ddr200_close_boot_power_window();
    if (!sd_storage.ddr.capable)
        return sdmmc_ddr200_profile_ready(&sd_storage) && !wait_for_native;
    if (!_ddr200_power_ready()) {
        if (!boot_power_window_closed && !sdmmc_get_sd_power_enabled())
            return _ddr200_recover_boot_power();

        if (!wait_for_native) {
            ddr_trace_host(DdrReassert, &sd_sdmmc);
            ddr_trace_wait_native();
        }
        wait_for_native = true;
        ddr_trace_stage(DdrWaitForNative, mmc_id);
        return false;
    }
    if (!native_sd_opened)
        return sdmmc_ddr200_profile_ready(&sd_storage) && !wait_for_native;

    u32 mode = sd_sdmmc.regs->hostctl2 & 7;
    if (mode == UHS_DDR50_BUS_SPEED)
        return sdmmc_ddr200_profile_ready(&sd_storage) && !wait_for_native;
    if (mode != UHS_SDR104_BUS_SPEED)
        return false;
    if (!wait_for_native && sd_storage.ddr.rate == DdrRateSdr104 &&
        sdmmc_ddr200_profile_ready(&sd_storage))
        return true;

    ddr_trace_host(DdrReassert, &sd_sdmmc);
    clock_sdmmc_invalidate_clock_source(SDMMC_1);
    bool resuming = sd_storage.ddr.resume_rate != DdrRateUnknown;
    bool ready = sdmmc_ddr200_handoff(&sd_storage);
    wait_for_native = !ready;
    if (resuming)
        ddr_trace_power(DdrPowerReasserted, ready);
#ifdef EMUMMC_DDR200_MATRIX
    if (ready && sd_storage.ddr.rate >= DdrRateReduced && !matrix_next_step)
        matrix_next_step = get_tmr_us() + 20000000;
#endif
    return ready;
}

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

static bool _sdmmc_ensure_initialized(void)
{
    if (sdmmc_first_init)
        return true;
    if (boot_power_window_closed)
        return false;
    if (!boot_power_deadline)
        boot_power_deadline = get_tmr_us() + BootPowerWindowUs;
    sdmmc_first_init = sdmmc_initialize();
    return sdmmc_first_init;
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

static bool _file_based_ensure_mounted(void)
{
    if (emuMMC_ctx.EMMC_Type != emuMMC_SD_File || fat_mounted)
        return true;
    if (f_mount(&f_emu.sd_fs, "", 1) != FR_OK)
        return false;
    _file_based_emmc_initialize();
    fat_mounted = true;
    return true;
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

                if (!_file_based_ensure_mounted())
                    fatal_abort(Fatal_InitSD);

                // Check if nand patrol offset is inside limits.
                _nand_patrol_ensure_integrity();

                break;
            }

            retries--;
        }

#ifdef EMUMMC_FORCE_DDR200
        if (!storageSDinitialized)
            fatal_abort(Fatal_InitSD);
#endif
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

#ifdef EMUMMC_DDR200_MATRIX
static void _ddr200_matrix_poll(void)
{
    if (matrix_finished || !matrix_next_step)
        return;
    if (ddr_watchdog_volume_down()) {
        matrix_finished = true;
        sd_storage.ddr.matrix_active = false;
        ddr_trace_matrix(DdrMatrixSkipped, 0, 0);
        return;
    }
    if (get_tmr_us() < matrix_next_step)
        return;
    matrix_finished = !sdmmc_ddr200_matrix_step(&sd_storage);
    matrix_next_step = get_tmr_us() + 20000;
}
#endif

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
        if (mmc_id != FS_SDMMC_SD)
            return _this->vtab->sdmmc_accessor_controller_open(_this);

        mutex_lock_handler(FS_SDMMC_EMMC);
        ++native_sd_transitions;
        _ddr200_close_boot_power_window();
        bool trace_open = sd_storage.initialized;
        if (trace_open) {
            ddr_trace_stage(DdrNativeOpen, mmc_id);
            ddr_trace_host(DdrBeforeOpen, &sd_sdmmc);
        }
        mutex_unlock_handler(FS_SDMMC_EMMC);

        /* Horizon runs its ordinary SD open while emuMMC I/O waits for
           the in-flight flag to clear. */
        result = _this->vtab->sdmmc_accessor_controller_open(_this);
        mutex_lock_handler(FS_SDMMC_EMMC);
        --native_sd_transitions;
        native_sd_opened = !result;
        if (sd_storage.initialized) {
            bool retained_ddr = sd_storage.ddr.ready && _ddr200_power_ready() &&
                (sd_sdmmc.regs->hostctl2 & 7) == UHS_DDR50_BUS_SPEED;
            wait_for_native = result || (sd_storage.ddr.capable && (wait_for_native || !retained_ddr));
        }
        if (trace_open) {
            ddr_trace_stage(DdrNativeOpen, result);
            ddr_trace_host(DdrAfterOpen, &sd_sdmmc);
            ddr_trace_power(DdrPowerOpened, !result);
        }

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
            sdmmc_accessor_t *sd = sdmmc_accessor_get(FS_SDMMC_SD);
            mutex_lock_handler(FS_SDMMC_EMMC);
            sdmmc_accessor_t *previous = _current_accessor;
            _current_accessor = _this;
            _file_based_emmc_finalize();
            bool close_sd = sd && native_sd_opened && !native_sd_transitions;
            if (close_sd) {
                ++native_sd_transitions;
                wait_for_native = true;
                _current_accessor = sd;
                ddr_trace_stage(DdrNativeClose, 0);
                ddr_trace_power(DdrPowerClosing, true);
                bool standard = !sd_storage.ddr.capable ||
                    (_ddr200_power_ready() && sdmmc_ddr200_prepare_suspend(&sd_storage));
                ddr_trace_power(DdrPowerHandback, standard);
            }
            _current_accessor = previous;
            mutex_unlock_handler(FS_SDMMC_EMMC);

            /* Only the actual eMMC close pairs with an SD close. Let
               Horizon run its regulator lifecycle with our locks released. */
            if (close_sd) {
                u64 result = sd->vtab->sdmmc_accessor_controller_close(sd);
                mutex_lock_handler(FS_SDMMC_EMMC);
                --native_sd_transitions;
                native_sd_opened = result != 0;
                ddr_trace_stage(DdrNativeClose, result);
                ddr_trace_power(DdrPowerClosed, !result);
                mutex_unlock_handler(FS_SDMMC_EMMC);
            }
        }

        return _this->vtab->sdmmc_accessor_controller_close(_this);
    }

    fatal_abort(Fatal_CloseAccessor);
}

/* Leave both mutexes owned on return. If Horizon is still restoring power,
   give it a chance to acquire the storage locks between readiness checks. */
static bool _sdmmc_prepare_io(int mmc_id, sdmmc_accessor_t *accessor)
{
    u64 deadline = get_tmr_us() + 1500000;
    for (;;) {
        mutex_lock_handler(mmc_id);
        if (!native_sd_transitions) {
            _current_accessor = accessor;
            _sdmmc_ensure_device_attached();
            if (!_sdmmc_ensure_initialized())
                return false;
            if (_ddr200_reassert(mmc_id)) {
#ifdef EMUMMC_DDR200_MATRIX
                if (mmc_id == FS_SDMMC_SD)
                    _ddr200_matrix_poll();
                if (!sdmmc_ddr200_profile_ready(&sd_storage))
                    return false;
#endif
                return mmc_id != FS_SDMMC_EMMC || _file_based_ensure_mounted();
            }
        }
        bool pending = native_sd_transitions ||
            (wait_for_native && (!native_sd_opened || !_ddr200_power_ready()));
        if (!pending || get_tmr_us() >= deadline)
            return false;
        mutex_unlock_handler(mmc_id);
        svcSleepThread(10000000);
    }
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
            if (!_sdmmc_prepare_io(mmc_id, _this)) {
#ifdef EMUMMC_FORCE_DDR200
                fatal_abort(Fatal_DdrReassert);
#endif
                mutex_unlock_handler(mmc_id);
                return FS_READ_WRITE_ERROR;
            }
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
            if (!_sdmmc_prepare_io(mmc_id, _this)) {
#ifdef EMUMMC_FORCE_DDR200
                fatal_abort(Fatal_DdrReassert);
#endif
                mutex_unlock_handler(mmc_id);
                return FS_READ_WRITE_ERROR;
            }
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
            if (!_sdmmc_prepare_io(mmc_id, _this)) {
#ifdef EMUMMC_FORCE_DDR200
                fatal_abort(Fatal_DdrReassert);
#endif
                mutex_unlock_handler(mmc_id);
                return FS_READ_WRITE_ERROR;
            }
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
    u64 started = mmc_id == FS_SDMMC_SD ? get_tmr_us() : 0;
    u64 result = controller_read(buf, bufSize, mmc_id, sector, num_sectors);
    if (!result && mmc_id == FS_SDMMC_SD)
        ddr_trace_fs_perf(false, num_sectors, get_tmr_us() - started);
    ddr_watchdog_leave(slot);
    return result;
}

uint64_t sdmmc_wrapper_write(int mmc_id, unsigned int sector, unsigned int num_sectors, void *buf, uint64_t bufSize)
{
    u32 slot = ddr_watchdog_enter((uintptr_t)armGetTls(), DdrOpWrite, mmc_id);
    u64 started = mmc_id == FS_SDMMC_SD ? get_tmr_us() : 0;
    u64 result = controller_write(mmc_id, sector, num_sectors, buf, bufSize);
    if (!result && mmc_id == FS_SDMMC_SD)
        ddr_trace_fs_perf(true, num_sectors, get_tmr_us() - started);
    ddr_watchdog_leave(slot);
    return result;
}
