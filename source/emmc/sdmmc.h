/*
 * Copyright (c) 2018 naehrwert
 * Copyright (c) 2018-2020 CTCaer
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

#ifndef _SDMMC_H_
#define _SDMMC_H_

#include "../utils/types.h"
#include "../FS/FS.h"
#include "sdmmc_driver.h"
#include "ddr200_policy.h"

typedef struct _mmc_cid
{
	u32 manfid;
	u8  prod_name[8];
	u8  card_bga;
	u8  prv;
	u32 serial;
	u16 oemid;
	u16	year;
	u8  hwrev;
	u8  fwrev;
	u8  month;
} mmc_cid_t;

typedef struct _mmc_csd
{
	u8  structure;
	u8  mmca_vsn;
	u16 cmdclass;
	u32 c_size;
	u32 r2w_factor;
	u32 max_dtr;
	u32 erase_size;		/* In sectors */
	u32 read_blkbits;
	u32 write_blkbits;
	u32 capacity;
	u8  write_protect;
	u16 busspeed;
} mmc_csd_t;

typedef struct _mmc_ext_csd
{
	u32 sectors;
	int bkops;        /* background support bit */
	int bkops_en;     /* manual bkops enable bit */
	u8  rev;
	u8  ext_struct;   /* 194 */
	u8  card_type;    /* 196 */
	u8  bkops_status; /* 246 */
	u8  pre_eol_info;
	u8  dev_life_est_a;
	u8  dev_life_est_b;
	u8  boot_mult;
	u8  rpmb_mult;
	u16 dev_version;
} mmc_ext_csd_t;

typedef struct _sd_scr
{
	u8 sda_vsn;
	u8 sda_spec3;
	u8 bus_widths;
	u8 cmds;
} sd_scr_t;

typedef struct _sd_ssr
{
	u8 bus_width;
	u8 speed_class;
	u8 uhs_grade;
	u8 video_class;
	u8 app_class;
	u32 protected_size;
} sd_ssr_t;

/*! SDMMC storage context. */
typedef struct _sdmmc_storage_t
{
	sdmmc_t *sdmmc;
	u32 rca;
	int has_sector_access;
	u32 sec_cnt;
	int is_low_voltage;
	u32 partition;
	u8  raw_cid[0x10];
	u8  raw_csd[0x10];
	u8  raw_scr[8];
	mmc_cid_t     cid;
	mmc_csd_t     csd;
	mmc_ext_csd_t ext_csd;
	sd_scr_t      scr;
	ddr_policy_t ddr;
	int initialized;
} sdmmc_storage_t;

extern sdmmc_accessor_t *_current_accessor;
extern bool sdmmc_memcpy_buf;

int sdmmc_storage_end(sdmmc_storage_t *storage);
int sdmmc_storage_read(sdmmc_storage_t *storage, u32 sector, u32 num_sectors, void *buf);
int sdmmc_storage_write(sdmmc_storage_t *storage, u32 sector, u32 num_sectors, void *buf);
#ifdef EMUMMC_DDR200_READ_AHEAD
bool sdmmc_read_ahead(sdmmc_storage_t *storage, Handle das, u32 sector, u32 blocks, void *out);
u64 sdmmc_read_ahead_dma(const void *buf, u32 blocks);
void sdmmc_read_ahead_invalidate(u32 sector, u32 blocks);
#endif
#if defined(EMUMMC_DDR200_READ_AHEAD) || defined(EMUMMC_DDR200_ADMA_PROBE)
void sdmmc_reset_private_dma(void);
#endif
int sdmmc_storage_init_mmc(sdmmc_storage_t *storage, sdmmc_t *sdmmc, u32 bus_width, u32 type);
int sdmmc_storage_set_mmc_partition(sdmmc_storage_t *storage, u32 partition);
int sdmmc_storage_init_sd(sdmmc_storage_t *storage, sdmmc_t *sdmmc, u32 bus_width, u32 type);
int sdmmc_storage_init_gc(sdmmc_storage_t *storage, sdmmc_t *sdmmc);
intptr_t sdmmc_calculate_dma_addr(sdmmc_accessor_t *_this, void *buf, unsigned int num_sectors);
int sdmmc_calculate_dma_index(sdmmc_accessor_t *_this, void *buf, unsigned int num_sectors);
int sdmmc_calculate_fitting_dma_index(sdmmc_accessor_t *_this, unsigned int num_sectors);
bool sdmmc_ddr200_handoff(sdmmc_storage_t *storage);
bool sdmmc_ddr200_prepare_suspend(sdmmc_storage_t *storage);
bool sdmmc_ddr200_profile_ready(const sdmmc_storage_t *storage);
bool sdmmc_ddr200_initial(sdmmc_storage_t *storage, const u8 *functions);
bool sdmmc_ddr200_before_io(sdmmc_storage_t *storage, u32 sector, bool writing);
bool sdmmc_ddr200_recover(sdmmc_storage_t *storage, u32 sector, bool degrade);
#ifdef EMUMMC_DDR200_ADMA_PROBE
bool sdmmc_ddr200_retry_io(sdmmc_storage_t *storage, u32 sector, u32 failed[4], bool exhausted);
#endif
void sdmmc_ddr200_count_io(sdmmc_storage_t *storage, u32 blocks, u64 elapsed);
int sdmmc_storage_transfer(sdmmc_storage_t *storage, u32 *done, u32 sector, u32 count, void *buf, u32 write);
bool sdmmc_storage_wait_ready(sdmmc_storage_t *storage, u64 deadline);
int sdmmc_sd_switch(sdmmc_storage_t *storage, u8 *status, u32 argument);
void sdmmc_sd_set_current_limit(sdmmc_storage_t *storage, u16 current_limit, u8 *status);

#ifdef EMUMMC_DDR200_MATRIX
bool sdmmc_ddr200_matrix_step(sdmmc_storage_t *storage);
void sdmmc_ddr200_matrix_skip(sdmmc_storage_t *storage);
#endif

#endif

#ifdef EMUMMC_DDR200_ADMA_PROBE
u64 sdmmc_adma_table(u64 dma, u32 bytes);
bool sdmmc_adma_remap(void);
#endif
#ifdef EMUMMC_DDR200_DLL_DIAG
void sdmmc_ddr200_matrix_restart(void);
#endif
