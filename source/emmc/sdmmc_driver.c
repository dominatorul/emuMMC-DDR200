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

#include <string.h>
#include "mmc.h"
#include "sd.h"
#include "sdmmc.h"
#include "../nx/cache.h"
#include "../power/max7762x.h"
#include "../soc/clock.h"
#include "../soc/gpio.h"
#include "../soc/pinmux.h"
#include "../soc/pmc.h"
#include "../soc/t210.h"
#include "../utils/fatal.h"
#include "../utils/types.h"
#include "../utils/util.h"
#include "../utils/log.h"
#include "../utils/ddr_trace.h"
#include "sdmmc_tuning.h"

#define DPRINTF(...)

static const u8 _trim_values_t210[4]    = {  2,  8,  3,  8 };
static const u8 _trim_values_t210b01[4] = { 14, 13, 15, 13 };

/*! SCMMC controller base addresses. */
static const u64 _sdmmc_bases[4] = {
	0x700B0000,
	0x700B0200,
	0x700B0400,
	0x700B0600,
};

int sdmmc_get_io_power(sdmmc_t *sdmmc)
{
	u32 p = sdmmc->regs->pwrcon;
	if (!(p & SDHCI_POWER_ON))
		return SDMMC_POWER_OFF;
	if ((p & ~SDHCI_POWER_MASK) == SDHCI_POWER_180)
		return SDMMC_POWER_1_8;
	if ((p & ~SDHCI_POWER_MASK) == SDHCI_POWER_330)
		return SDMMC_POWER_3_3;
	return -1;
}

static int _sdmmc_set_io_power(sdmmc_t *sdmmc, u32 power)
{
	switch (power)
	{
	case SDMMC_POWER_OFF:
		sdmmc->regs->pwrcon &= ~SDHCI_POWER_ON;
		break;

	case SDMMC_POWER_1_8:
		sdmmc->regs->pwrcon = SDHCI_POWER_180;
		break;

	case SDMMC_POWER_3_3:
		sdmmc->regs->pwrcon = SDHCI_POWER_330;
		break;

	default:
		return 0;
	}

	if (power != SDMMC_POWER_OFF)
		sdmmc->regs->pwrcon |= SDHCI_POWER_ON;

	return 1;
}

u32 sdmmc_get_bus_width(sdmmc_t *sdmmc)
{
	u32 h = sdmmc->regs->hostctl;
	if (h & SDHCI_CTRL_8BITBUS)
		return SDMMC_BUS_WIDTH_8;
	if (h & SDHCI_CTRL_4BITBUS)
		return SDMMC_BUS_WIDTH_4;
	return SDMMC_BUS_WIDTH_1;
}

void sdmmc_set_bus_width(sdmmc_t *sdmmc, u32 bus_width)
{
	u32 host_control = sdmmc->regs->hostctl & ~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);

	if (bus_width == SDMMC_BUS_WIDTH_1)
		sdmmc->regs->hostctl = host_control;
	else if (bus_width == SDMMC_BUS_WIDTH_4)
		sdmmc->regs->hostctl = host_control | SDHCI_CTRL_4BITBUS;
	else if (bus_width == SDMMC_BUS_WIDTH_8)
		sdmmc->regs->hostctl = host_control | SDHCI_CTRL_8BITBUS;
}

void sdmmc_save_tap_value(sdmmc_t *sdmmc)
{
	sdmmc->venclkctl_tap = sdmmc->regs->venclkctl >> 16;
	sdmmc->venclkctl_set = 1;
}

static int _sdmmc_config_tap_val(sdmmc_t *sdmmc, u32 type)
{
	const u32 dqs_trim_val = 0x28;
	const u32 tap_values_t210[] = { 4, 0, 3, 0 };

	u32 tap_val = 0;

	if (type == SDHCI_TIMING_MMC_HS400)
		sdmmc->regs->vencapover = (sdmmc->regs->vencapover & 0xFFFFC0FF) | (dqs_trim_val << 8);

	sdmmc->regs->ventunctl0 &= ~TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;

	if (type == SDHCI_TIMING_MMC_HS400)
	{
		if (!sdmmc->venclkctl_set)
			return 0;

		tap_val = sdmmc->venclkctl_tap;
	}
	else
		tap_val = sdmmc->t210b01 ? 11 : tap_values_t210[sdmmc->id];

	sdmmc->regs->venclkctl = (sdmmc->regs->venclkctl & 0xFF00FFFF) | (tap_val << 16);

	return 1;
}

static int _sdmmc_commit_changes(sdmmc_t *sdmmc)
{
	return sdmmc->regs->clkcon;
}

static void _sdmmc_pad_config_fallback(sdmmc_t *sdmmc, u32 power)
{
	_sdmmc_commit_changes(sdmmc);
	switch (sdmmc->id)
	{
	case SDMMC_1: // 33 Ohm 2X Driver.
		if (power == SDMMC_POWER_OFF)
			break;
		u32 sdmmc1_pad_cfg = APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) & 0xF8080FFF;
		if (sdmmc->t210b01)
			sdmmc1_pad_cfg |= (0x808 << 12); // Up:  8, Dn:  8. For 33 ohm.
		else if (power == SDMMC_POWER_1_8)
			sdmmc1_pad_cfg |= (0xB0F << 12); // Up: 11, Dn: 15. For 33 ohm.
		else if (power == SDMMC_POWER_3_3)
			sdmmc1_pad_cfg |= (0xC0C << 12); // Up: 12, Dn: 12. For 33 ohm.
		APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) = sdmmc1_pad_cfg;
		(void)APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL); // Commit write.
		break;

	case SDMMC_2:
		if (sdmmc->t210b01)
			APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL) = (APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL) & 0xF8080FFF) | 0xA0A000;
		else
			APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL) = (APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL) & 0xFFFFC003) | 0x1040; // PU:16, PD:16.
		(void)APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL);
		break;

	case SDMMC_4: // 50 Ohm 2X Driver. PU:16, PD:16, B01: PU:10, PD:10.
		APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL) =
			(APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL) & 0xFFFFC003) | (sdmmc->t210b01 ? 0xA28 : 0x1040);
		(void)APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL); // Commit write.
		break;
	}
}

static void _sdmmc_autocal_execute(sdmmc_t *sdmmc, u32 power)
{
	bool should_enable_sd_clock = false;
	if (sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN)
	{
		should_enable_sd_clock = true;
		sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
	}

	// Enable E_INPUT power.
	if (!(sdmmc->regs->sdmemcmppadctl & TEGRA_MMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_PWRD))
	{
		sdmmc->regs->sdmemcmppadctl |= TEGRA_MMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_PWRD;
		_sdmmc_commit_changes(sdmmc);
		usleep(1);
	}

	// Enable auto calibration and start auto configuration.
	sdmmc->regs->autocalcfg |= TEGRA_MMC_AUTOCALCFG_AUTO_CAL_ENABLE | TEGRA_MMC_AUTOCALCFG_AUTO_CAL_START;
	_sdmmc_commit_changes(sdmmc);
	usleep(2);

	u64 timeout = get_tmr_ms() + 10;
	while (sdmmc->regs->autocalsts & TEGRA_MMC_AUTOCALSTS_AUTO_CAL_ACTIVE)
	{
		if (get_tmr_ms() > timeout)
		{
			timeout = 0; // Set timeout to 0 if we timed out.
			break;
		}
	}

	sdmmc->autocal_sts = sdmmc->regs->autocalsts;
	sdmmc->autocal_fallback = !timeout;

	// In case auto calibration fails, we load suggested standard values.
	if (!timeout)
	{
		_sdmmc_pad_config_fallback(sdmmc, power);
		sdmmc->regs->autocalcfg &= ~TEGRA_MMC_AUTOCALCFG_AUTO_CAL_ENABLE;
	}

	// Disable E_INPUT to conserve power.
	sdmmc->regs->sdmemcmppadctl &= ~TEGRA_MMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_PWRD;

	if(should_enable_sd_clock)
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
}

static int _sdmmc_dll_cal_execute(sdmmc_t *sdmmc)
{
	int result = 1, should_disable_sd_clock = 0;

	if (!(sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN))
	{
		should_disable_sd_clock = 1;
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
	}

	sdmmc->regs->vendllcalcfg |= TEGRA_MMC_DLLCAL_CFG_EN_CALIBRATE;
	_sdmmc_commit_changes(sdmmc);

	u64 timeout = get_tmr_ms() + 5;
	while (sdmmc->regs->vendllcalcfg & TEGRA_MMC_DLLCAL_CFG_EN_CALIBRATE)
	{
		if (get_tmr_ms() > timeout)
		{
			result = 0;
			goto out;
		}
	}

	timeout = get_tmr_ms() + 10;
	while (sdmmc->regs->vendllcalcfgsts & TEGRA_MMC_DLLCAL_CFG_STATUS_DLL_ACTIVE)
	{
		if (get_tmr_ms() > timeout)
		{
			result = 0;
			goto out;
		}
	}

out:;
	if (should_disable_sd_clock)
		sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
	return result;
}

#ifdef EMUMMC_DDR200_DLL_DIAG
int sdmmc_ddr200_dll_probe(sdmmc_t *host)
{
    t210_sdmmc_t *regs = host->regs;
    if ((regs->hostctl2 & 7) != UHS_DDR50_BUS_SPEED)
        return 0;
    const u16 clock = regs->clkcon;
    const u32 config = regs->vendllcalcfg;
    regs->clkcon |= SDHCI_CLOCK_CARD_EN;
    regs->vendllcalcfg = config | TEGRA_MMC_DLLCAL_CFG_EN_CALIBRATE;
    _sdmmc_commit_changes(host);
    u64 deadline = get_tmr_us() + 10000;
    bool active = false;
    int result = -1;
    do {
        u32 status = regs->vendllcalcfgsts;
        active |= (status & TEGRA_MMC_DLLCAL_CFG_STATUS_DLL_ACTIVE) != 0;
        if (!(regs->vendllcalcfg & TEGRA_MMC_DLLCAL_CFG_EN_CALIBRATE) &&
            !(status & TEGRA_MMC_DLLCAL_CFG_STATUS_DLL_ACTIVE)) {
            result = active ? 1 : 0;
            break;
        }
    } while (get_tmr_us() < deadline);
    regs->vendllcalcfg = config;
    regs->clkcon = clock;
    _sdmmc_commit_changes(host);
    return result;
}
#endif

static u64 _sdmmc_limit_ms(const sdmmc_t *sdmmc, u64 timeout)
{
	return sdmmc->deadline_us ? MIN(timeout, (sdmmc->deadline_us + 999) / 1000) : timeout;
}

static void _sdmmc_reset(sdmmc_t *sdmmc)
{
	sdmmc->regs->swrst |= SDHCI_RESET_CMD | SDHCI_RESET_DATA;
	_sdmmc_commit_changes(sdmmc);
	u64 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
	while ((sdmmc->regs->swrst & (SDHCI_RESET_CMD | SDHCI_RESET_DATA)) && get_tmr_ms() < timeout)
		;
}

bool sdmmc_reset_transfer(sdmmc_t *sdmmc)
{
	_sdmmc_reset(sdmmc);
	return !(sdmmc->regs->swrst & (SDHCI_RESET_CMD | SDHCI_RESET_DATA));
}

static void _sdmmc_reset_all(sdmmc_t *sdmmc)
{
	sdmmc->regs->swrst |= SDHCI_RESET_ALL;
	_sdmmc_commit_changes(sdmmc);
	u32 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);//100ms
	while ((sdmmc->regs->swrst & SDHCI_RESET_ALL) && get_tmr_ms() < timeout)
		;
}

static int _sdmmc_setup_clock(sdmmc_t *sdmmc, u32 type, u32 source_override)
{
	// Disable the SD clock if it was enabled, and reenable it later.
	bool should_enable_sd_clock = false;
	if (sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN)
	{
		should_enable_sd_clock = true;
		sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
	}

	_sdmmc_config_tap_val(sdmmc, type);

	_sdmmc_reset(sdmmc);

	switch (type)
	{
	case SDHCI_TIMING_MMC_ID:
	case SDHCI_TIMING_MMC_LS26:
	case SDHCI_TIMING_SD_ID:
	case SDHCI_TIMING_SD_DS12:
		sdmmc->regs->hostctl  &= ~SDHCI_CTRL_HISPD;
		sdmmc->regs->hostctl2 &= ~SDHCI_CTRL_VDD_180;
		break;

	case SDHCI_TIMING_MMC_HS52:
	case SDHCI_TIMING_SD_HS25:
		sdmmc->regs->hostctl  |= SDHCI_CTRL_HISPD;
		sdmmc->regs->hostctl2 &= ~SDHCI_CTRL_VDD_180;
		break;

	case SDHCI_TIMING_MMC_HS200:
	case SDHCI_TIMING_UHS_SDR50: // T210 Errata for SDR50, the host must be set to SDR104.
	case SDHCI_TIMING_UHS_SDR104:
	case SDHCI_TIMING_UHS_SDR82:
	case SDHCI_TIMING_UHS_DDR50:
	case SDHCI_TIMING_MMC_HS102:
		sdmmc->regs->hostctl2  = (sdmmc->regs->hostctl2 & SDHCI_CTRL_UHS_MASK) | UHS_SDR104_BUS_SPEED;
		sdmmc->regs->hostctl2 |= SDHCI_CTRL_VDD_180;
		break;

	case SDHCI_TIMING_MMC_HS400:
		// Non standard.
		sdmmc->regs->hostctl2  = (sdmmc->regs->hostctl2 & SDHCI_CTRL_UHS_MASK) | HS400_BUS_SPEED;
		sdmmc->regs->hostctl2 |= SDHCI_CTRL_VDD_180;
		break;

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
	case SDHCI_TIMING_UHS_DDR200:
		sdmmc->regs->hostctl2  = (sdmmc->regs->hostctl2 & SDHCI_CTRL_UHS_MASK) | UHS_DDR50_BUS_SPEED;
		sdmmc->regs->hostctl2 |= SDHCI_CTRL_VDD_180;
		break;
#endif

	case SDHCI_TIMING_UHS_SDR25:
		sdmmc->regs->hostctl2  = (sdmmc->regs->hostctl2 & SDHCI_CTRL_UHS_MASK) | UHS_SDR25_BUS_SPEED;
		sdmmc->regs->hostctl2 |= SDHCI_CTRL_VDD_180;
		break;

	case SDHCI_TIMING_UHS_SDR12:
		sdmmc->regs->hostctl2  = (sdmmc->regs->hostctl2 & SDHCI_CTRL_UHS_MASK) | UHS_SDR12_BUS_SPEED;
		sdmmc->regs->hostctl2 |= SDHCI_CTRL_VDD_180;
		break;
	}

	_sdmmc_commit_changes(sdmmc);

	u32 clock;
	u16 divisor;
	clock_sdmmc_get_card_clock_div(&clock, &divisor, type);
	if (source_override)
		clock = source_override;
	if (!clock_sdmmc_config_clock_source(&clock, sdmmc->id, clock))
		return 0;
	sdmmc->divisor = (clock + divisor - 1) / divisor;

	//if divisor != 1 && divisor << 31 -> error

	u16 div = divisor >> 1;
	divisor = 0;
	if (div > 0xFF)
		divisor = div >> SDHCI_DIVIDER_SHIFT;

	sdmmc->regs->clkcon = (sdmmc->regs->clkcon & ~(SDHCI_DIV_MASK | SDHCI_DIV_HI_MASK))
		| (div << SDHCI_DIVIDER_SHIFT) | (divisor << SDHCI_DIVIDER_HI_SHIFT);

	// Enable the SD clock again.
	if (should_enable_sd_clock)
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;

	if (type == SDHCI_TIMING_MMC_HS400)
		return _sdmmc_dll_cal_execute(sdmmc);
	return 1;
}

int sdmmc_setup_clock(sdmmc_t *sdmmc, u32 type)
{
	return _sdmmc_setup_clock(sdmmc, type, 0);
}

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
int sdmmc_setup_ddr200_clock(sdmmc_t *sdmmc, bool reduced)
{
	return _sdmmc_setup_clock(sdmmc, SDHCI_TIMING_UHS_DDR200,
		reduced ? SDMMC_DDR200_RECOVERY_SRC_KHZ : 0);
}
#endif

static void _sdmmc_card_clock_enable(sdmmc_t *sdmmc)
{
	// Recalibrate conditionally.
	if (sdmmc->manual_cal && !sdmmc->powersave_enabled)
		_sdmmc_autocal_execute(sdmmc, sdmmc_get_io_power(sdmmc));

	if (!sdmmc->powersave_enabled)
	{
		if (!(sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN))
			sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
	}
	sdmmc->card_clock_enabled = 1;
}

static void _sdmmc_sd_clock_disable(sdmmc_t *sdmmc)
{
	sdmmc->card_clock_enabled = 0;
	sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
}

void sdmmc_card_clock_powersave(sdmmc_t *sdmmc, int powersave_enable)
{
	// Recalibrate periodically for SDMMC1.
	if (sdmmc->manual_cal && !powersave_enable && sdmmc->card_clock_enabled)
		_sdmmc_autocal_execute(sdmmc, sdmmc_get_io_power(sdmmc));

	sdmmc->powersave_enabled = powersave_enable;
	if (powersave_enable)
	{
		if (sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN)
			sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
		return;
	}

	if (sdmmc->card_clock_enabled)
		if (!(sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN))
			sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
}

static int _sdmmc_cache_rsp(sdmmc_t *sdmmc, u32 *rsp, u32 size, u32 type)
{
	switch (type)
	{
	case SDMMC_RSP_TYPE_1:
	case SDMMC_RSP_TYPE_3:
	case SDMMC_RSP_TYPE_4:
	case SDMMC_RSP_TYPE_5:
		if (size < 4)
			return 0;
		rsp[0] = sdmmc->regs->rspreg0;
		break;

	case SDMMC_RSP_TYPE_2:
		if (size < 0x10)
			return 0;
		// CRC is stripped, so shifting is needed.
		u32 tempreg;
		for (int i = 0; i < 4; i++)
		{
			switch(i)
			{
			case 0:
				tempreg = sdmmc->regs->rspreg3;
				break;
			case 1:
				tempreg = sdmmc->regs->rspreg2;
				break;
			case 2:
				tempreg = sdmmc->regs->rspreg1;
				break;
			case 3:
				tempreg = sdmmc->regs->rspreg0;
				break;
			}
			rsp[i] = tempreg << 8;

			if (i != 0)
				rsp[i - 1] |= (tempreg >> 24) & 0xFF;
		}
		break;

	default:
		return 0;
	}

	return 1;
}

int sdmmc_get_rsp(sdmmc_t *sdmmc, u32 *rsp, u32 size, u32 type)
{
	if (!rsp || sdmmc->expected_rsp_type != type)
		return 0;

	switch (type)
	{
	case SDMMC_RSP_TYPE_1:
	case SDMMC_RSP_TYPE_3:
	case SDMMC_RSP_TYPE_4:
	case SDMMC_RSP_TYPE_5:
		if (size < 4)
			return 0;
		rsp[0] = sdmmc->rsp[0];
		break;

	case SDMMC_RSP_TYPE_2:
		if (size < 0x10)
			return 0;
		rsp[0] = sdmmc->rsp[0];
		rsp[1] = sdmmc->rsp[1];
		rsp[2] = sdmmc->rsp[2];
		rsp[3] = sdmmc->rsp[3];
		break;

	default:
		return 0;
	}

	return 1;
}

static int _sdmmc_wait_cmd_data_inhibit(sdmmc_t *sdmmc, bool wait_dat)
{
	_sdmmc_commit_changes(sdmmc);

	u64 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
	while(sdmmc->regs->prnsts & SDHCI_CMD_INHIBIT)
		if (get_tmr_ms() > timeout)
		{
			_sdmmc_reset(sdmmc);
			return 0;
		}

	if (wait_dat)
	{
		timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
		while (sdmmc->regs->prnsts & SDHCI_DATA_INHIBIT)
			if (get_tmr_ms() > timeout)
			{
				_sdmmc_reset(sdmmc);
				return 0;
			}
	}

	return 1;
}

#ifdef EMUMMC_DDR200_ADMA_PROBE
static void _sdmmc_capture_dma_error(sdmmc_t *sdmmc)
{
	if (!sdmmc->dma_progress.blocks)
		return;
	sdmmc->dma_progress.remaining = sdmmc->regs->blkcnt;
	sdmmc->dma_progress.adma_error = sdmmc->dma_progress.mode == SdmmcDmaAdma2 ? sdmmc->regs->admaerr : 0;
	sdmmc->dma_progress.sampled = true;
}
#endif

static int _sdmmc_wait_card_busy(sdmmc_t *sdmmc)
{
	_sdmmc_commit_changes(sdmmc);

	u64 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
	while (!(sdmmc->regs->prnsts & SDHCI_DATA_0_LVL_MASK))
		if (get_tmr_ms() > timeout)
		{
#ifdef EMUMMC_DDR200_ADMA_PROBE
			_sdmmc_capture_dma_error(sdmmc);
#endif
			_sdmmc_reset(sdmmc);
			return 0;
		}

	return 1;
}

static int _sdmmc_setup_read_small_block(sdmmc_t *sdmmc)
{
	switch (sdmmc_get_bus_width(sdmmc))
	{
	case SDMMC_BUS_WIDTH_1:
		return 0;

	case SDMMC_BUS_WIDTH_4:
		sdmmc->regs->blksize = 64;
		break;

	case SDMMC_BUS_WIDTH_8:
		sdmmc->regs->blksize = 128;
		break;
	}

	sdmmc->regs->blkcnt = 1;
	sdmmc->regs->trnmod = SDHCI_TRNS_READ;

	return 1;
}

static int _sdmmc_send_cmd(sdmmc_t *sdmmc, sdmmc_cmd_t *cmd, bool is_data_present)
{
	u16 cmdflags = 0;

	switch (cmd->rsp_type)
	{
	case SDMMC_RSP_TYPE_0:
		break;

	case SDMMC_RSP_TYPE_1:
	case SDMMC_RSP_TYPE_4:
	case SDMMC_RSP_TYPE_5:
		if (cmd->check_busy)
			cmdflags = SDHCI_CMD_RESP_LEN48_BUSY | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;
		else
			cmdflags = SDHCI_CMD_RESP_LEN48 | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;
		break;

	case SDMMC_RSP_TYPE_2:
		cmdflags = SDHCI_CMD_RESP_LEN136 | SDHCI_CMD_CRC;
		break;

	case SDMMC_RSP_TYPE_3:
		cmdflags = SDHCI_CMD_RESP_LEN48;
		break;

	default:
		return 0;
	}

	if (cmd->cmd == MMC_STOP_TRANSMISSION)
		cmdflags |= SDHCI_CMD_ABORTCMD;
	if (is_data_present)
		cmdflags |= SDHCI_CMD_DATA;
#ifdef EMUMMC_DDR200_COUNTED_READS
	else
		sdmmc->regs->trnmod &= ~(SDHCI_TRNS_AUTO_CMD12 | SDHCI_TRNS_AUTO_CMD23);
#endif
	sdmmc->regs->argument = cmd->arg;
	sdmmc->regs->cmdreg = (cmd->cmd << 8) | cmdflags;

	return 1;
}

static void _sdmmc_send_tuning_cmd(sdmmc_t *sdmmc, u32 cmd)
{
	sdmmc_cmd_t cmdbuf;
	cmdbuf.cmd = cmd;
	cmdbuf.arg = 0;
	cmdbuf.rsp_type = SDMMC_RSP_TYPE_1;
	cmdbuf.check_busy = 0;
	_sdmmc_send_cmd(sdmmc, &cmdbuf, true);
}

static int _sdmmc_tuning_execute_once(sdmmc_t *sdmmc, u32 cmd, u32 tap)
{
	if (sdmmc->powersave_enabled)
		return 0;
	if (!_sdmmc_wait_cmd_data_inhibit(sdmmc, true))
		return 0;

	_sdmmc_setup_read_small_block(sdmmc);

	sdmmc->regs->norintstsen |= SDHCI_INT_DATA_AVAIL;
	sdmmc->regs->norintsts = sdmmc->regs->norintsts;
	sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
	if (tap != SDMMC_HW_TAP_TUNING)
	{
		sdmmc->regs->ventunctl0 &= ~TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
		sdmmc->regs->venclkctl   = (sdmmc->regs->venclkctl & 0xFF00FFFF) | (tap << 16);
		sdmmc->regs->ventunctl0 |=  TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
	}
#else
	(void)tap;
#endif

	_sdmmc_send_tuning_cmd(sdmmc, cmd);
	_sdmmc_commit_changes(sdmmc);
	usleep(1);

	_sdmmc_reset(sdmmc);

	sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
	_sdmmc_commit_changes(sdmmc);

	u64 timeout = get_tmr_us() + 5000;
	if (sdmmc->deadline_us)
		timeout = MIN(timeout, sdmmc->deadline_us);
	while (get_tmr_us() < timeout)
	{
		if (sdmmc->regs->norintsts & SDHCI_INT_DATA_AVAIL)
		{
			sdmmc->regs->norintsts = SDHCI_INT_DATA_AVAIL;
			sdmmc->regs->norintstsen &= ~SDHCI_INT_DATA_AVAIL;
			_sdmmc_commit_changes(sdmmc);
			usleep((1000 * 8 + sdmmc->divisor - 1) / sdmmc->divisor);
			return 1;
		}
	}

	_sdmmc_reset(sdmmc);

	sdmmc->regs->norintstsen &= ~SDHCI_INT_DATA_AVAIL;
	_sdmmc_commit_changes(sdmmc);
	usleep((1000 * 8 + sdmmc->divisor - 1) / sdmmc->divisor);

	return 0;
}

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT

typedef struct _sdmmc_manual_tuning_t
{
	u32 result[8];
	u32 raw[4];
	u32 num_iter;
	u32 iter_done;
} sdmmc_manual_tuning_t;

#define SDMMC_TUNING_LOG_MAX 16

static u32 _sdmmc_tuning_log_id = 0;

static void _sdmmc_manual_tuning_log(sdmmc_t *sdmmc, sdmmc_manual_tuning_t *tuning, u32 best_tap, u32 best_size, int pass)
{
	u32 tap_start = SDMMC_INVALID_TAP;
	u32 win_size  = 0;

	if (_sdmmc_tuning_log_id >= SDMMC_TUNING_LOG_MAX)
		return;

	Log("[%u] iter %u/%u map %08X %08X %08X %08X\n", _sdmmc_tuning_log_id, tuning->iter_done, tuning->num_iter, tuning->result[0], tuning->result[1], tuning->result[2], tuning->result[3]);

	for (u32 i = 0; i < tuning->iter_done; i++)
	{
		bool iter_end = i == tuning->iter_done - 1;
		u32 stable = tuning->result[i / 32] & (1u << (i % 32));

		if (stable)
		{
			if (tap_start == SDMMC_INVALID_TAP)
				tap_start = i;

			win_size++;
		}
		if ((!stable || iter_end) && tap_start != SDMMC_INVALID_TAP)
		{
			u32 tap_end = stable ? i : i - 1;

			Log("[%u] win %u-%u sz %u mid %u\n", _sdmmc_tuning_log_id, tap_start, tap_end, win_size, (tap_start + tap_end) / 2);

			tap_start = SDMMC_INVALID_TAP;
			win_size  = 0;
		}
	}

	Log("[%u] best tap %u sz %u min %u %s\n", _sdmmc_tuning_log_id, best_tap, best_size, SDMMC_SAMPLE_WIN_SIZE_MIN, pass ? "OK" : "FAIL");

	Log("[%u] clk %u vclk %08X tun0 %08X acal %08X cmp %08X hc2 %04X\n", _sdmmc_tuning_log_id, sdmmc->divisor, sdmmc->regs->venclkctl, sdmmc->regs->ventunctl0, sdmmc->regs->autocalcfg, sdmmc->regs->sdmemcmppadctl, sdmmc->regs->hostctl2);

	u32 pad_cfg = APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL);
	u32 code_mask = (sdmmc->t210b01 || sdmmc->id != SDMMC_1) ? 0x1F : 0x7F;
	u32 comp = sdmmc->autocal_sts & code_mask;

	Log("[%u] pad %08X up %u dn %u slw %u sts %08X comp %u%s fb %d\n", _sdmmc_tuning_log_id, pad_cfg, (pad_cfg >> 20) & 0x7F, (pad_cfg >> 12) & 0x7F, (pad_cfg >> 28) & 0xF,sdmmc->autocal_sts, comp,
        !comp ? " SHORT" : (comp == code_mask ? " OPEN" : ""), sdmmc->autocal_fallback);

	Log("[%u] pmx %04X %04X %04X lpbk %u iot %08X iosp %08X msc %08X hc %02X ck %04X pw %02X\n", _sdmmc_tuning_log_id, (u32)PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK), (u32)PINMUX_AUX(PINMUX_AUX_SDMMC1_CMD), (u32)PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT0),
		(u32)APB_MISC(APB_MISC_GP_SDMMC1_CLK_LPBK_CONTROL), sdmmc->regs->veniotrimctl, sdmmc->regs->iospare, sdmmc->regs->venmiscctl, (u32)sdmmc->regs->hostctl, (u32)sdmmc->regs->clkcon, (u32)sdmmc->regs->pwrcon);

	_sdmmc_tuning_log_id++;
}

void sdmmc_set_ddr200_tap(sdmmc_t *sdmmc, u32 tap)
{
	u16 clock = sdmmc->regs->clkcon;
	sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
	sdmmc->regs->ventunctl0 &= ~TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
	sdmmc->regs->venclkctl = (sdmmc->regs->venclkctl & 0xFF00FFFF) | (tap << 16);
	sdmmc->regs->ventunctl0 |= TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
	sdmmc->regs->clkcon = clock;
	_sdmmc_commit_changes(sdmmc);
}

static int _sdmmc_manual_tuning_set_tap(sdmmc_t *sdmmc, sdmmc_manual_tuning_t *tuning)
{
	u32 best_size;
	u32 best_tap = sdmmc_tuning_select_window(tuning->result, tuning->iter_done, &best_size);
	int pass = best_size >= SDMMC_SAMPLE_WIN_SIZE_MIN;
	u32 selected_tap = best_tap;
#ifdef EMUMMC_FORCE_DDR200
	/* After Horizon takes over, this card lost the high side of its full-rate
	   window. Select an interior tap on the low side of the measured run. */
	bool prefer_lower = sdmmc->ddr200_use_lower_tap &&
		sdmmc->divisor != SDMMC_DDR200_RECOVERY_CARD_KHZ;
	if (pass && prefer_lower)
		selected_tap = sdmmc_tuning_lower_quarter(best_tap, best_size);
#endif
	if (pass) {
		sdmmc_set_ddr200_tap(sdmmc, selected_tap);
		selected_tap = (sdmmc->regs->venclkctl >> 16) & 0xFF;
	}
	ddr_trace_tuning(sdmmc, tuning->raw, tuning->result, tuning->iter_done, best_tap,
		best_size, pass ? selected_tap : SDMMC_INVALID_TAP);

	_sdmmc_manual_tuning_log(sdmmc, tuning, best_tap, best_size, pass);

	sdmmc->ddr200_retry_count = 0;
	if (!pass)
		return 0;

#ifdef EMUMMC_FORCE_DDR200
	/* On a read CRC error, try a separate passing window first, then two
	   interior taps from the winning window. Keep the eight-tap minimum. */
	u32 start = sdmmc_tuning_window_start(best_tap, best_size);
	for (u32 tap = start; tap < start + best_size; ++tap)
		tuning->result[tap / 32] &= ~(1u << (tap % 32));
	u32 other_size;
	u32 other_tap = sdmmc_tuning_select_window(tuning->result, tuning->iter_done, &other_size);
	if (other_size >= SDMMC_SAMPLE_WIN_SIZE_MIN)
		sdmmc->ddr200_retry_taps[sdmmc->ddr200_retry_count++] = other_tap;
	sdmmc->ddr200_retry_taps[sdmmc->ddr200_retry_count++] =
		prefer_lower ? best_tap : sdmmc_tuning_lower_quarter(best_tap, best_size);
	sdmmc->ddr200_retry_taps[sdmmc->ddr200_retry_count++] = start + best_size - 1 - best_size / 4;
#endif
	return 1;
}

u32 sdmmc_scan_ddr200(sdmmc_t *sdmmc, u32 first, u32 end, u32 raw[4], u32 complete[4])
{
	if (sdmmc->powersave_enabled || first >= end || end > 128)
		return 0;

	u32 saved_tap = (sdmmc->regs->venclkctl >> 16) & 0xFF;
	u16 saved_tuned = sdmmc->regs->hostctl2 & SDHCI_CTRL_TUNED_CLK;
	u16 saved_clock = sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN;
	sdmmc->regs->ventunctl1 = 0;
	sdmmc->regs->ventunctl0 = (sdmmc->regs->ventunctl0 & 0xFFFF1FFF) | (2 << 13);
	sdmmc->regs->ventunctl0 = (sdmmc->regs->ventunctl0 & 0xFFFFE03F) | (1 << 6);
	sdmmc->regs->ventunctl0 |= TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
	sdmmc->regs->hostctl2 &= ~(SDHCI_CTRL_EXEC_TUNING | SDHCI_CTRL_TUNED_CLK);
	sdmmc->regs->hostctl2 |= SDHCI_CTRL_EXEC_TUNING;

	u32 tap = first;
	for (; tap < end; ++tap) {
		if (sdmmc->deadline_us && get_tmr_us() >= sdmmc->deadline_us)
			break;
		int finished = _sdmmc_tuning_execute_once(sdmmc, MMC_SEND_TUNING_BLOCK, tap);
		if (sdmmc->regs->hostctl2 & SDHCI_CTRL_TUNED_CLK) {
			raw[tap / 32] |= 1u << (tap % 32);
			if (finished)
				complete[tap / 32] |= 1u << (tap % 32);
		}
		if (!(sdmmc->regs->hostctl2 & SDHCI_CTRL_EXEC_TUNING)) {
			++tap;
			break;
		}
		if (sdmmc->ddr200_scan_gated) {
			sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
			_sdmmc_commit_changes(sdmmc);
			usleep(1);
		}
	}
	sdmmc->regs->hostctl2 = (sdmmc->regs->hostctl2 &
		~(SDHCI_CTRL_EXEC_TUNING | SDHCI_CTRL_TUNED_CLK)) | saved_tuned;
	sdmmc_set_ddr200_tap(sdmmc, saved_tap);
	sdmmc->regs->clkcon = (sdmmc->regs->clkcon & ~SDHCI_CLOCK_CARD_EN) | saved_clock;
	return tap - first;
}

static int _sdmmc_tuning_execute_ddr200(sdmmc_t *sdmmc, u32 cmd)
{
	static bool first = true;
	if (cmd != MMC_SEND_TUNING_BLOCK)
		return 0;
	ddr_trace_stage(DdrSweep, sdmmc->divisor);
	if (first) {
		ddr_trace_host(DdrInitialTune, sdmmc);
		first = false;
	}
	ddr_trace_host(DdrLatestTune, sdmmc);
	sdmmc_manual_tuning_t tuning = {.num_iter = 128};
	tuning.iter_done = sdmmc_scan_ddr200(sdmmc, 0, tuning.num_iter, tuning.raw, tuning.result);
	return _sdmmc_manual_tuning_set_tap(sdmmc, &tuning);
}

#endif

int sdmmc_tuning_execute(sdmmc_t *sdmmc, u32 type, u32 cmd)
{
	u32 max = 0, flag = 0;

	if (sdmmc->powersave_enabled)
		return 0;

	switch (type)
	{
#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
	case SDHCI_TIMING_UHS_DDR200:
		return _sdmmc_tuning_execute_ddr200(sdmmc, cmd);
#endif

	case SDHCI_TIMING_MMC_HS200:
	case SDHCI_TIMING_MMC_HS400:
	case SDHCI_TIMING_UHS_SDR104:
	case SDHCI_TIMING_UHS_SDR82:
		max = 128;
		flag = (2 << 13); // 128 iterations.
		break;

	case SDHCI_TIMING_UHS_SDR50:
	case SDHCI_TIMING_UHS_DDR50:
	case SDHCI_TIMING_MMC_HS102:
		max = 256;
		flag = (4 << 13); // 256 iterations.
		break;

	case SDHCI_TIMING_UHS_SDR12:
	case SDHCI_TIMING_UHS_SDR25:
		return 1;

	default:
		return 0;
	}

	sdmmc->regs->ventunctl1 = 0; // step_size 1.

	sdmmc->regs->ventunctl0 = (sdmmc->regs->ventunctl0 & 0xFFFF1FFF) | flag; // Tries.
	sdmmc->regs->ventunctl0 = (sdmmc->regs->ventunctl0 & 0xFFFFE03F) | (1 << 6); // 1x Multiplier.
	sdmmc->regs->ventunctl0 |= TEGRA_MMC_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
	sdmmc->regs->hostctl2   |= SDHCI_CTRL_EXEC_TUNING;

	for (u32 i = 0; i < max; i++)
	{
		_sdmmc_tuning_execute_once(sdmmc, cmd, SDMMC_HW_TAP_TUNING);
		if (!(sdmmc->regs->hostctl2 & SDHCI_CTRL_EXEC_TUNING))
			break;
	}

	if (sdmmc->regs->hostctl2 & SDHCI_CTRL_TUNED_CLK)
		return 1;

	return 0;
}

static int _sdmmc_enable_internal_clock(sdmmc_t *sdmmc)
{
	//Enable internal clock and wait till it is stable.
	sdmmc->regs->clkcon |= SDHCI_CLOCK_INT_EN;
	_sdmmc_commit_changes(sdmmc);
	u64 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
	while (!(sdmmc->regs->clkcon & SDHCI_CLOCK_INT_STABLE))
	{
		if (get_tmr_ms() > timeout)
			return 0;
	}

	sdmmc->regs->hostctl2 &= ~SDHCI_CTRL_PRESET_VAL_EN;
	sdmmc->regs->clkcon   &= ~SDHCI_PROG_CLOCK_MODE;
	sdmmc->regs->hostctl2 |= SDHCI_HOST_VERSION_4_EN;

	if (!(sdmmc->regs->capareg & SDHCI_CAN_64BIT))
		return 0;

	sdmmc->regs->hostctl2 |= SDHCI_ADDRESSING_64BIT_EN;
	sdmmc->regs->hostctl &= ~SDHCI_CTRL_DMA_MASK;
	sdmmc->regs->timeoutcon = (sdmmc->regs->timeoutcon & 0xF0) | 0xE;

	return 1;
}

static int _sdmmc_autocal_config_offset(sdmmc_t *sdmmc, u32 power)
{
	u32 off_pd = 0;
	u32 off_pu = 0;

	switch (sdmmc->id)
	{
	case SDMMC_2:
	case SDMMC_4:
		if (power != SDMMC_POWER_1_8)
			return 0;
		off_pd = 5;
		off_pu = 5;
		break;

	case SDMMC_1:
		if (power == SDMMC_POWER_1_8)
		{
			if (!sdmmc->t210b01)
			{
				off_pd = 123;
				off_pu = 123;
			}
			else
			{
				off_pd = 6;
				off_pu = 6;
			}
		}
		else if (power == SDMMC_POWER_3_3)
		{
			if (!sdmmc->t210b01)
			{
				off_pd = 125;
				off_pu = 0;
			}
		}
		else
			return 0;
		break;
	}

	sdmmc->regs->autocalcfg = (sdmmc->regs->autocalcfg & 0xFFFF8080) | (off_pd << 8) | off_pu;
	return 1;
}

static void _sdmmc_enable_interrupts(sdmmc_t *sdmmc)
{
	sdmmc->regs->norintstsen |= SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE;
	sdmmc->regs->errintstsen |= SDHCI_ERR_INT_ALL_EXCEPT_ADMA_BUSPWR;
#ifdef EMUMMC_DDR200_ADMA_PROBE
	if (sdmmc->dma_progress.mode == SdmmcDmaAdma2)
		sdmmc->regs->errintstsen |= SDHCI_ERR_INT_ADMA_ERROR;
#endif
	sdmmc->regs->norintsts = sdmmc->regs->norintsts;
	sdmmc->regs->errintsts = sdmmc->regs->errintsts;
}

static void _sdmmc_mask_interrupts(sdmmc_t *sdmmc)
{
	sdmmc->regs->errintstsen &= ~SDHCI_ERR_INT_ALL_EXCEPT_ADMA_BUSPWR;
	sdmmc->regs->norintstsen &= ~(SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE);
}

static int _sdmmc_check_mask_interrupt(sdmmc_t *sdmmc, u16 *pout, u16 mask)
{
	u16 norintsts = sdmmc->regs->norintsts;

	if (pout)
		*pout = norintsts;

	// Check for error interrupt.
	if (norintsts & SDHCI_INT_ERROR)
	{
		u16 errintsts = sdmmc->regs->errintsts;
		sdmmc->command_irq = ((u32)errintsts << 16) | norintsts;
		if (errintsts & SDHCI_ERR_INT_AUTO_CMD_ERR)
			sdmmc->command_auto_stop_error = sdmmc->regs->acmd12errsts;
		sdmmc->regs->errintsts = errintsts;
		return SDMMC_MASKINT_ERROR;
	}
	else if (norintsts & mask)
	{
		sdmmc->regs->norintsts = norintsts & mask;
		return SDMMC_MASKINT_MASKED;
	}

	return SDMMC_MASKINT_NOERROR;
}

static int _sdmmc_wait_response(sdmmc_t *sdmmc)
{
	_sdmmc_commit_changes(sdmmc);

	u64 timeout = _sdmmc_limit_ms(sdmmc, get_tmr_ms() + 2000);
	while (true)
	{
		int result = _sdmmc_check_mask_interrupt(sdmmc, NULL, SDHCI_INT_RESPONSE);
		if (result == SDMMC_MASKINT_MASKED)
			break;
		if (result != SDMMC_MASKINT_NOERROR || get_tmr_ms() > timeout)
		{
#ifdef EMUMMC_DDR200_ADMA_PROBE
			_sdmmc_capture_dma_error(sdmmc);
#endif
			_sdmmc_reset(sdmmc);
			return 0;
		}
	}

	return 1;
}

static int _sdmmc_stop_transmission_inner(sdmmc_t *sdmmc, u32 *rsp)
{
	sdmmc_cmd_t cmd;

	if (!_sdmmc_wait_cmd_data_inhibit(sdmmc, false))
		return 0;

	_sdmmc_enable_interrupts(sdmmc);

	cmd.cmd = MMC_STOP_TRANSMISSION;
	cmd.arg = 0;
	cmd.rsp_type = SDMMC_RSP_TYPE_1;
	cmd.check_busy = 1;

	_sdmmc_send_cmd(sdmmc, &cmd, false);

	int result = _sdmmc_wait_response(sdmmc);
	_sdmmc_mask_interrupts(sdmmc);

	if (!result)
		return 0;

	_sdmmc_cache_rsp(sdmmc, rsp, 4, SDMMC_RSP_TYPE_1);

	return _sdmmc_wait_card_busy(sdmmc);
}

int sdmmc_stop_transmission(sdmmc_t *sdmmc, u32 *rsp)
{
	if (!sdmmc->card_clock_enabled)
		return 0;

	// Recalibrate periodically for SDMMC1.
	if (sdmmc->manual_cal && sdmmc->powersave_enabled)
		_sdmmc_autocal_execute(sdmmc, sdmmc_get_io_power(sdmmc));

	bool should_disable_sd_clock = false;
	if (!(sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN))
	{
		should_disable_sd_clock = true;
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
		_sdmmc_commit_changes(sdmmc);
		usleep((8000 + sdmmc->divisor - 1) / sdmmc->divisor);
	}

	int result = _sdmmc_stop_transmission_inner(sdmmc, rsp);
	usleep((8000 + sdmmc->divisor - 1) / sdmmc->divisor);

	if (should_disable_sd_clock)
		sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;

	return result;
}

static int _sdmmc_config_dma(sdmmc_t *sdmmc, u32 *blkcnt_out, sdmmc_req_t *req)
{
	if (!req->blksize || !req->num_sectors || !req->buf ||
	    !_current_accessor || !_current_accessor->parent)
		return 0;

	u32 blkcnt = req->num_sectors;
	if (blkcnt >= 0xFFFF)
		blkcnt = 0xFFFF;
#ifdef EMUMMC_DDR200_ADMA_PROBE
	bool adma = sdmmc->id == SDMMC_1 && !sdmmc->handoff_sdma && !sdmmc->wake_sdma &&
		(sdmmc->regs->capareg & SDHCI_CAN_ADMA2);
#ifdef EMUMMC_DDR200_ADMA_READAHEAD_ONLY
	adma &= !req->is_write && req->blksize == 512 &&
		sdmmc_read_ahead_dma(req->buf, blkcnt) != 0;
#endif
	if (adma) {
		sdmmc->dma_progress.mode = SdmmcDmaAdma2;
		blkcnt = MIN(blkcnt, SdmmcAdmaBytes / req->blksize);
		if (!blkcnt || (!req->is_multi_block && blkcnt < req->num_sectors))
			return 0;
	}
#endif

	u64 dma_address = 0;
#ifdef EMUMMC_DDR200_READ_AHEAD
	if (sdmmc->id == SDMMC_1 && !req->is_write && req->blksize == 512)
		dma_address = sdmmc_read_ahead_dma(req->buf, blkcnt);
#endif
	if (dma_address)
		sdmmc_memcpy_buf = false;
	else
		dma_address = (u64)sdmmc_calculate_dma_addr(_current_accessor, req->buf,
			(blkcnt * req->blksize + 511) / 512);
	if (dma_address & 7)
		dma_address = 0;
	if (!dma_address)
	{
		/* The caller's buffer is outside the mapped windows. Keep DMA
		   inside a mapped buffer. The storage loop handles partial transfers. */
		u32 capacity = 0;
		for (unsigned i = 0; i < 3; ++i) {
			const sdmmc_dma_buffer_t *pool = &_current_accessor->parent->dmaBuffers[i];
			u64 address = (u64)pool->device_addr_buffer_masked;
			u32 blocks = MIN(pool->device_addr_buffer_size / req->blksize, blkcnt);
			uintptr_t pool_start = (uintptr_t)pool->device_addr_buffer;
			uintptr_t source = (uintptr_t)req->buf;
			bool overlaps = source >= pool_start ? source - pool_start < pool->device_addr_buffer_size :
				pool_start - source < (u64)blkcnt * req->blksize;
			if (!pool_start || !address || (address & 7) || blocks <= capacity || overlaps)
				continue;
			capacity = blocks;
			dma_address = address;
			sdmmc->last_dma_idx = i;
			if (capacity == blkcnt)
				break;
		}
		if (!capacity || (capacity < blkcnt && !req->is_multi_block))
			return 0;
		blkcnt = capacity;
		sdmmc_memcpy_buf = true;
	}

	// Check alignment.
	if (dma_address & 7)
		return 0;

	const u32 boundary = 512 * 1024;
	u64 address_register = dma_address;
#ifdef EMUMMC_DDR200_ADMA_PROBE
	sdmmc->dma_progress.mode = adma ? SdmmcDmaAdma2 : SdmmcDmaSdma512;
	sdmmc->dma_progress.blocks = blkcnt;
	sdmmc->dma_progress.block_bytes = req->blksize;
	sdmmc->dma_progress.address_low = dma_address & (boundary - 1);
	if (adma) {
		if (!sdmmc_adma2_supported(sdmmc))
			return 0;
		u32 bytes = blkcnt * req->blksize;
		address_register = sdmmc_adma_table(dma_address, bytes);
		if (!address_register)
			return 0;
		sdmmc->dma_progress.descriptors = (bytes + SdmmcAdmaChunk - 1) / SdmmcAdmaChunk;
		/* Version 4 selects ADMA2 with bit 4. Host Control 2
		   already enables the 64-bit address and 16-byte descriptor. */
		sdmmc->regs->hostctl = (sdmmc->regs->hostctl & ~SDHCI_CTRL_DMA_MASK) | SDHCI_CTRL_ADMA32;
	} else if (sdmmc->id == SDMMC_1) {
		sdmmc->regs->hostctl &= ~SDHCI_CTRL_DMA_MASK;
	}
#endif
	sdmmc->regs->admaaddr = (u32)address_register;
	sdmmc->regs->admaaddr_hi = (u32)(address_register >> 32);

	sdmmc->dma_addr_next = (dma_address + boundary) & ~((u64)boundary - 1);
	sdmmc->dma_boundaries_left = ((dma_address & (boundary - 1)) + (u64)blkcnt * req->blksize) / boundary;
#ifdef EMUMMC_DDR200_ADMA_PROBE
	if (adma)
		sdmmc->dma_boundaries_left = 0;
#endif
	sdmmc->regs->blksize = req->blksize | (7 << 12);
	sdmmc->regs->blkcnt = blkcnt;

	if (blkcnt_out)
		*blkcnt_out = blkcnt;

	u32 trnmode = SDHCI_TRNS_DMA;
	if (req->is_multi_block)
		trnmode = SDHCI_TRNS_MULTI | SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_DMA;
	if (!req->is_write)
		trnmode |= SDHCI_TRNS_READ;
	if (req->auto_command == SdmmcAutoStop)
		trnmode |= SDHCI_TRNS_AUTO_CMD12;
	else if (req->auto_command == SdmmcAutoBlockCount) {
		/* In v4 mode SDMA uses admaaddr. sysad is the Auto CMD23
		   argument, which must match the size after DMA window limits. */
		sdmmc->regs->sysad = blkcnt;
		trnmode |= SDHCI_TRNS_AUTO_CMD23;
	}

	sdmmc->regs->trnmod = trnmode;

	return 1;
}

static int _sdmmc_update_dma(sdmmc_t *sdmmc)
{
	u16 remaining = sdmmc->regs->blkcnt;
	u64 now = get_tmr_ms();
	u64 deadline = _sdmmc_limit_ms(sdmmc, now + 5000);
	u64 stalled = now + 1500;
	unsigned polls = 0;
	while (now < deadline && now < stalled) {
		u16 intr;
		int result = _sdmmc_check_mask_interrupt(sdmmc, &intr,
			SDHCI_INT_DATA_END | SDHCI_INT_DMA_END);
		if (result == SDMMC_MASKINT_ERROR)
			break;
		if (intr & SDHCI_INT_DATA_END)
			return 1;
		if (intr & SDHCI_INT_DMA_END) {
			if (!sdmmc->dma_boundaries_left) {
				sdmmc->command_irq = intr;
				break;
			}
			--sdmmc->dma_boundaries_left;
			++sdmmc->dma_resumes;
#ifdef EMUMMC_DDR200_ADMA_PROBE
			if (sdmmc->id == SDMMC_1)
				ddr_trace_sdma_restart(sdmmc->handoff_sdma);
#endif
			sdmmc->regs->admaaddr = (u32)sdmmc->dma_addr_next;
			sdmmc->regs->admaaddr_hi = (u32)(sdmmc->dma_addr_next >> 32);
			sdmmc->dma_addr_next += 512 * 1024;
		}
		/* Keep the boundary interrupt hot path to one status read.
		   A short cadence is enough for timeout and progress checks. */
		if ((++polls & 31u) == 0) {
			now = get_tmr_ms();
			u16 next_remaining = sdmmc->regs->blkcnt;
			if (next_remaining < remaining) {
				remaining = next_remaining;
				stalled = now + 1500;
			}
		}
	}

#ifdef EMUMMC_DDR200_ADMA_PROBE
	_sdmmc_capture_dma_error(sdmmc);
#endif
	_sdmmc_reset(sdmmc);
	return 0;
}

static int _sdmmc_execute_cmd_inner(sdmmc_t *sdmmc, sdmmc_cmd_t *cmd, sdmmc_req_t *req, u32 *blkcnt_out, u64 *dma_us)
{
	sdmmc->command_failure = DdrCommandInhibit;
	int has_req_or_check_busy = req || cmd->check_busy;
	if (!_sdmmc_wait_cmd_data_inhibit(sdmmc, has_req_or_check_busy))
		return 0;

	u32 blkcnt = 0;
	bool is_data_present = false;
	if (req)
	{
		sdmmc->command_failure = DdrCommandDmaSetup;
		if (!_sdmmc_config_dma(sdmmc, &blkcnt, req))
			return 0;

		if(!sdmmc_memcpy_buf)
		{
			// Flush from/to phys
			armDCacheFlush(req->buf, req->blksize * blkcnt);
		}
		else
		{
			if(req->is_write)
			{
				void* dma_addr = &_current_accessor->parent->dmaBuffers[sdmmc->last_dma_idx].device_addr_buffer[0];
				memcpy(dma_addr, req->buf, req->blksize * blkcnt);

				// Flush to phys
				armDCacheFlush(dma_addr, req->blksize * blkcnt);
			}
		}

		is_data_present = true;
	}

	_sdmmc_enable_interrupts(sdmmc);

	sdmmc->command_failure = DdrCommandResponse;
	if (!_sdmmc_send_cmd(sdmmc, cmd, is_data_present)) {
		_sdmmc_mask_interrupts(sdmmc);
		return 0;
	}

	int result = _sdmmc_wait_response(sdmmc);
DPRINTF("rsp(%d): %08X, %08X, %08X, %08X\n", result,
		sdmmc->regs->rspreg0, sdmmc->regs->rspreg1, sdmmc->regs->rspreg2, sdmmc->regs->rspreg3);
	if (result)
	{
		if (cmd->rsp_type)
		{
			sdmmc->expected_rsp_type = cmd->rsp_type;
			result = _sdmmc_cache_rsp(sdmmc, sdmmc->rsp, 0x10, cmd->rsp_type);
		}

		if (req && result) {
			sdmmc->command_failure = DdrCommandTransfer;
			if (sdmmc->id == SDMMC_1)
			{
				u32 detail = cmd->cmd == SD_SWITCH ? ((cmd->arg >> 16) & 0x8000) | (cmd->arg & 0x0fff) : blkcnt;
				ddr_trace_stage(DdrDma, ((u32)cmd->cmd << 16) | detail);
			}
			u64 dma_start = dma_us ? get_tmr_us() : 0;
			result = _sdmmc_update_dma(sdmmc);
			if (dma_us)
				*dma_us = get_tmr_us() - dma_start;
		}
	}

	_sdmmc_mask_interrupts(sdmmc);

	if (result)
	{
		if (req)
		{
			if(!req->is_write)
			{
				if(!sdmmc_memcpy_buf)
				{
					// Flush from phys
					armDCacheFlush(req->buf, req->blksize * blkcnt);
				}
				else
				{
					void* dma_addr = &_current_accessor->parent->dmaBuffers[sdmmc->last_dma_idx].device_addr_buffer[0];
					// Flush from phys
					armDCacheFlush(dma_addr, req->blksize * blkcnt);
					// Copy to buffer
					memcpy(req->buf, dma_addr, req->blksize * blkcnt);
				}
			}

			if (blkcnt_out)
				*blkcnt_out = blkcnt;

			if (req->auto_command == SdmmcAutoStop)
				sdmmc->rsp3 = sdmmc->regs->rspreg3;
		}

		if (cmd->check_busy || req) {
			sdmmc->command_failure = DdrCommandBusy;
			return _sdmmc_wait_card_busy(sdmmc);
		}
	}

	return result;
}

int sdmmc_get_sd_power_enabled(void)
{
	u32 mux = PINMUX_AUX(PINMUX_AUX_DMIC3_CLK);
	u32 pin = gpio_get_pin_state(GPIO_PORT_E, GPIO_PIN_4);
	u32 driven = GpioPinModeGpio | GpioPinOutputEnabled;
	if (!(mux & PINMUX_TRISTATE) && (pin & driven) == driven)
		return (pin & GpioPinOutputHigh) &&
			(!(mux & PINMUX_INPUT_ENABLE) || (pin & GpioPinInputHigh));
	return (mux & PINMUX_INPUT_ENABLE) && (pin & GpioPinInputHigh);
}

bool sdmmc_get_sd_inserted()
{
	return (!gpio_read(GPIO_PORT_Z, GPIO_PIN_1));
}

static void _sdmmc_config_sdmmc1_schmitt()
{
	PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK)  |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_CMD)  |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT3) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT2) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT1) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT0) |= PINMUX_SCHMT;
}

static void _sdmmc_config_sdmmc2_schmitt()
{
	PINMUX_AUX(PINMUX_AUX_SDMMC2_CLK)  |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_CMD)  |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT7) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT6) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT5) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT4) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT3) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT2) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT1) |= PINMUX_SCHMT;
	PINMUX_AUX(PINMUX_AUX_SDMMC2_DAT0) |= PINMUX_SCHMT;
}

static void _sdmmc_config_sdmmc1_pads(bool discharge)
{
	u32 sdmmc1_pin_mask = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

	// Set values for Reset state.
	u32 function = GPIO_MODE_SPIO;
	u32 level    = GPIO_LOW;
	u32 output   = GPIO_OUTPUT_DISABLE;

	// Set values for dicharging.
	if (discharge)
	{
		function = GPIO_MODE_GPIO;
		level    = GPIO_HIGH;
		output   = GPIO_OUTPUT_ENABLE;
	}

	// Set all pads function.
	gpio_config(GPIO_PORT_M, sdmmc1_pin_mask, function);
	// Set all pads output level.
	gpio_write(GPIO_PORT_M, sdmmc1_pin_mask, level);
	// Set all pads output.
	gpio_output_enable(GPIO_PORT_M, sdmmc1_pin_mask, output);
}

static int _sdmmc_config_sdmmc1(bool t210b01)
{
	// Configure SD card detect.
	PINMUX_AUX(PINMUX_AUX_GPIO_PZ1) = PINMUX_INPUT_ENABLE | PINMUX_PULL_UP | 2; // GPIO control, pull up.
	APB_MISC(APB_MISC_GP_VGPIO_GPIO_MUX_SEL) = 0;
	gpio_config(GPIO_PORT_Z, GPIO_PIN_1, GPIO_MODE_GPIO);
	gpio_output_enable(GPIO_PORT_Z, GPIO_PIN_1, GPIO_OUTPUT_DISABLE);
	usleep(100);

	// Check if SD card is inserted.
	if(!sdmmc_get_sd_inserted())
		return 0;

	/*
	* Pinmux config:
	*  DRV_TYPE = DRIVE_2X (for 33 Ohm driver)
	*  E_SCHMT  = ENABLE (for 1.8V),  DISABLE (for 3.3V)
	*  E_INPUT  = ENABLE
	*  TRISTATE = PASSTHROUGH
	*  APB_MISC_GP_SDMMCx_CLK_LPBK_CONTROL = SDMMCx_CLK_PAD_E_LPBK for CLK
	*/

	// Enable deep loopback for SDMMC1 CLK pad.
	APB_MISC(APB_MISC_GP_SDMMC1_CLK_LPBK_CONTROL) = 1;

	// Configure SDMMC1 CLK pinmux, based on state and SoC type.
	if (PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK) != (PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_DOWN)) // Check if CLK pad is already configured.
		PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK) = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | (t210b01 ? PINMUX_PULL_NONE : PINMUX_PULL_DOWN);

	// Configure the reset of SDMMC1 pins pinmux.
	PINMUX_AUX(PINMUX_AUX_SDMMC1_CMD)  = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT3) = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT2) = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT1) = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;
	PINMUX_AUX(PINMUX_AUX_SDMMC1_DAT0) = PINMUX_DRIVE_2X | PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;

	// Force schmitt trigger for T210B01.
	if (t210b01)
		_sdmmc_config_sdmmc1_schmitt();

	// Make sure the SDMMC1 controller is powered.
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_NO_IOPOWER, ~PMC_NO_IOPOWER_SDMMC1_IO_EN, PMC_NO_IOPOWER_SDMMC1_IO_EN);
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_NO_IOPOWER, 0, 0); // Commit write.

	// Inform IO pads that voltage is gonna be 3.3V.
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, PMC_PWR_DET_SDMMC1_IO_EN, PMC_PWR_DET_SDMMC1_IO_EN);
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, 0, 0); // Commit write.

	// Set enable SD card power.
	//PINMUX_AUX(PINMUX_AUX_DMIC3_CLK) = PINMUX_PULL_DOWN | 2; // Proper pinmuxing. Breaks on HOS, takes over 1 minute to recover.
	PINMUX_AUX(PINMUX_AUX_DMIC3_CLK) = PINMUX_INPUT_ENABLE | PINMUX_PULL_DOWN | 1; // Wrong but working pinmuxing. Instant take over by FS.
	gpio_config(GPIO_PORT_E, GPIO_PIN_4, GPIO_MODE_GPIO);
	gpio_write(GPIO_PORT_E, GPIO_PIN_4, GPIO_HIGH);
	gpio_output_enable(GPIO_PORT_E, GPIO_PIN_4, GPIO_OUTPUT_ENABLE);
	usleep(10000);

	// Enable SD card IO power.
	if (!max77620_regulator_set_voltage(REGULATOR_LDO2, 3300000) ||
	    !max77620_regulator_enable(REGULATOR_LDO2, 1))
		return 0;
	usleep(1000);

	// Set pad slew codes to get good quality clock.
	if (!t210b01)
	{
		APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) = (APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) & 0xFFFFFFF) | 0x50000000;
		(void)APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL); // Commit write.
		usleep(1000);
	}

	return 1;
}

static void _sdmmc_config_emmc(u32 id, bool t210b01)
{
	switch (id)
	{
	case SDMMC_2:
		if (!t210b01)
		{
			// Unset park for pads.
			APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL) &= 0xF8003FFF;
			(void)APB_MISC(APB_MISC_GP_EMMC2_PAD_CFGPADCTRL); // Commit write.
		}
		else // Enable schmitt trigger for T210B01.
			_sdmmc_config_sdmmc2_schmitt();
		break;

	case SDMMC_4:
		// Unset park for pads.
		APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL) &= 0xF8003FFF;
		// Set default pad cfg.
		if (t210b01)
			APB_MISC(APB_MISC_GP_EMMC4_PAD_PUPD_CFGPADCTRL) &= 0xFFBFFFF9; // Unset CMD/CLK/DQS powedown.
		// Enable schmitt trigger.
		APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL) |= 1;
		(void)APB_MISC(APB_MISC_GP_EMMC4_PAD_CFGPADCTRL); // Commit write.
		break;
	}
}

int sdmmc_init(sdmmc_t *sdmmc, u32 id, u32 power, u32 bus_width, u32 type, int powersave_enable)
{
	u32 clock;
	u16 divisor;
	u8 vref_sel = 7;
	const u8 *trim_values;

	if (id > SDMMC_4 || id == SDMMC_3)
		return 0;

	memset(sdmmc, 0, sizeof(sdmmc_t));

	sdmmc->regs = (t210_sdmmc_t *)QueryIoMapping(_sdmmc_bases[id], 0x200);
	sdmmc->id = id;
	sdmmc->clock_stopped = 1;
	sdmmc->t210b01 = splGetSocType() == SplSocType_Mariko;

	trim_values = sdmmc->t210b01 ? _trim_values_t210b01 : _trim_values_t210;

	// Do specific SDMMC HW configuration.
	switch (id)
	{
	case SDMMC_1:
		if (!_sdmmc_config_sdmmc1(sdmmc->t210b01))
			return 0;
		if (sdmmc->t210b01)
			vref_sel = 0;
		else
			sdmmc->manual_cal = 1;
		break;

	case SDMMC_2:
	case SDMMC_4:
		_sdmmc_config_emmc(id, sdmmc->t210b01);
		break;
	}

	// Disable clock if enabled.
	if (clock_sdmmc_is_not_reset_and_enabled(id))
	{
		_sdmmc_sd_clock_disable(sdmmc);
		_sdmmc_commit_changes(sdmmc);
	}

	// Configure and enable selected clock.
	clock_sdmmc_get_card_clock_div(&clock, &divisor, type);
	clock_sdmmc_enable(id, clock);

	// Make sure all sdmmc registers are reset.
	_sdmmc_reset_all(sdmmc);

	sdmmc->clock_stopped = 0;

	// Set default pad IO trimming configuration.
	sdmmc->regs->iospare |= 0x80000; // Enable muxing.
	sdmmc->regs->veniotrimctl &= 0xFFFFFFFB; // Set Band Gap VREG to supply DLL.
	sdmmc->regs->venclkctl = (sdmmc->regs->venclkctl & 0xE0FFFFFB) | (trim_values[sdmmc->id] << 24);
	sdmmc->regs->sdmemcmppadctl =
		(sdmmc->regs->sdmemcmppadctl & TEGRA_MMC_SDMEMCOMPPADCTRL_COMP_VREF_SEL_MASK) | vref_sel;

	// Configure auto calibration values.
	if (!_sdmmc_autocal_config_offset(sdmmc, power))
		return 0;

	// Calibrate pads.
	_sdmmc_autocal_execute(sdmmc, power);

	// Enable internal clock and power.
	if (_sdmmc_enable_internal_clock(sdmmc))
	{
		sdmmc_set_bus_width(sdmmc, bus_width);
		_sdmmc_set_io_power(sdmmc, power);

		if (sdmmc_setup_clock(sdmmc, type))
		{
			sdmmc_card_clock_powersave(sdmmc, powersave_enable);
			_sdmmc_card_clock_enable(sdmmc);
			_sdmmc_commit_changes(sdmmc);

			return 1;
		}
	}

	return 0;
}

#ifdef EMUMMC_SDMMC_UHS_DDR200_SUPPORT
#ifdef EMUMMC_DDR200_ADMA_PROBE
bool sdmmc_recalibrate_ddr200_pads(sdmmc_t *host)
{
	if (host->id != SDMMC_1 || host->powersave_enabled || host->clock_stopped ||
	    sdmmc_get_io_power(host) != SDMMC_POWER_1_8 ||
	    (host->regs->prnsts & (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) ||
	    (host->deadline_us && get_tmr_us() + 12000 >= host->deadline_us))
		return false;

	/* Use the same calibrated offsets and fallback as the native handoff. */
	if (!_sdmmc_autocal_config_offset(host, SDMMC_POWER_1_8))
		return false;
	_sdmmc_autocal_execute(host, SDMMC_POWER_1_8);
	return true;
}
#endif

void sdmmc_ddr200_restore_host_cfg(sdmmc_t *sdmmc)
{
	if (sdmmc->id != SDMMC_1 || sdmmc->clock_stopped)
		return;

	APB_MISC(APB_MISC_GP_SDMMC1_CLK_LPBK_CONTROL) = 1;

	_sdmmc_config_sdmmc1_schmitt();

	if (!sdmmc->t210b01)
	{
		APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) =
			(APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL) & 0xFFFFFFF) | 0x50000000;
		(void)APB_MISC(APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL);
	}

	sdmmc->regs->iospare |= 0x80000;
	sdmmc->regs->veniotrimctl &= 0xFFFFFFFB;
	sdmmc->regs->venclkctl = (sdmmc->regs->venclkctl & 0xE0FFFFFB) |
		((u32)(sdmmc->t210b01 ? _trim_values_t210b01 : _trim_values_t210)[sdmmc->id] << 24);
	sdmmc->regs->sdmemcmppadctl =
		(sdmmc->regs->sdmemcmppadctl & TEGRA_MMC_SDMEMCOMPPADCTRL_COMP_VREF_SEL_MASK) |
		(sdmmc->t210b01 ? 0 : 7);

	_sdmmc_commit_changes(sdmmc);
	usleep(3);
	_sdmmc_reset(sdmmc);

	if (!_sdmmc_autocal_config_offset(sdmmc, SDMMC_POWER_1_8))
		return;

	_sdmmc_autocal_execute(sdmmc, SDMMC_POWER_1_8);
}
#endif

void sdmmc1_disable_power()
{
	// Clear pull down from CLK pad.
	PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK) &= ~PINMUX_PULL_MASK;

	// Set pads to discharge state.
	_sdmmc_config_sdmmc1_pads(true);

	// Disable SD card IO power regulator.
	max77620_regulator_enable(REGULATOR_LDO2, 0);
	usleep(4000);

	// Disable SD card IO power pin.
	gpio_write(GPIO_PORT_E, GPIO_PIN_4, GPIO_LOW);

	// T210/T210B01 WAR: Set start timer for IO and Controller power discharge.
	msleep(239);

	// Disable SDMMC1 controller power.
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_NO_IOPOWER, PMC_NO_IOPOWER_SDMMC1_IO_EN, PMC_NO_IOPOWER_SDMMC1_IO_EN);
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_NO_IOPOWER, 0, 0); // Commit write.

	// Inform IO pads that next voltage might be 3.3V.
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, PMC_PWR_DET_SDMMC1_IO_EN, PMC_PWR_DET_SDMMC1_IO_EN);
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, 0, 0); // Commit write.

	// Set pads to reset state.
	_sdmmc_config_sdmmc1_pads(false);

	// Set pull down to CLK pad.
	PINMUX_AUX(PINMUX_AUX_SDMMC1_CLK) |= PINMUX_PULL_DOWN;
}

void sdmmc_end(sdmmc_t *sdmmc)
{
	if (!sdmmc->clock_stopped)
	{
		if (clock_sdmmc_is_not_reset_and_enabled(sdmmc->id)) {
			_sdmmc_sd_clock_disable(sdmmc);
			_sdmmc_set_io_power(sdmmc, SDMMC_POWER_OFF);
			_sdmmc_commit_changes(sdmmc);
		}

		// Disable SD card power.
		if (sdmmc->id == SDMMC_1)
			sdmmc1_disable_power();

		clock_sdmmc_disable(sdmmc->id);
		sdmmc->clock_stopped = 1;
	}
}

void sdmmc_init_cmd(sdmmc_cmd_t *cmdbuf, u16 cmd, u32 arg, u32 rsp_type, u32 check_busy)
{
	cmdbuf->cmd = cmd;
	cmdbuf->arg = arg;
	cmdbuf->rsp_type = rsp_type;
	cmdbuf->check_busy = check_busy;
}

int sdmmc_execute_cmd(sdmmc_t *sdmmc, sdmmc_cmd_t *cmd, sdmmc_req_t *req, u32 *blkcnt_out)
{
	sdmmc->command_failure = DdrCommandClock;
	sdmmc->command_irq = 0;
	sdmmc->command_auto_stop_error = 0;
	sdmmc->dma_resumes = 0;
	bool sd_data = sdmmc->id == SDMMC_1 && req;
#ifdef EMUMMC_DDR200_ADMA_PROBE
	sdmmc->dma_progress = (sdmmc_dma_progress_t){0};
#endif
	if (!sdmmc->card_clock_enabled) {
#ifdef EMUMMC_DDR200_ADMA_PROBE
		if (sd_data)
			ddr_trace_io_error(sdmmc, cmd->cmd, req->num_sectors);
#endif
		return 0;
	}
	bool sample = sd_data && req->blksize == 512;
	u64 started = sample ? get_tmr_us() : 0;
	u64 dma_us = 0;

	// Recalibrate periodically for SDMMC1.
	if (sdmmc->manual_cal && sdmmc->powersave_enabled)
		_sdmmc_autocal_execute(sdmmc, sdmmc_get_io_power(sdmmc));

	int should_disable_sd_clock = 0;
	if (!(sdmmc->regs->clkcon & SDHCI_CLOCK_CARD_EN))
	{
		should_disable_sd_clock = 1;
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
		_sdmmc_commit_changes(sdmmc);
		usleep((8000 + sdmmc->divisor - 1) / sdmmc->divisor);
	}

#ifdef EMUMMC_DDR200_ADMA_PROBE
	u8 dma_select = sd_data ? sdmmc->regs->hostctl & SDHCI_CTRL_DMA_MASK : 0;
	u16 adma_irq = sd_data ? sdmmc->regs->errintstsen & SDHCI_ERR_INT_ADMA_ERROR : 0;
#endif
	int result = _sdmmc_execute_cmd_inner(sdmmc, cmd, req, blkcnt_out, sample ? &dma_us : NULL);
#ifdef EMUMMC_DDR200_ADMA_PROBE
	if (sd_data) {
		sdmmc->regs->hostctl = (sdmmc->regs->hostctl & ~SDHCI_CTRL_DMA_MASK) | dma_select;
		sdmmc->regs->errintstsen = (sdmmc->regs->errintstsen & ~SDHCI_ERR_INT_ADMA_ERROR) | adma_irq;
		if (sdmmc->handoff_sdma && sdmmc->dma_progress.mode == SdmmcDmaSdma512)
			ddr_trace_handoff_sdma();
	}
	if (!result && sd_data)
		ddr_trace_io_error(sdmmc, cmd->cmd, req->num_sectors);
#endif
	usleep((8000 + sdmmc->divisor - 1) / sdmmc->divisor);

	if (should_disable_sd_clock)
		sdmmc->regs->clkcon &= ~SDHCI_CLOCK_CARD_EN;
	if (sample)
		ddr_trace_sd_perf(sdmmc, req->is_write, result, sdmmc_memcpy_buf,
			result && blkcnt_out ? *blkcnt_out : req->num_sectors, get_tmr_us() - started, dma_us);

	return result;
}

int sdmmc_enable_low_voltage(sdmmc_t *sdmmc)
{
	if(sdmmc->id != SDMMC_1)
		return 0;

	if (!sdmmc_setup_clock(sdmmc, SDHCI_TIMING_UHS_SDR12))
		return 0;

	_sdmmc_commit_changes(sdmmc);

	// Switch to 1.8V and wait for regulator to stabilize. Assume max possible wait needed.
	if (!max77620_regulator_set_voltage(REGULATOR_LDO2, 1800000))
		return 0;
	usleep(150);

	// Inform IO pads that we switched to 1.8V.
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, ~PMC_PWR_DET_SDMMC1_IO_EN, PMC_PWR_DET_SDMMC1_IO_EN);
	smcReadWriteRegister(PMC_BASE + APBDEV_PMC_PWR_DET_VAL, 0, 0); // Commit write.

	// Enable schmitt trigger for better duty cycle and low jitter clock.
	_sdmmc_config_sdmmc1_schmitt();

	_sdmmc_autocal_config_offset(sdmmc, SDMMC_POWER_1_8);
	_sdmmc_autocal_execute(sdmmc, SDMMC_POWER_1_8);
	_sdmmc_set_io_power(sdmmc, SDMMC_POWER_1_8);
	_sdmmc_commit_changes(sdmmc);
	msleep(5); // Wait minimum 5ms before turning on the card clock.

	// Turn on SDCLK.
	if (sdmmc->regs->hostctl2 & SDHCI_CTRL_VDD_180)
	{
		sdmmc->regs->clkcon |= SDHCI_CLOCK_CARD_EN;
		_sdmmc_commit_changes(sdmmc);
		usleep(1000);
		if ((sdmmc->regs->prnsts & SDHCI_DATA_LVL_MASK) == SDHCI_DATA_LVL_MASK)
			return 1;
	}

	return 0;
}
