/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include "../source/emmc/sdmmc.c"

static sdmmc_t host = { .id = SDMMC_1, .divisor = 199680 };
static sdmmc_storage_t card = { .sdmmc = &host, .initialized = 1 };
static unsigned full_setups, reduced_setups, card_checks, reinitializations, io_errors;
static bool full_tunes, reduced_tunes, full_status, read_after_reinit;
static u32 io_failure;

int sdmmc_setup_ddr200_clock(sdmmc_t *target, bool reduced)
{
    assert(target == &host);
    if (reduced) ++reduced_setups;
    else ++full_setups;
    target->divisor = reduced ? 166400 : 199680;
    return 1;
}

int sdmmc_tuning_execute(sdmmc_t *target, u32 timing, u32 command)
{
    assert(target == &host && timing == SDHCI_TIMING_UHS_DDR200);
    assert(command == MMC_SEND_TUNING_BLOCK);
    return target->divisor == 166400 ? reduced_tunes : full_tunes;
}

void sdmmc_init_cmd(sdmmc_cmd_t *cmd, u16 code, u32 arg, u32 response, u32 busy)
{
    *cmd = (sdmmc_cmd_t){ .cmd = code, .arg = arg, .rsp_type = response, .check_busy = busy };
}

int sdmmc_execute_cmd(sdmmc_t *target, sdmmc_cmd_t *cmd, sdmmc_req_t *request, u32 *blocks)
{
    assert(target == &host);
    if (!request) {
        assert(cmd->cmd == MMC_SEND_STATUS);
        ++card_checks;
        return host.divisor == 166400 || full_status;
    }
    if (read_after_reinit && reinitializations && !request->is_write) {
        *blocks = 1;
        return 1;
    }
    target->command_failure = io_failure;
    target->command_irq = (u32)SDHCI_ERR_INT_DATA_CRC << 16;
    return 0;
}

int sdmmc_get_rsp(sdmmc_t *target, u32 *response, u32 size, u32 type)
{
    assert(target == &host && size == 4 && type == SDMMC_RSP_TYPE_1);
    *response = R1_STATE_TRAN << 9;
    return 1;
}

int sdmmc_stop_transmission(sdmmc_t *target, u32 *response)
{
    assert(target == &host);
    *response = 0;
    return 1;
}

bool nx_sd_initialize(bool cycle)
{
    assert(cycle && sd_ddr200_reduced);
    ++reinitializations;
    return _sd_storage_train_DDR200(&card);
}

int nx_sd_init_retry(bool cycle) { (void)cycle; assert(false); return 0; }
void msleep(u64 duration) { assert(duration == 50); }
void ddr_trace_stage(enum DdrStage next, u32 detail) { (void)next; (void)detail; }
void ddr_trace_io_error(const sdmmc_t *target, u32 cmd, u32 blocks)
{
    assert(target == &host && blocks == 1);
    assert(cmd == MMC_READ_MULTIPLE_BLOCK || cmd == MMC_WRITE_MULTIPLE_BLOCK);
    ++io_errors;
}

static void reset(void)
{
    host.divisor = 199680;
    card.csd.busspeed = 200;
    sd_ddr200_reduced = false;
    full_setups = reduced_setups = card_checks = reinitializations = io_errors = 0;
    full_tunes = reduced_tunes = full_status = true;
    read_after_reinit = false;
    io_failure = DdrCommandTransfer;
}

int main(void)
{
    reset();
    assert(_sd_storage_train_DDR200(&card));
    assert(full_setups == 1 && reduced_setups == 0 && card_checks == 1);

    reset();
    full_tunes = false;
    assert(_sd_storage_train_DDR200(&card));
    assert(full_setups == 1 && reduced_setups == 1 && host.divisor == 166400);
    assert(_sd_storage_train_DDR200(&card));
    assert(full_setups == 1 && reduced_setups == 2);

    reset();
    full_tunes = reduced_tunes = false;
    assert(!_sd_storage_train_DDR200(&card));
    assert(full_setups == 1 && reduced_setups == 1 && card_checks == 0);

    reset();
    full_status = false;
    assert(_sd_storage_train_DDR200(&card));
    assert(reduced_setups == 1 && card_checks == 2);

    u8 buffer[512] = {0};
    reset();
    read_after_reinit = true;
    assert(_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(reinitializations == 1 && reduced_setups == 1 && io_errors == 5);

    reset();
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(reinitializations == 1 && io_errors == 8);

    reset();
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 1));
    assert(reinitializations == 0 && !sd_ddr200_reduced);

    reset();
    io_failure = DdrCommandDmaSetup;
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(reinitializations == 0 && !sd_ddr200_reduced);

    puts("DDR200 recovery: full rate, narrower-window recovery, status checks, bounded reads, and no write or DMA-setup reinit passed");
}
