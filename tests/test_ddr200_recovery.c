/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include "../source/emmc/sdmmc.c"

static t210_sdmmc_t regs;
static sdmmc_t host = { .regs = &regs, .id = SDMMC_1, .divisor = 199680 };
static sdmmc_storage_t card = { .sdmmc = &host, .initialized = 1 };
static unsigned full_setups, reduced_setups, card_checks, io_errors;
static unsigned tap_changes, recovery_attempts, recovery_successes;
static bool full_tunes, full_status, powered;
static bool train_after_restore;
static unsigned host_restores;
static bool reopen_succeeds, cid_matches;
static unsigned native_reopens, cid_commands;
static u32 accepted_tap;
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
    assert(target->divisor == 199680 && !host.powersave_enabled);
    host.ddr200_retry_taps[0] = 85;
    host.ddr200_retry_taps[1] = 36;
    host.ddr200_retry_taps[2] = 44;
    host.ddr200_retry_count = 3;
    regs.venclkctl = 40 << 16;
    return full_tunes || (train_after_restore && host_restores);
}

void sdmmc_ddr200_restore_host_cfg(sdmmc_t *target)
{
    assert(target == &host && !host.powersave_enabled);
    ++host_restores;
}

void sdmmc_set_ddr200_tap(sdmmc_t *target, u32 tap)
{
    assert(target == &host && host.powersave_enabled);
    assert(tap_changes < 3 && tap == host.ddr200_retry_taps[tap_changes]);
    ++tap_changes;
    regs.venclkctl = tap << 16;
}

void sdmmc_init_cmd(sdmmc_cmd_t *cmd, u16 code, u32 arg, u32 response, u32 busy)
{
    *cmd = (sdmmc_cmd_t){ .cmd = code, .arg = arg, .rsp_type = response, .check_busy = busy };
}

int sdmmc_execute_cmd(sdmmc_t *target, sdmmc_cmd_t *cmd, sdmmc_req_t *request, u32 *blocks)
{
    assert(target == &host);
    if (!request) {
        if (cmd->cmd == MMC_SELECT_CARD) {
            if (!cid_commands) {
                assert(cmd->arg == 0 && cmd->rsp_type == SDMMC_RSP_TYPE_0);
            } else {
                assert(cid_commands == 2 && cmd->arg == (card.rca << 16));
            }
            ++cid_commands;
            return 1;
        }
        if (cmd->cmd == MMC_SEND_CID) {
            assert(cid_commands == 1 && cmd->arg == (card.rca << 16) && cmd->rsp_type == SDMMC_RSP_TYPE_2);
            ++cid_commands;
            return 1;
        }
        assert(cmd->cmd == MMC_SEND_STATUS);
        ++card_checks;
        return host.divisor == 166400 || full_status;
    }
    if (accepted_tap == regs.venclkctl >> 16 && full_setups && !request->is_write) {
        *blocks = 1;
        return 1;
    }
    target->command_failure = io_failure;
    target->command_irq = (u32)SDHCI_ERR_INT_DATA_CRC << 16;
    return 0;
}

int sdmmc_get_rsp(sdmmc_t *target, u32 *response, u32 size, u32 type)
{
    if (type == SDMMC_RSP_TYPE_2) {
        assert(target == &host && size == sizeof(card.raw_cid));
        memcpy(response, card.raw_cid, size);
        if (!cid_matches)
            response[0] ^= 1;
        return 1;
    }
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

bool nx_sd_initialize(bool cycle) { (void)cycle; assert(false); return false; }

int nx_sd_init_retry(bool cycle) { (void)cycle; assert(false); return 0; }
int emummc_ddr200_reopen_for_read(sdmmc_storage_t *target)
{
    assert(target == &card && host.powersave_enabled && host.divisor == 199680);
    ++native_reopens;
    if (!reopen_succeeds)
        return 0;
    regs.venclkctl = 40 << 16;
    return 1;
}
int sdmmc_get_sd_power_enabled(void) { return powered; }
int clock_sdmmc_is_not_reset_and_enabled(u32 id) { assert(id == SDMMC_1); return 1; }
int sdmmc_get_io_power(sdmmc_t *target) { assert(target == &host); return SDMMC_POWER_1_8; }
u32 sdmmc_get_bus_width(sdmmc_t *target) { assert(target == &host); return SDMMC_BUS_WIDTH_4; }
void sdmmc_card_clock_powersave(sdmmc_t *target, int enable)
{
    assert(target == &host);
    target->powersave_enabled = enable;
}
void msleep(u64 duration) { assert(duration == 50); }
void ddr_trace_stage(enum DdrStage next, u32 detail) { (void)next; (void)detail; }
void ddr_trace_io_error(const sdmmc_t *target, u32 cmd, u32 blocks)
{
    assert(target == &host && blocks == 1);
    assert(cmd == MMC_READ_MULTIPLE_BLOCK || cmd == MMC_WRITE_MULTIPLE_BLOCK);
    ++io_errors;
}

void ddr_trace_read_recovery(const sdmmc_t *target, bool succeeded)
{
    assert(target == &host);
    if (succeeded) ++recovery_successes;
    else ++recovery_attempts;
}

static void reset(void)
{
    host.divisor = 199680;
    regs.hostctl2 = UHS_DDR50_BUS_SPEED;
    regs.venclkctl = 40 << 16;
    host.powersave_enabled = SDMMC_POWER_SAVE_ENABLE;
    card.csd.busspeed = 200;
    host.ddr200_retry_count = 0;
    full_setups = reduced_setups = card_checks = io_errors = 0;
    tap_changes = recovery_attempts = recovery_successes = 0;
    native_reopens = cid_commands = 0;
    host_restores = 0;
    full_tunes = full_status = true;
    train_after_restore = false;
    reopen_succeeds = false;
    cid_matches = true;
    card.rca = 0xABCD;
    accepted_tap = SDMMC_INVALID_TAP;
    powered = true;
    io_failure = DdrCommandTransfer;
}

int main(void)
{
    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(_sd_storage_train_DDR200(&card));
    assert(full_setups == 1 && reduced_setups == 0 && card_checks == 1);

    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    full_tunes = false;
    assert(!_sd_storage_train_DDR200(&card));
    assert(full_setups == 2 && reduced_setups == 0 && card_checks == 0 && host_restores == 1);

    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    full_tunes = false;
    train_after_restore = true;
    assert(_sd_storage_train_DDR200(&card));
    assert(full_setups == 2 && reduced_setups == 0 && card_checks == 1 && host_restores == 1);

    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    full_status = false;
    assert(!_sd_storage_train_DDR200(&card));
    assert(full_setups == 2 && reduced_setups == 0 && card_checks == 2 && host_restores == 1);

    u8 buffer[512] = {0};
    reset();
    accepted_tap = 40;
    assert(_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 1 && reduced_setups == 0 && io_errors == 1);
    assert(recovery_attempts == 1 && recovery_successes == 1 && tap_changes == 0);

    reset();
    accepted_tap = 36;
    assert(_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 1 && reduced_setups == 0 && io_errors == 3);
    assert(recovery_attempts == 1 && recovery_successes == 1 && tap_changes == 2 && !native_reopens);
    assert(regs.venclkctl >> 16 == 36 && host.powersave_enabled);

    reset();
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 1 && reduced_setups == 0 && io_errors == 5 && host.powersave_enabled);
    assert(recovery_attempts == 1 && !recovery_successes && tap_changes == 3 && native_reopens == 1);

    reset();
    full_tunes = false;
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 2 && reduced_setups == 0 && io_errors == 1 && tap_changes == 0 && native_reopens == 1);

    reset();
    full_tunes = false;
    reopen_succeeds = true;
    accepted_tap = 40;
    assert(_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 2 && reduced_setups == 0 && io_errors == 1);
    assert(native_reopens == 1 && recovery_successes == 1 && tap_changes == 0);

    reset();
    assert(sdmmc_storage_verify_sd_cid(&card));
    assert(cid_commands == 3 && card_checks == 1);

    reset();
    cid_matches = false;
    assert(!sdmmc_storage_verify_sd_cid(&card));
    assert(cid_commands == 2 && !card_checks);

    reset();
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 1));
    assert(full_setups == 0 && reduced_setups == 0 && io_errors == 5 && !recovery_attempts && !native_reopens);

    reset();
    io_failure = DdrCommandDmaSetup;
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 0 && reduced_setups == 0 && io_errors == 5 && !recovery_attempts);

    reset();
    powered = false;
    assert(!_sdmmc_storage_readwrite(&card, 10, 1, buffer, 0));
    assert(full_setups == 0 && reduced_setups == 0 && io_errors == 5 && !recovery_attempts);

    puts("DDR200 recovery: full-rate retraining, alternate taps, native reopen, and CID checks passed");
}
