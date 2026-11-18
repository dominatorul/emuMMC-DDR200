/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/emmc/sdmmc.c"
#include "../source/emmc/sdmmc_ddr200.c"

static t210_sdmmc_t regs;
static sdmmc_t host = {.regs = &regs, .id = SDMMC_1};
static sdmmc_storage_t card;
static char window[DdrVerifyBytes];
static mmc_obj_t dma;
static sdmmc_accessor_t accessor = {.parent = &dma};
static ddr_eye_t full_eye, reduced_eye;
static u64 now;
static unsigned fail_reads, commands, verifies, failed_verifies, stops, resets;
static unsigned no_ready, hint_scans, switched_to_stock, log_events;
static bool stop_ok, inconsistent, verify_rejected;
static bool card_retained_ddr;
static unsigned rejected_sdr_checks, retained_restorations;
static char flow[256];
static unsigned cursor;
static u32 status_word;
static unsigned verify_taps[16];
#ifdef EMUMMC_DDR200_MATRIX
static unsigned matrix_sweeps, timed_blocks[2];
static enum DdrMatrixStage matrix_result;
#endif

static void event(char action)
{
    if (cursor + 1 == sizeof(flow))
        return;
    flow[cursor++] = action;
    flow[cursor] = 0;
}

u64 get_tmr_us(void) { return now++; }
void usleep(u64 delay) { now += delay; }
void Log(const char *data, ...) { (void)data; }

int sdmmc_setup_ddr200_clock(sdmmc_t *target, bool reduced)
{
    assert(target == &host);
    host.divisor = reduced ? 166400 : 199680;
    regs.hostctl2 = UHS_DDR50_BUS_SPEED;
    return 1;
}

void sdmmc_ddr200_restore_host_cfg(sdmmc_t *target)
{
    assert(target == &host && (regs.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
    event('H');
}

int sdmmc_setup_clock(sdmmc_t *target, u32 type)
{
    assert(target == &host);
    assert(type == SDHCI_TIMING_UHS_SDR104 || type == SDHCI_TIMING_UHS_SDR12);
    host.divisor = type == SDHCI_TIMING_UHS_SDR104 ? 199680 : 24728;
    regs.hostctl2 = type == SDHCI_TIMING_UHS_SDR104 ? UHS_SDR104_BUS_SPEED : UHS_SDR12_BUS_SPEED;
    return 1;
}

int sdmmc_tuning_execute(sdmmc_t *target, u32 type, u32 command)
{
    assert(target == &host && type == SDHCI_TIMING_UHS_SDR104 && command == MMC_SEND_TUNING_BLOCK);
    return 1;
}

static void produce_eye(u32 map[4], ddr_eye_t eye, unsigned period, u32 first, u32 end)
{
    for (u32 tap = first; tap < end; ++tap)
        if ((tap + period - eye.left) % period < ddr_eye_width(eye))
            map[tap / 32] |= 1u << (tap % 32);
}

u32 sdmmc_scan_ddr200(sdmmc_t *target, u32 first, u32 end, u32 raw[4], u32 passing[4])
{
    assert(target == &host && !host.powersave_enabled);
    event('T');
    if (end - first < DdrTapCount)
        ++hint_scans;
    bool reduced = host.divisor == 166400;
    produce_eye(passing, reduced ? reduced_eye : full_eye, reduced ? 55 : 46, first, end);
    memcpy(raw, passing, 4 * sizeof(u32));
    return end - first;
}

void sdmmc_set_ddr200_tap(sdmmc_t *target, u32 tap)
{
    assert(target == &host && tap < DdrTapCount);
    regs.venclkctl = tap << 16;
}

void sdmmc_card_clock_powersave(sdmmc_t *target, int on)
{
    assert(target == &host);
    target->powersave_enabled = on;
}

void sdmmc_init_cmd(sdmmc_cmd_t *cmd, u16 opcode, u32 arg, u32 rsp, u32 busy)
{
    *cmd = (sdmmc_cmd_t){.cmd = opcode, .arg = arg, .rsp_type = rsp, .check_busy = busy};
}

int sdmmc_execute_cmd(sdmmc_t *target, sdmmc_cmd_t *cmd, sdmmc_req_t *req, u32 *done)
{
    assert(target == &host);
    status_word = R1_STATE_TRAN << 9 | R1_READY_FOR_DATA;
    if (!req) {
        assert(cmd->cmd == MMC_SEND_STATUS && cmd->arg == card.rca << 16);
        event('S');
        if (no_ready) {
            --no_ready;
            status_word &= ~R1_READY_FOR_DATA;
        }
        return 1;
    }
    if (cmd->cmd == SD_SWITCH) {
        assert(!req->is_write && req->blksize == 64);
        if (card_retained_ddr) {
            assert(cmd->arg == 0x00ffffff && (regs.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
            ++rejected_sdr_checks;
            return 0;
        }
        memset(req->buf, 0, 64);
        u8 *reply = req->buf;
        reply[9] = 3;
        reply[16] = cmd->arg & 0xff;
        if (cmd->arg == 0x00ffffff)
            reply[16] = UHS_SDR104_BUS_SPEED;
        if (reply[16] == 0xf0)
            reply[16] = 0;
        if (reply[16] == 0xef)
            reply[16] = 0xe0;
        if (cmd->arg == 0x80ffff03)
            ++switched_to_stock;
        if (((cmd->arg >> 8) & 15) != 15)
            reply[15] = (cmd->arg >> 8) & 15;
        return 1;
    }
    assert(req->blksize == 512);
    ++commands;
    unsigned tap = regs.venclkctl >> 16;
    if (req->num_sectors == DdrVerifyBlocks) {
        assert(cmd->cmd == MMC_READ_MULTIPLE_BLOCK);
        if (verifies < sizeof(verify_taps) / sizeof(verify_taps[0]))
            verify_taps[verifies] = tap;
        ++verifies;
        if (!verify_rejected) {
            memset(req->buf, inconsistent && verifies % 2 == 0 ? 0x22 : 0x11, DdrVerifyBytes);
            *done = DdrVerifyBlocks;
            return 1;
        }
        ++failed_verifies;
    } else if (!fail_reads) {
        *done = req->num_sectors;
        return 1;
    } else {
        --fail_reads;
    }
    event('F');
    host.command_failure = DdrCommandTransfer;
    host.command_irq = 0x01208000;
    return 0;
}

int sdmmc_get_rsp(sdmmc_t *target, u32 *rsp, u32 size, u32 type)
{
    assert(target == &host && size == 4 && type == SDMMC_RSP_TYPE_1);
    *rsp = status_word;
    return 1;
}

int sdmmc_stop_transmission(sdmmc_t *target, u32 *rsp)
{
    assert(target == &host && rsp);
    event('A');
    ++stops;
    return stop_ok;
}

bool sdmmc_reset_transfer(sdmmc_t *target)
{
    assert(target == &host);
    event('R');
    ++resets;
    return true;
}

void armDCacheFlush(void *buf, size_t size)
{
    assert(buf == window && (size == 512 || size == DdrVerifyBytes));
}

void ddr_trace_io_error(const sdmmc_t *target, u32 opcode, u32 count)
{
    assert(target == &host && count && (opcode == MMC_READ_MULTIPLE_BLOCK || opcode == MMC_WRITE_MULTIPLE_BLOCK));
}
void ddr_trace_hygiene(bool stopped, bool ready, bool reset) { assert(reset == ready && (!ready || stopped)); }
void ddr_trace_read_recovery(const sdmmc_t *target, bool ok) { assert(target == &host); (void)ok; }
void ddr_trace_stage(enum DdrStage stage, u32 data) { (void)stage; (void)data; }
void ddr_trace_reassert(void) {}
void ddr_trace_power(enum DdrPowerEvent event, bool ok)
{
    assert(event == DdrPowerRetained && ok && card_retained_ddr);
    ++retained_restorations;
}
void ddr_trace_card(const u8 *data, bool selected) { (void)data; (void)selected; }
void ddr_trace_functions(u16 drive, u16 power, u8 chosen_drive, u8 chosen_power)
{
    (void)drive; (void)power; (void)chosen_drive; (void)chosen_power;
}
void ddr_trace_verify(unsigned tap, unsigned second, bool success)
{
    assert(tap != second && tap < DdrTapCount && second < DdrTapCount);
    if (verify_rejected || inconsistent)
        assert(!success);
}
void ddr_trace_ladder(enum DdrRate rate, bool ready, bool unstable, unsigned upgrades)
{
    assert(rate <= DdrRateFull);
    (void)ready; (void)unstable; (void)upgrades;
    ++log_events;
}
void ddr_trace_policy_sweep(const sdmmc_t *target, ddr_eye_t eye, unsigned tap, u8 kind, u8 drive, u64 started)
{
    assert(target == &host && started <= now);
    if (tap < DdrTapCount && kind < DdrSweepNoise)
        assert(tap >= eye.left + DdrTapMargin && tap + DdrTapMargin <= eye.right);
#ifdef EMUMMC_DDR200_MATRIX
    if (kind >= DdrSweepNoise && kind != DdrSweepSettleStart)
        ++matrix_sweeps;
#endif
    (void)kind; (void)drive;
}

#ifdef EMUMMC_DDR200_MATRIX
bool ddr_watchdog_volume_down(void) { return false; }
void ddr_trace_matrix(enum DdrMatrixStage state, unsigned phase, u32 detail)
{
    (void)phase;
    assert(!detail);
    matrix_result = state;
}
void ddr_trace_matrix_transfer(bool reduced, u32 blocks, u64 elapsed)
{
    assert(blocks == DdrVerifyBlocks && elapsed);
    timed_blocks[reduced] += blocks;
}
#endif

static void reset(void)
{
    now = 1000;
    memset(&host, 0, sizeof(host));
    memset(&regs, 0, sizeof(regs));
    host.regs = &regs;
    host.id = SDMMC_1;
    host.divisor = 199680;
    host.powersave_enabled = SDMMC_POWER_SAVE_ENABLE;
    regs.hostctl2 = UHS_DDR50_BUS_SPEED;
    regs.venclkctl = 36 << 16;
    memset(&card, 0, sizeof(card));
    card.sdmmc = &host;
    card.initialized = true;
    card.rca = 0xabcd;
    card.has_sector_access = true;
    card.sec_cnt = 0x100000;
    card.ddr.rate = DdrRateFull;
    card.ddr.ready = card.ddr.capable = true;
    card.csd.busspeed = 200;
    _current_accessor = &accessor;
    dma.dmaBuffers[0].device_addr_buffer = window;
    dma.dmaBuffers[0].device_addr_buffer_masked = (char *)0x1000;
    dma.dmaBuffers[0].device_addr_buffer_size = sizeof(window);
    memset(window, 0x6f, sizeof(window));
    full_eye = (ddr_eye_t){29, 35};
    reduced_eye = (ddr_eye_t){32, 46};
    stops = resets = no_ready = verifies = failed_verifies = commands = fail_reads = 0;
    hint_scans = switched_to_stock = log_events = cursor = 0;
    stop_ok = true;
    inconsistent = verify_rejected = card_retained_ddr = false;
    rejected_sdr_checks = retained_restorations = 0;
    flow[0] = 0;
#ifdef EMUMMC_DDR200_MATRIX
    memset(&matrix, 0, sizeof(matrix));
    matrix_sweeps = timed_blocks[0] = timed_blocks[1] = 0;
    matrix_result = DdrMatrixPending;
#endif
}

static void test_eye_model(void)
{
    ddr_history_t history = {0};
    const ddr_eye_t eyes[] = {{32, 48}, {29, 44}, {29, 35}, {29, 44}};
    u32 excluded[4] = {0};
    for (unsigned i = 0; i < 4; ++i) {
        u32 map[4] = {0};
        produce_eye(map, eyes[i], 46, 0, DdrTapCount);
        ddr_eye_t measured = ddr_first_eye(map, 15, 64, DdrRuntimeWidth);
        assert(measured.left == eyes[i].left && measured.right == eyes[i].right);
        ddr_remember_eye(&history, measured);
        int tap = ddr_pick_tap(&history, excluded);
        assert(tap >= measured.left + DdrTapMargin && tap + DdrTapMargin <= measured.right);
        unsigned next = MIN(i + 1, 3u);
        assert(tap >= eyes[next].left && tap <= eyes[next].right);
    }
    assert(ddr_failure_next(DdrRateFull) == DdrRateReduced);
    assert(ddr_failure_next(DdrRateReduced) == DdrRateSdr104);
    assert(ddr_failure_next(DdrRateSdr104) == DdrRateUnknown);
    assert(ddr_retry_delay(0) == 5000000 && ddr_retry_delay(2) == 20000000);
    assert(ddr_retry_delay(6) == 300000000);
}

int main(void)
{
    test_eye_model();
    u8 functions[64] = {[10] = 0x40};
    u8 buf[16 * 512];
    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(!sdmmc_ddr200_initial(&card, functions) && !verifies);

    reset();
    full_eye = (ddr_eye_t){32, 48};
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(sdmmc_ddr200_initial(&card, functions) && tap_readback(&host) == 40);
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    full_eye = (ddr_eye_t){29, 44};
    assert(sdmmc_ddr200_handoff(&card) && tap_readback(&host) == 35 && verifies == 2);
    assert(strchr(flow, 'H'));

    reset();
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){29, 35});
    assert(sdmmc_ddr200_prepare_suspend(&card));
    assert(card.ddr.ready && card.ddr.rate == DdrRateSdr104 && card.ddr.resume_rate == DdrRateFull);
    assert(switched_to_stock == 1 && card.csd.busspeed == 104 && !verifies && !stops);
    assert(sdmmc_ddr200_handoff(&card));
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && card.ddr.resume_rate == DdrRateUnknown);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    /* Native wake restored the host to SDR104 while the card kept DDR200. */
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    card_retained_ddr = true;
    assert(sdmmc_ddr200_handoff(&card));
    assert(rejected_sdr_checks == 2 && retained_restorations == 1 && !switched_to_stock);
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && verifies == 4);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    fail_reads = 1;
    no_ready = 1;
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){29, 44});
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(strncmp(flow, "FASSRT", 6) == 0 && stops == 1 && resets == 1 && hint_scans);
    assert(card.ddr.rate == DdrRateFull && card.ddr.ready && verifies == 2);
    assert(verify_taps[0] == 32 && verify_taps[1] != 32 && verify_taps[1] != 36);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    fail_reads = 1;
    full_eye = (ddr_eye_t){29, 34};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateReduced && card.ddr.unstable && verifies == 2 && log_events >= 2);
    assert(!ddr_should_upgrade(&card.ddr, now));
    now += DdrCoolIdleUs * 2;
    full_eye = (ddr_eye_t){29, 44};
    assert(sdmmc_ddr200_before_io(&card, 1024) && card.ddr.rate == DdrRateFull);
    assert(card.ddr.upgrades == 1 && card.ddr.unstable);

    reset();
    inconsistent = true;
    fail_reads = 1;
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateSdr104 && switched_to_stock == 1 && verifies >= 4);

    reset();
    verify_rejected = true;
    fail_reads = 1;
    reduced_eye = (ddr_eye_t){32, 36};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateSdr104 && failed_verifies == 1 && switched_to_stock == 1);

    reset();
    fail_reads = 1;
    stop_ok = false;
    assert(!_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(!card.ddr.ready && card.ddr.rate == DdrRateUnknown && !resets && !verifies);
    assert(strcmp(flow, "FA") == 0);

    reset();
    fail_reads = 1;
    assert(!_sdmmc_storage_readwrite(&card, 1024, 16, buf, 1));
    assert(stops == 1 && resets == 1 && verifies == 2 && commands == 3);

#ifdef EMUMMC_DDR200_MATRIX
    reset();
    full_eye = (ddr_eye_t){29, 44};
    card.ddr.drive_support = 3;
    for (unsigned step = 0; sdmmc_ddr200_matrix_step(&card); ++step) {
        assert(step < 160 && host.deadline_us == 0 && card.ddr.ready);
        now += matrix.phase == MatrixIdle ? MatrixIdleUs : 100000;
    }
    assert(matrix_result == DdrMatrixComplete && !card.ddr.matrix_active);
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && card.ddr.drive == 0);
    assert(timed_blocks[0] == MatrixSpeedBlocks && timed_blocks[1] == MatrixSpeedBlocks);
    assert(matrix_sweeps == 37 && !stops);
    puts("DDR200 matrix completed both 64 MiB loads and both 256 MiB timings");
#endif
    puts("DDR200 eye drift, read verification, fallback, backoff and CMD12/CMD13 hygiene passed");
}
