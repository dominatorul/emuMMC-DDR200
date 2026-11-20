/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/emmc/sdmmc.c"
#include "../source/emmc/sdmmc_ddr200.c"

#ifdef EMUMMC_DDR200_ADMA_PROBE
void ddr_trace_wake(enum DdrWakeStep step, u32 detail, u32 result)
{ (void)step; (void)detail; (void)result; }
void ddr_trace_wake_host(const sdmmc_t *host) { (void)host; }
#endif

static t210_sdmmc_t regs;
static sdmmc_t host = {.regs = &regs, .id = SDMMC_1};
static sdmmc_storage_t card;
static char window[DdrVerifyBytes];
static mmc_obj_t dma;
static sdmmc_accessor_t accessor = {.parent = &dma};
static ddr_eye_t full_eye, reduced_eye;
static u64 now;
static unsigned fail_reads, commands, verifies, failed_verifies, stops, resets;
static bool reject_type_c;
#ifdef EMUMMC_DDR200_ADMA_PROBE
enum { RelaxedProbes = 3 };
static unsigned full_probe_failures, full_data_taps[8], full_data_calls, drop_probes;
static unsigned handoff_data, wake_remaps, wake_attempts;
static bool wake_modes[4];
static bool inject_adma_error;
bool sdmmc_adma_remap(void) { ++wake_remaps; return true; }
void fatal_abort(enum FatalReason reason) { (void)reason; __builtin_trap(); }
static bool reject_handback;
static enum DdrDropReason drop_reason;
void ddr_trace_clock_drop(enum DdrRate from, enum DdrRate to, enum DdrDropReason reason,
    u8 probes, ddr_eye_t eye)
{
    assert(from > to);
    (void)eye;
    drop_reason = reason;
    drop_probes = probes;
}
#else
enum { RelaxedProbes = 2 };
#endif
#ifdef EMUMMC_DDR200_COUNTED_READS
static enum SdmmcAutoCommand last_auto_command;
static unsigned counted_reads;
static bool counted_reads_disabled;
#endif
static unsigned no_ready, hint_scans, switched_to_stock, log_events;
static bool stop_ok, inconsistent, verify_rejected, verify_guarded_only;
static bool card_retained_ddr;
static unsigned rejected_sdr_checks, retained_restorations;
static unsigned group4_request;
static bool retained_full_clock_fails, retained_reduced_clock_fails;
static unsigned retained_clock_changes, retained_failures;
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
    if (card_retained_ddr) {
        ++retained_clock_changes;
        if (reduced ? retained_reduced_clock_fails : retained_full_clock_fails)
            return 0;
    }
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
#ifdef EMUMMC_DDR200_ADMA_PROBE
        host.dma_progress = (sdmmc_dma_progress_t){0};
        host.dma_resumes = 0;
#endif
        if (no_ready) {
            --no_ready;
            status_word &= ~R1_READY_FOR_DATA;
        }
        return 1;
    }
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (host.handoff_sdma)
        ++handoff_data;
    if (wake_remaps || host.wake_commands)
        if (wake_attempts < 4) wake_modes[wake_attempts++] = host.wake_sdma;
    if (reject_handback && cmd->cmd == SD_SWITCH && cmd->arg == 0x80ffff03) {
        assert(host.handoff_sdma);
        return 0;
    }
#endif
    if (cmd->cmd == SD_SWITCH) {
        assert(!req->is_write && req->blksize == 64);
        if ((cmd->arg & 0xffff0fff) == 0x80ff0fff)
            group4_request = (cmd->arg >> 12) & 15;
        if (card_retained_ddr) {
            assert(cmd->arg == 0x00ffffff && (regs.hostctl2 & 7) == UHS_SDR104_BUS_SPEED);
            ++rejected_sdr_checks;
            return 0;
        }
        memset(req->buf, 0, 64);
        u8 *reply = req->buf;
        reply[9] = 7;
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
        if (reject_type_c && !(cmd->arg & 0x80000000) && reply[15] == SD_SET_DRIVER_TYPE_C)
            reply[15] = 15;
        return 1;
    }
    assert(req->blksize == 512);
#ifdef EMUMMC_DDR200_COUNTED_READS
    last_auto_command = req->auto_command;
#endif
    ++commands;
    unsigned tap = regs.venclkctl >> 16;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (host.divisor == 199680 && req->num_sectors == 16 && full_data_calls < 8)
        full_data_taps[full_data_calls++] = tap;
    bool injected = host.divisor == 199680 && full_probe_failures && req->num_sectors == DdrVerifyBlocks;
    if (injected)
        --full_probe_failures;
#else
    bool injected = false;
#endif
    if (req->num_sectors == DdrVerifyBlocks) {
        assert(cmd->cmd == MMC_READ_MULTIPLE_BLOCK);
        if (verifies < sizeof(verify_taps) / sizeof(verify_taps[0]))
            verify_taps[verifies] = tap;
        ++verifies;
        bool guarded = tap >= full_eye.left + DdrTapMargin && tap + DdrTapMargin <= full_eye.right;
        if (!injected && !verify_rejected && (!verify_guarded_only || host.divisor != 199680 || guarded)) {
            memset(req->buf, inconsistent ? (int)verifies : 0x11, DdrVerifyBytes);
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
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (inject_adma_error)
        host.command_irq = (SDHCI_ERR_INT_ADMA_ERROR << 16) | 0x8000;
    if (host.prefetch) {
        assert(host.deadline_us == now + 199999 && req->auto_command == SdmmcAutoStop);
        host.dma_progress = (sdmmc_dma_progress_t){.mode = SdmmcDmaAdma2,
            .blocks = req->num_sectors, .remaining = 128, .sampled = true};
        host.dma_resumes = 3;
        now = host.deadline_us + 1;
    }
#endif
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
#ifdef EMUMMC_DDR200_ADMA_PROBE
    if (host.prefetch)
        assert(host.deadline_us > now);
#endif
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
#ifdef EMUMMC_DDR200_COUNTED_READS
void ddr_trace_counted_read(u8 version, bool supported, bool disabled, bool completed)
{
    assert(!completed || (supported && version >= 2 && !disabled));
    counted_reads += completed;
    counted_reads_disabled = disabled;
}
#endif
void ddr_trace_stage(enum DdrStage stage, u32 data) { (void)stage; (void)data; }
void ddr_trace_reassert(void) {}
void ddr_trace_power(enum DdrPowerEvent event, bool ok)
{
    assert(event == DdrPowerRetained && card_retained_ddr);
    retained_restorations += ok;
    retained_failures += !ok;
}
void ddr_trace_card(const u8 *data, bool selected) { (void)data; (void)selected; }
void ddr_trace_functions(u16 drive, u16 power, u8 chosen_drive, u8 chosen_power)
{
    (void)drive; (void)power; (void)chosen_drive; (void)chosen_power;
}
void ddr_trace_verify(unsigned tap, unsigned second, bool success)
{
    assert(!success || (tap != second && tap < DdrTapCount && second < DdrTapCount));
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
    if (tap < DdrTapCount && kind < DdrSweepNoise) {
        unsigned guard = target->divisor == 199680 && kind != DdrSweepInitial &&
                         ddr_eye_width(eye) < 2 * DdrTapMargin + 1 ? DdrNarrowMargin : DdrTapMargin;
        assert(tap >= eye.left + guard && tap + guard <= eye.right);
    }
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
    card.ddr.last_sweep_us = now;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    full_probe_failures = full_data_calls = drop_probes = 0;
    handoff_data = wake_remaps = wake_attempts = 0;
    inject_adma_error = false;
    reject_handback = false;
    drop_reason = DdrDropUnknown;
#endif
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
    reject_type_c = false;
    inconsistent = verify_rejected = verify_guarded_only = card_retained_ddr = false;
    rejected_sdr_checks = retained_restorations = 0;
    retained_clock_changes = retained_failures = 0;
    retained_full_clock_fails = retained_reduced_clock_fails = false;
    flow[0] = 0;
#ifdef EMUMMC_DDR200_MATRIX
    memset(&matrix, 0, sizeof(matrix));
    matrix_sweeps = timed_blocks[0] = timed_blocks[1] = 0;
    matrix_result = DdrMatrixPending;
#endif
}

static void test_eye_model(void)
{
    u32 initial_map[4] = {0};
    for (unsigned tap = 2; tap <= 9; ++tap)
        initial_map[0] |= 1u << tap;
    for (unsigned tap = 117; tap < DdrTapCount; ++tap)
        initial_map[tap / 32] |= 1u << (tap % 32);
    ddr_eye_t widest = ddr_widest_eye(initial_map, DdrTapCount);
    assert(widest.left == 117 && widest.right == 127);
    widest = ddr_widest_eye(initial_map, 124);
    assert(widest.left == 2 && widest.right == 9);

    reset();
    full_eye = (ddr_eye_t){19, 40};
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){78, 85});
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    regs.venclkctl = 81 << 16;
    assert(train(&card, DdrRateFull, DdrSweepRecovery, true, 1024, 81));
    /* The local scan starts inside this eye, forcing a full scan. */
    assert(hint_scans == 1 && card.ddr.history[0].eyes[0].left == 65);
    assert(card.ddr.history[0].eyes[0].right == 86 && tap_readback(&host) == 82);

    ddr_history_t history = {0};
    const ddr_eye_t eyes[] = {{32, 48}, {29, 44}, {29, 35}, {29, 44}};
    u32 excluded[4] = {0};
    for (unsigned i = 0; i < 4; ++i) {
        u32 map[4] = {0};
        produce_eye(map, eyes[i], 46, 0, DdrTapCount);
        ddr_eye_t measured = ddr_first_eye(map, 15, 64, DdrRuntimeWidth);
        assert(measured.left == eyes[i].left && measured.right == eyes[i].right);
        ddr_remember_eye(&history, measured);
        int tap = ddr_pick_tap(&history, excluded, DdrTapMargin, false);
        assert(tap >= measured.left + DdrTapMargin && tap + DdrTapMargin <= measured.right);
        unsigned next = MIN(i + 1, 3u);
        assert(tap >= eyes[next].left && tap <= eyes[next].right);
    }
    assert(ddr_failure_next(DdrRateFull) == DdrRateReduced);
    assert(ddr_failure_next(DdrRateReduced) == DdrRateSdr104);
    assert(ddr_failure_next(DdrRateSdr104) == DdrRateUnknown);
    assert(ddr_retry_delay(0) == 5000000 && ddr_retry_delay(2) == 20000000);
    assert(ddr_retry_delay(6) == 300000000);

    history = (ddr_history_t){0};
    ddr_remember_eye(&history, (ddr_eye_t){29, 35});
    ddr_remember_eye(&history, (ddr_eye_t){29, 36});
    assert(ddr_pick_tap(&history, excluded, DdrTapMargin, true) == 32);
    exclude_tap(excluded, 32);
    assert(ddr_pick_tap(&history, excluded, DdrTapMargin, true) == 33);
    exclude_tap(excluded, 33);
    assert(ddr_pick_tap(&history, excluded, DdrTapMargin, true) == -1);
}

int main(void)
{
    test_eye_model();
    reset();
    sdmmc_sd_set_current_limit(&card, SD_MAX_CURRENT_800 | SD_MAX_CURRENT_400, (u8 *)window);
    assert(group4_request == SD_SET_CURRENT_LIMIT_400);
    sdmmc_sd_set_current_limit(&card, SD_MAX_CURRENT_800, (u8 *)window);
    assert(group4_request == SD_SET_CURRENT_LIMIT_200);
    u8 functions[64] = {[9] = 7, [10] = 0x40};
    u8 buf[16 * 512];
#ifdef EMUMMC_DDR200_ADMA_PROBE
    reset();
    host.wake_commands = 8;
    regs.capareg = SDHCI_CAN_ADMA2;
    regs.hcver = 3;
    regs.hostctl2 |= SDHCI_HOST_VERSION_4_EN | SDHCI_ADDRESSING_64BIT_EN;
    fail_reads = 2;
    inject_adma_error = true;
    u32 retried = 0;
    assert(wake_transfer(&card, &retried, 0, 16, buf, false));
    assert(retried == 16 && wake_remaps == 1 && wake_attempts == 3 && host.wake_commands == 7);
    assert(!wake_modes[0] && !wake_modes[1] && wake_modes[2] && !host.wake_sdma);
    reset();
    host.wake_commands = 8;
    regs.venclkctl = 33 << 16;
    full_eye = (ddr_eye_t){30, 44};
    fail_reads = 1;
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, false));
    assert(!wake_remaps && host.wake_commands == 7 && card.ddr.rate == DdrRateFull);
    assert(full_data_calls == 2 && full_data_taps[0] == 33 && full_data_taps[1] != 33);
    assert(stops == 1 && resets == 1);
    reset();
    host.prefetch = true;
    fail_reads = no_ready = 1;
    u32 completed = 0;
    assert(!sdmmc_storage_transfer(&card, &completed, 1024, 4096, window, 0));
    assert(strcmp(flow, "FASSR") == 0 && stops == 1 && resets == 1);
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && !host.deadline_us);
    assert(host.dma_progress.sampled && host.dma_progress.remaining == 128 && host.dma_resumes == 3);
#endif
#ifdef EMUMMC_DDR200_COUNTED_READS
    reset();
    u32 transferred;
    regs.hcver = 2;
    regs.hostctl2 |= SDHCI_HOST_VERSION_4_EN | SDHCI_ADDRESSING_64BIT_EN;
    assert(sdmmc_storage_transfer(&card, &transferred, 1024, 1024, window, 0));
    assert(last_auto_command == SdmmcAutoStop);
    card.scr.cmds = SD_SCR_CMD23_SUPPORT;
    assert(sdmmc_storage_transfer(&card, &transferred, 1024, 1024, window, 1));
    assert(last_auto_command == SdmmcAutoStop);
    assert(sdmmc_storage_transfer(&card, &transferred, 1024, 1024, window, 0));
    assert(last_auto_command == SdmmcAutoBlockCount && counted_reads == 1);
    fail_reads = 1;
    assert(!sdmmc_storage_transfer(&card, &transferred, 1024, 1024, window, 0));
    assert(stops == 1 && resets == 1 && counted_reads_disabled && card.ddr.counted_reads_failed);
    assert(sdmmc_storage_transfer(&card, &transferred, 1024, 1024, window, 0));
    assert(last_auto_command == SdmmcAutoStop && counted_reads == 1);
#endif
    reset();
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(!sdmmc_ddr200_initial(&card, functions) && !verifies);

    reset();
    full_eye = (ddr_eye_t){32, 48};
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(sdmmc_ddr200_initial(&card, functions) && tap_readback(&host) == 40);
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    full_eye = (ddr_eye_t){29, 44};
    assert(sdmmc_ddr200_handoff(&card) && tap_readback(&host) == 32 && verifies == 2);
    assert(card.ddr.history[0].count == 1);
    assert(strchr(flow, 'H'));
    full_eye = (ddr_eye_t){30, 44};
    card.ddr.busy_us = DdrTrackBusyUs;
    assert(sdmmc_ddr200_before_io(&card, 1024, false) && tap_readback(&host) == 33);
    full_eye = (ddr_eye_t){30, 36};
    card.ddr.busy_us = DdrTrackBusyUs;
    assert(sdmmc_ddr200_before_io(&card, 1024, false) && tap_readback(&host) == 33);
    assert(card.ddr.rate == DdrRateFull && verifies == 4 + RelaxedProbes);
#ifdef EMUMMC_DDR200_DRIVE_C
    assert(card.ddr.drive == SD_SET_DRIVER_TYPE_C);
    reset();
    reject_type_c = true;
    full_eye = (ddr_eye_t){32, 48};
    host.powersave_enabled = SDMMC_POWER_SAVE_DISABLE;
    assert(sdmmc_ddr200_initial(&card, functions));
    assert(card.ddr.drive == SD_SET_DRIVER_TYPE_A);
#elif defined(EMUMMC_DDR200_PERF_TRIAL)
    assert(card.ddr.drive == SD_SET_DRIVER_TYPE_A);
#endif

    reset();
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){29, 35});
    assert(sdmmc_ddr200_prepare_suspend(&card));
    assert(card.ddr.ready && card.ddr.rate == DdrRateSdr104 && card.ddr.resume_rate == DdrRateFull);
    assert(switched_to_stock == 1 && card.csd.busspeed == 104 && !verifies && !stops);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(handoff_data == 1 && !host.handoff_sdma);
#endif
    assert(sdmmc_ddr200_handoff(&card));
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && card.ddr.resume_rate == DdrRateUnknown);
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(handoff_data >= 4 && !host.handoff_sdma);
    unsigned before_runtime = handoff_data;
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0) && handoff_data == before_runtime);
#endif
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    /* Native wake restored the host to SDR104 while the card kept DDR200. */
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    card_retained_ddr = true;
    assert(sdmmc_ddr200_handoff(&card));
    assert(rejected_sdr_checks == 2 && retained_restorations == 1 && !switched_to_stock);
    assert(card.ddr.ready && card.ddr.rate == DdrRateFull && verifies == 2 && retained_clock_changes == 1);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    card_retained_ddr = retained_full_clock_fails = true;
    assert(sdmmc_ddr200_handoff(&card));
    assert(card.ddr.ready && card.ddr.rate == DdrRateReduced && verifies == 2 && retained_clock_changes == 2);
    assert(card.ddr.unstable && card.ddr.retry_at > now && retained_restorations == 1);

    reset();
    regs.hostctl2 = UHS_SDR104_BUS_SPEED;
    card_retained_ddr = retained_reduced_clock_fails = true;
    card.ddr.resume_rate = DdrRateReduced;
    assert(!sdmmc_ddr200_handoff(&card));
    assert(!card.ddr.ready && card.ddr.rate == DdrRateUnknown && !verifies && retained_clock_changes == 1);
    assert(retained_failures == 1 && host.powersave_enabled && !host.deadline_us);

#ifdef EMUMMC_DDR200_ADMA_PROBE
    reset();
    reject_handback = true;
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){31, 36});
    assert(!sdmmc_ddr200_prepare_suspend(&card));
    assert(handoff_data == 1 && !host.handoff_sdma && !host.deadline_us);
    assert(!card.ddr.ready && card.ddr.resume_rate == DdrRateFull);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);
#endif

    reset();
    fail_reads = 1;
    no_ready = 1;
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){29, 44});
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(strncmp(flow, "FASSRT", 6) == 0 && stops == 1 && resets == 1 && hint_scans);
    assert(card.ddr.rate == DdrRateFull && card.ddr.ready && verifies == RelaxedProbes);
    assert(verify_taps[0] == 32 && verify_taps[1] != 32 && verify_taps[1] != 36);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    reset();
    full_eye = (ddr_eye_t){29, 36};
    verify_guarded_only = true;
    card.ddr.busy_us = DdrTrackBusyUs;
    assert(sdmmc_ddr200_before_io(&card, 1024, false));
    assert(card.ddr.rate == DdrRateFull && card.ddr.ready && tap_readback(&host) == 32);
    assert(verifies == 2 && verify_taps[0] == 32 && verify_taps[1] == 33 && !stops);

    reset();
    fail_reads = 1;
    regs.venclkctl = 32 << 16;
    full_eye = (ddr_eye_t){29, 36};
    ddr_remember_eye(&card.ddr.history[0], (ddr_eye_t){29, 35});
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateFull && tap_readback(&host) == 33);
    assert(stops == 1 && resets == 1 && verifies == RelaxedProbes && verify_taps[1] != 32);

    for (unsigned edge = 33; edge <= 34; ++edge) {
        reset();
        fail_reads = 1;
        full_eye = (ddr_eye_t){29, edge};
        assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
        unsigned expected = edge == 33 ? RelaxedProbes : 2;
        assert(card.ddr.rate == DdrRateFull && card.ddr.narrow && verifies == expected);
        assert(tap_readback(&host) == 31 && verify_taps[0] != verify_taps[1]);
        card.ddr.busy_us = DdrNarrowTrackBusyUs;
        assert(sdmmc_ddr200_before_io(&card, 1024, false) && verifies == expected && card.ddr.ready);
        now += DdrNarrowVerifyUs;
        card.ddr.busy_us = DdrNarrowTrackBusyUs;
        assert(sdmmc_ddr200_before_io(&card, 1024, false) && verifies == expected * 2 && card.ddr.ready);
    }

    reset();
    fail_reads = 1;
    full_eye = (ddr_eye_t){29, 32};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(card.ddr.rate == DdrRateReduced && card.ddr.unstable && verifies >= 5 && log_events >= 2);
    assert(drop_reason == DdrDropMargin && drop_probes >= 3);
#else
    assert(card.ddr.rate == DdrRateReduced && card.ddr.unstable && verifies == 2 && log_events >= 2);
#endif
    assert(!ddr_should_upgrade(&card.ddr, now));
    now += DdrCoolIdleUs * 2;
    full_eye = (ddr_eye_t){29, 44};
    assert(sdmmc_ddr200_before_io(&card, 1024, false) && card.ddr.rate == DdrRateFull);
    assert(card.ddr.upgrades == 1 && card.ddr.unstable);

    reset();
    card.ddr.narrow = true;
#ifdef EMUMMC_DDR200_ADMA_PROBE
    full_eye = (ddr_eye_t){29, 33};
#endif
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, window, 1));
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(card.ddr.rate == DdrRateReduced && host.divisor == 166400 && commands == 6 && verifies == 5);
    assert(drop_reason == DdrDropWriteMargin);
#else
    assert(card.ddr.rate == DdrRateReduced && host.divisor == 166400 && commands == 3 && verifies == 2);
#endif
    assert(card.ddr.last_write != 0 && !card.ddr.upgrades);
    for (size_t i = 0; i < sizeof(window); ++i)
        assert(window[i] == 0x6f);

    now += DdrCoolIdleUs * 2;
    card.ddr.blocks = DdrCoolBlocks;
    assert(ddr_should_upgrade(&card.ddr, now));
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 1));
    assert(card.ddr.rate == DdrRateReduced && !card.ddr.upgrades);
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateReduced && !card.ddr.upgrades);

    now += DdrCoolIdleUs;
    full_eye = (ddr_eye_t){29, 44};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateFull && card.ddr.upgrades == 1);

    reset();
    card.ddr.busy_us = DdrTrackBusyUs;
    full_eye = (ddr_eye_t){29, 34};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 1));
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(card.ddr.rate == DdrRateFull && card.ddr.narrow_pair && verifies == 2 && commands == 3);
    assert(verify_taps[0] == 31 && verify_taps[1] == 32 && drop_reason == DdrDropUnknown);
#else
    assert(card.ddr.rate == DdrRateReduced && verifies == 4 && commands == 5);
#endif

    reset();
    inconsistent = true;
    full_eye = (ddr_eye_t){29, 33};
    fail_reads = 1;
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(card.ddr.rate == DdrRateSdr104 && switched_to_stock == 1 && verifies >= 4);

    reset();
    verify_rejected = true;
    fail_reads = 1;
    reduced_eye = (ddr_eye_t){32, 36};
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
#ifdef EMUMMC_DDR200_ADMA_PROBE
    assert(card.ddr.rate == DdrRateSdr104 && failed_verifies >= 4 && switched_to_stock == 1);
#else
    assert(card.ddr.rate == DdrRateSdr104 && failed_verifies == 1 && switched_to_stock == 1);
#endif

#ifdef EMUMMC_DDR200_ADMA_PROBE
    reset();
    full_eye = (ddr_eye_t){31, 36};
    full_probe_failures = 8;
    card.ddr.busy_us = DdrTrackBusyUs;
    assert(sdmmc_ddr200_before_io(&card, 1024, false) && card.ddr.rate == DdrRateReduced);
    assert(drop_reason == DdrDropProbeIo && drop_probes >= 3);
    for (unsigned i = 0; i < drop_probes; ++i)
        for (unsigned j = i + 1; j < drop_probes; ++j)
            assert(verify_taps[i] != verify_taps[j]);

    reset();
    full_eye = (ddr_eye_t){29, 44};
    fail_reads = 4;
    assert(_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(full_data_calls == 4 && drop_reason == DdrDropRetryLimit);
    for (unsigned i = 0; i < full_data_calls; ++i)
        for (unsigned j = i + 1; j < full_data_calls; ++j)
            assert(full_data_taps[i] != full_data_taps[j]);
#endif

    reset();
    fail_reads = 1;
    stop_ok = false;
    assert(!_sdmmc_storage_readwrite(&card, 1024, 16, buf, 0));
    assert(!card.ddr.ready && card.ddr.rate == DdrRateUnknown && !resets && !verifies);
    assert(strcmp(flow, "FA") == 0);

    reset();
    fail_reads = 1;
    assert(!_sdmmc_storage_readwrite(&card, 1024, 16, buf, 1));
    assert(stops == 1 && resets == 1 && verifies == RelaxedProbes && commands == RelaxedProbes + 1);

#ifdef EMUMMC_DDR200_MATRIX
    reset();
    full_eye = (ddr_eye_t){29, 44};
    card.ddr.drive_support = 3;
    for (unsigned step = 0; sdmmc_ddr200_matrix_step(&card); ++step) {
        assert(step < 2 * (MatrixLoadBlocks + MatrixSpeedBlocks) / MatrixSliceBlocks + 40 &&
               host.deadline_us == 0 && card.ddr.ready);
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
