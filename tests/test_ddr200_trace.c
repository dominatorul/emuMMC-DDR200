/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <stdio.h>
#include "../source/utils/ddr_trace.c"

u64 get_tmr_us(void) { return 1; }
void ddr_watchdog_report(u64 *out) { (void)out; }

int main(void)
{
    atmosphere_fatal_error_ctx report = {.magic = ATMOSPHERE_REBOOT_TO_FATAL_MAGIC};
    ddr_trace_report(&report);
    assert((report.gprs[0] & 0xff) == 28 && (u32)report.gprs[4] == 0xffffffffu);
    ddr_trace_read_ahead_cache(12, 18);
    ddr_trace_read_ahead_cache(26, 24);
    ddr_trace_read_ahead_retry(2, 1);
    t210_sdmmc_t regs = {.venclkctl = 32 << 16};
    sdmmc_t host = {.regs = &regs, .command_failure = DdrCommandTransfer,
        .dma_progress = {.mode = SdmmcDmaAdma2, .blocks = 256, .remaining = 128,
            .block_bytes = 512, .descriptors = 3, .sampled = true}};
    ddr_trace_wake_io(&host, MMC_READ_MULTIPLE_BLOCK, (ddr_eye_t){29, 35});
    ddr_trace_report(&report);
    assert((u32)report.gprs[4] == (21u << 16 | 19u));
    assert(fwrite(&report, sizeof(report), 1, stdout) == 1);
}
