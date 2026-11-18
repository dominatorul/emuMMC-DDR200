# Fusee DDR200 diagnostic

This build starts from Lightos1 develop at `108f1632`. It retains the original
single manual sweep and reassert sequence. The test build requires a successful
DDR200 switch and a passing interval of at least eight taps. It captures both
the hardware comparator results and the subset with completed commands.

The last Hekate test, report `0000000000a49f0b`, reached DDR200, completed
3,097 reads and 73 writes, then failed during recovery with `DD202011`.
The center moved from tap 40 to 33 and the measured width dropped from 17 to 5.
The log showed changes in clock source, host mode, divider and tap.

Fusee report `0000000000bf143a` ended with `DD200063` after 3,102 reads, 81
writes, three reassert attempts and five completed sweeps. CMD6 confirmed
command system 14 and Access Mode 0. Initial and final sweeps had identical SD
clock, host mode, pad, pinmux and calibration values. The window again shrank
from 17 taps at center 40 to 5 taps at center 33. This narrowed window persists
after resetting Access Mode and using the official Fusee boot path.

Hekate and L4T use CMD6 vendor command-system function 14 with the Access Mode
group at SDR12. Native FS can configure that group during SD open. The new build
returns the host and card to low-clock SDR12 before reselecting DDR200 during
in-place recovery. Actual SD power, controller clock, voltage or width loss
reopens the card through native FS before reselecting DDR200. Recovery preserves the 512-byte DMA
scratch sector. The capture includes the original and latest sweeps, card CMD6
status, registers before and after native open, and the reassert entry state.

The previous recovery build tried 199.68 MHz first and used the original
fork's 166.4 MHz clock in the same vendor DDR200 mode if tuning or card status
failed. It retrained the powered card after repeated read failures.
It retained a successful reduced clock setting through reopens and resume
for that boot. Both clocks required eight passing
taps and a valid card status response. The updated trace saves the measured
clock and window at each rate plus the first and latest failed data commands
and interrupt status, before command cleanup overwrites that status.

The recovery build reached the Nintendo logo but did not produce a new report
after 30 seconds. This revision adds an independent watchdog. It saves a report
if an SDMMC wrapper takes 15 active seconds, including time waiting for either
storage mutex. It also caps SDMA polling at 5 seconds overall and 1.5 seconds
without block progress. Boundary interrupts can advance only through the
programmed transfer length. The active DDR200 clock policy is the same.

The stall-trace boot produced an Atmosphère fatal report from `pcv`, Program ID
`010000000000001A`, with result `0x265` (`2101-0001`, I²C NoAck). The associated
PTM and OMM crashes share that boot. The PCV test moved later SD power recovery
to native FS and changed read retries to retune the powered card in place.

PCV-test report `00000000001648d8` now shows the earlier boot transition.
The driver completed 98 reads and 36 writes at 199.68 MHz with a 17-tap window.
SD power GPIO was then low while the host still held the DDR settings. Native
SD open returned `0x218` (NoDevice), leaving the card off. The boot-handoff
build restored power once and trained another 17-tap window at 199.68 MHz.
Report `000000000058a4a8` failed on another low reading after 101 reads and 36
writes. The report captured no data command failures.

Atmosphère applies a late pinmux configuration that clears input-enable on
DMIC3_CLK, the SD power GPIO pad. A low input reading can then coexist with a
high GPIO output. Check the driven output in that configuration and capture
both values, output-enable, GPIO mode and card detect in the trace. If power
really drops, allow up to two early cold recoveries within ten seconds of
initial setup. Native SD open permanently closes direct I²C5 access. Later
power recovery uses native FS and read retries retune in place. HOC clock and
SysFanControl startup flags remain under `ddr200-debug/previous-fusee-stall-trace`.

The power-pin build reached HOME on the Samsung PRO Plus. The SD card had no
new fatal report or saved benchmark, so the active clock and the cause of the
reported slowdown are still unknown. This build retains that clock and recovery
policy. Its trace records the last sampled data command clock and controller
registers, plus command size, elapsed time, failures and the share transferred
through a DMA bounce buffer. SD command totals include emulated NAND I/O.
Separate SD filesystem request totals include mutex wait and retry time.

The speed trace reproduced five CMD18 data CRC errors at 199.68 MHz with
and without sys-clk. Its latest full-rate window narrowed from 17 to 15
passing taps with sys-clk. Both runs spent roughly 99% of write command time
waiting for DMA. Both finished at 166.40 MHz in DDR200 mode.

This forced build uses 199.68 MHz for data transfers, including after
reassertion. If a sweep or card status check fails, it restores the SD pad
configuration, recalibrates it, and sweeps again at 199.68 MHz.
After a read CRC or data end-bit error, it retunes the powered
card once and repeats the original DMA read. If that still fails with a
timing error, it tries the center of a second passing window, when one exists,
and two interior taps of the largest window. It keeps the tap that passes the
actual read. Windows still need eight passing taps.

Report `0000000000aa0640` shows that the second reassertion found a 16-tap
window at 199.68 MHz. A subsequent 32-block CMD18 failed at tap 36 with a
data CRC error. The controller marked Auto CMD12 as unexecuted. Both
recovery sweeps measured seven passing taps, including the sweep after
host pad recalibration. The driver stopped before retrying with another tap.

When full-rate host recovery still fails after a read timing error, this
build can close and reopen the card through Horizon's native SD accessor.
It checks the existing DMA mappings and the card CID, restores DDR200 at
199.68 MHz, and retries the original read. It allows one such reopen per
native handoff. A failed recovery saves a report. The report includes the
last eight sweeps, the failing tap and Auto CMD12 status, and the number of
native reopens for failed reads.

Use **More Configs → Fusee DDR200 card reopen (1.11.2)**. The entry chainloads the
official Atmosphère 1.11.2 `fusee.bin`. Its matching `package3` supplies the
kernel and secure monitor. Fusee loads the uncompressed `atmosphere/emummc.kip`.
Reports are saved to `atmosphere/fatal_errors`. A failed operation requests a
report automatically.

Run the same benchmark and record its read and write speeds. Once the benchmark
finishes, exit the app. Hold Volume + and Volume − together for three seconds
to intentionally open the Atmosphère report screen and save the driver trace.
Return to Hekate SD UMS to collect the report. The totals include activity since
boot. On a separate boot, sleep and wake the console, run the same benchmark
again, and capture another report to check the clock and tuning after resume.

Decode this build's report with `python3 tools/read-ddr-trace.py report_*.bin`
using a report from the Fusee test. `DD200062` means DDR200 recovery failed.
`DD200063` and `DD200064` identify read and write failure, `DD200065` is a manual
snapshot, `DD200066` is a clock lock or clock ownership error, `DD200067` is an
operation timeout and `DD200068` is a watchdog setup failure.

Host regression check for tuning and read recovery:

    clang -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
        -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
        -DEMUMMC_FORCE_DDR200 -DINNER_HEAP_SIZE=0x8000 tests/test_ddr200_recovery.c \
        -Wl,--gc-sections -o /tmp/test_ddr200_recovery
    UBSAN_OPTIONS=halt_on_error=1 /tmp/test_ddr200_recovery
