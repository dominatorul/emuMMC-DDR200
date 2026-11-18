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

The driver tries 199.68 MHz first. If the sweep or card status check fails, it
repeats tuning with the original fork's 166.4 MHz clock setting in the same
vendor DDR200 mode. A read that repeatedly fails during command response or
transfer can retry once after retraining the powered card in place. A successful
reduced clock setting is retained through
subsequent reopens and resume for that boot. Both clocks require eight passing
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
PTM and OMM crashes share that boot. The forced driver now releases its raw
boot I²C5 interface as soon as it finishes initial SD setup. Native FS handles
later power recovery so its regulator transactions go through Horizon's service.
The read retry changes the SD sampling clock without cycling the PMIC. HOC clock
and SysFanControl startup flags are saved under `ddr200-debug/previous-fusee-stall-trace`
for this test to isolate the power and clock services.

Use **More Configs → Fusee DDR200 PCV test (1.11.2)**. The entry chainloads the
official Atmosphère 1.11.2 `fusee.bin`. Its matching `package3` supplies the
kernel and secure monitor. Fusee loads the uncompressed `atmosphere/emummc.kip`.
Reports are saved to `atmosphere/fatal_errors`. A failed operation requests a
report automatically. If the logo still hangs, release both volume buttons and
then hold them together for three seconds to capture a report independently of
SD I/O. If HOME loads, test sleep and wake once. Console testing is
needed to verify DDR200 boot and resume.

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
