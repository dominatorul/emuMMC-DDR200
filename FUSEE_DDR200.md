# Fusee DDR200 diagnostic

This build starts from Lightos1 develop at `108f1632`. It retains the original
single manual sweep and reassert sequence. The test build requires a successful
DDR200 switch and a passing interval of at least eight taps. It captures both
the hardware comparator results and the subset with completed commands.

The last Hekate test, report `0000000000a49f0b`, reached DDR200, completed
3,097 reads and 73 writes, then failed during recovery with `DD202011`.
The center moved from tap 40 to 33 and the measured width dropped from 17 to 5.
The log showed changes in clock source, host mode, divider and tap.

Hekate and L4T use CMD6 vendor command-system function 14 with the Access Mode
group at SDR12. Native FS can configure that group during SD open. The new build
returns the host and card to low-clock SDR12 before reselecting DDR200 during
in-place recovery. Actual SD power, controller clock, voltage or width loss
forces a fresh DDR200 initialization. Recovery preserves the 512-byte DMA
scratch sector. The capture includes the original and latest sweeps, card CMD6
status, registers before and after native open, and the reassert entry state.

Use **More Configs → Fusee DDR200 debug (1.11.2)**. The entry chainloads the
official Atmosphère 1.11.2 `fusee.bin`. Its matching `package3` supplies the
kernel and secure monitor. Fusee loads the uncompressed `atmosphere/emummc.kip`.
Reports are saved to `atmosphere/fatal_errors`. A failed operation requests a
report automatically. Holding both volume buttons can request a report on
the next SD read. If HOME loads, test sleep and wake once. Console testing is
needed to verify DDR200 boot and resume.

Decode this build's report with `python3 tools/read-ddr-trace.py report_*.bin`
using a report from the Fusee test. `DD200062` means DDR200 recovery failed.
`DD200063` and `DD200064` identify read and write failure, `DD200065` is a manual
snapshot, and `DD200066` is a clock lock or clock ownership error.
