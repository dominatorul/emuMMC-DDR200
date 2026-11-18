# DDR200 release and matrix package

The target is the Samsung PRO Plus on HOS 21.2.0 with Atmosphère 1.11.2.
The driver tracks DDR200 at 199.68 MHz and 166.40 MHz. The release profile
uses official Fusee. The matrix and known-good payloads use its 1.11.2
program with a different emuMMC KIP filename. Each uses the same package3
and emuMMC configuration.

## Runtime policy

Initial DDR200 qualification requires an eight-tap window. After it qualifies,
the driver sweeps within 16 taps of the previous eye's center after native
handoff, 64 MiB of I/O, or 500 ms of busy time. The intersection of up to
three measured windows supplies candidate taps. Full-rate runtime tuning
prefers the low side to leave room for a collapsing right edge.

A five- or six-tap full-rate eye can continue with two taps of margin if two
256 KiB reads at distinct passing taps agree. The driver rechecks these
narrow windows every 32 MiB or 250 ms of busy time. Wider eyes and reduced
DDR200 use a three-tap guard. Failed verification triggers normal fallback.

The release clock uses PLLC4_OUT0 at 998.4 MHz, the SDMMC1 source divider
of 2.5, and the SDHCI divider of 2, giving 199.68 MHz. The experimental
**Fusee DDR200 (204 MHz PLLP trial)** entry uses an existing locked 408 MHz
PLLP clock and SDHCI divide by 2. This changes the controller's mux and
divider and lets a console test compare the two sources.

The driver first repairs a failed data command with explicit CMD12, waits
for CMD13 transfer state and READY_FOR_DATA, and resets CMD/DAT. Read recovery
tries new taps at the current clock, then validated DDR200 at 166.40 MHz,
then standard SDR104 timing. SDR104 uses the serialized emuMMC transport.
An ambiguous protocol switch or failed stop returns a storage error. A failed
write can have programmed some blocks, so the caller receives its error after
link recovery. At a degraded rate the driver schedules up to six full-rate
retries with a 5–300 second backoff after idle or enough traffic. The trace
marks the boot as unstable if it needed to leave full rate.

After the boot power window closes, Horizon owns SD power. At the native
eMMC close, the driver verifies SDR104 before handing back the SD link and
runs the paired SD close with the emuMMC locks released. On a cold access
it checks the power GPIO and pinmux and yields the locks while Horizon
restores power. Wake can restore the host's SDR104 setup while the card
retains DDR200. The driver checks for an SDR104 CMD6 reply. If it cannot
confirm SDR104 it verifies the retained link at 166.40 MHz before attempting
to restore the previous DDR200 rate. Both wake paths preserve DMA scratch.

## Matrix

The diagnostic boots with the release runtime policy. Its first step runs
on a normal SD filesystem request at least 20 seconds after the first
successful native DDR200 handoff. Upcoming requests advance the experiment.
Hold Volume Down by itself for three seconds to skip or end it.
It compares five repeated scans per clock, gating
between tap samples versus leaving the clock on, and the 0, 10 and 100 ms
settling targets after a confirmed DDR200 switch. The report measures each
settling scan's start relative to the switch. Later scans may start after
their target as earlier scans finish.

The diagnostic reads 64 MiB in 2 MiB slices at each clock, sweeps every
8 MiB and again after idle, and times up to 256 MiB of sequential reads at
each clock. It logs the card's driver strength and power limits from CMD6
and tests advertised standard driver strengths. It runs the CMD6 changes
after the read comparisons. Each step uses the caller's storage locks and a
250 ms command budget. The next request after an idle interval captures the
idle sweep. The report identifies shortened load tests and idle
periods if the system keeps using the card. Matching reads protect the
normal profile while the experiment changes clocks.

The trace holds 40 experimental records and eight recent runtime records in
matrix mode. The release profile uses all 48 records for runtime events. Each
sweep includes the sampled eye, applied tap, elapsed microseconds, bytes
transferred since reassertion, rate and driver strength. It also records
actual clock registers, CMD12/CMD13 cleanup, profile transitions and timed
throughput. The logs can show whether contraction correlates with load and
whether the eye recovers after idle. A physical measurement is needed to
attribute a reversible change to heat or rail droop.

## Console test

1. Boot **Fusee DDR200 matrix (diagnostic)** from Hekate More Configs. Wait
   30 seconds at HOME, then run SDBench to drive the matrix.
2. Hold **Volume + and Volume −** together for three seconds, then reconnect
   in SD UMS. Collect the new file from `atmosphere/fatal_errors`.
3. Boot **Fusee DDR200 (release policy)** and record the read and write
   benchmark results.
4. Run three sleep/wake cycles and benchmark after each. Save another report
   with both volume buttons and reconnect SD UMS.

The manual report opens the Atmosphère fatal screen. The **Fusee DDR200
(known-good power pin)** entry uses the earlier HOME-booting build.

Use `python3 tools/read-ddr-trace.py report_*.bin` to decode the reports.
Use `sh tests/run.sh` for the release and matrix host checks with ASan and
UBSan. Host checks simulate eye drift, two-tap reads, command cleanup and the
matrix schedule. They exercise three native suspend/wake pairs and a wake
where Horizon resets only the host. Console verification is pending.
