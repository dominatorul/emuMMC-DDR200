# For Lightos1

Report `14a3626e` captured a 2 MiB CMD18 at 199.68 MHz on tap 32 in the
29–36 window. SDMA hit a CRC error after its first software boundary restart
with 3,078 blocks left. The same sectors passed via ADMA2 at taps 32 and
33 with zero restarts. Across `14a3626e` and `a165f63`, 158 ADMA2 fills
passed and command throughput was 150–152 MB/s. These measurements isolate
a failure seen with the SDMA restart path. The new build uses ADMA2 for
ordinary SD data transfers as well, so the next trace can show any errors
that persist when the driver no longer restarts SDMA at 512 KiB boundaries.
