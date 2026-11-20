#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM
for variant in release matrix performance drive-c counted-reads adaptive hekate; do
    printf "Testing %s\n" "$variant"
    set --
    if [ "$variant" = matrix ]; then set -- -DEMUMMC_DDR200_MATRIX; fi
    if [ "$variant" = performance ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL; fi
    if [ "$variant" = drive-c ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL -DEMUMMC_DDR200_DRIVE_C; fi
    if [ "$variant" = counted-reads ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL -DEMUMMC_DDR200_COUNTED_READS; fi
    if [ "$variant" = adaptive ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL -DEMUMMC_DDR200_COUNTED_READS -DEMUMMC_DDR200_ADMA_PROBE -DEMUMMC_DDR200_READ_AHEAD; fi
    if [ "$variant" = hekate ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL -DEMUMMC_DDR200_HEKATE -DEMUMMC_DDR200_ADMA_PROBE -DEMUMMC_DDR200_READ_AHEAD; fi
    "${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
        -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
        -DINNER_HEAP_SIZE=0x8000 "$@" tests/test_ddr200_recovery.c \
        -Wl,--gc-sections -o "$out/test_ddr200_recovery"
    ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_recovery"
done

"${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
    -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
    -DINNER_HEAP_SIZE=0x8000 -DEMUMMC_DDR200_READ_AHEAD -DEMUMMC_DDR200_ADMA_PROBE \
    tests/test_ddr200_lifecycle.c \
    -Wl,--gc-sections -o "$out/test_ddr200_lifecycle"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_lifecycle"

"${CC:-clang}" -std=gnu2x -O1 -g -fsanitize=address,undefined \
    -DEMUMMC_DDR200_READ_AHEAD tests/test_ddr200_read_ahead.c -o "$out/test_ddr200_read_ahead"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_read_ahead"

for profile in adaptive hekate; do
    set --
    if [ "$profile" = hekate ]; then set -- -DEMUMMC_DDR200_HEKATE; fi
    "${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
        -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
        -DEMUMMC_DDR200_READ_AHEAD -DEMUMMC_DDR200_ADMA_PROBE "$@" tests/test_ddr200_adma.c \
        -Wl,--gc-sections -o "$out/test_ddr200_adma"
    ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_adma"
done

"${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
    -fsanitize=address,undefined -DEMUMMC_DDR200_READ_AHEAD -DEMUMMC_DDR200_ADMA_PROBE \
    -DEMUMMC_DDR200_PERF_TRIAL tests/test_ddr200_trace.c \
    -Wl,--gc-sections -o "$out/test_ddr200_trace"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_trace" > "$out/trace.bin"
python3 tools/read-ddr-trace.py "$out/trace.bin" > "$out/trace.txt"
grep -F 'Read-ahead cache maintenance per 2 MiB DMA: 19 us before, 21 us after' "$out/trace.txt"
grep -F 'Read-ahead cooldown: 2/3 strikes, 1 re-enables' "$out/trace.txt"
