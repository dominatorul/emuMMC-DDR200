#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM
for variant in release matrix performance; do
    set --
    if [ "$variant" = matrix ]; then set -- -DEMUMMC_DDR200_MATRIX; fi
    if [ "$variant" = performance ]; then set -- -DEMUMMC_DDR200_PERF_TRIAL; fi
    "${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
        -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
        -DINNER_HEAP_SIZE=0x8000 "$@" tests/test_ddr200_recovery.c \
        -Wl,--gc-sections -o "$out/test_ddr200_recovery"
    ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_recovery"
done

"${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
    -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
    -DINNER_HEAP_SIZE=0x8000 tests/test_ddr200_lifecycle.c \
    -Wl,--gc-sections -o "$out/test_ddr200_lifecycle"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_lifecycle"

"${CC:-clang}" -std=gnu2x -O1 -g -fsanitize=address,undefined \
    -DEMUMMC_DDR200_READ_AHEAD tests/test_ddr200_read_ahead.c -o "$out/test_ddr200_read_ahead"
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_read_ahead"
