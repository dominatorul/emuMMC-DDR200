#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM
for variant in release matrix; do
    set --
    if [ "$variant" = matrix ]; then set -- -DEMUMMC_DDR200_MATRIX; fi
    "${CC:-clang}" -std=gnu2x -O1 -g -ffunction-sections -fdata-sections \
        -fsanitize=address,undefined -DEMUMMC_SDMMC_UHS_DDR200_SUPPORT \
        -DINNER_HEAP_SIZE=0x8000 "$@" tests/test_ddr200_recovery.c \
        -Wl,--gc-sections -o "$out/test_ddr200_recovery"
    ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "$out/test_ddr200_recovery"
done
