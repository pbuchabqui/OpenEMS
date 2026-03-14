#!/usr/bin/env bash
# build_stm32h5.sh — Compile-only validation for OpenEMS STM32H562 standalone port
# Uses arm-none-eabi-g++ if available, otherwise skips firmware compilation but
# still validates host-test compilation to confirm no include path issues.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${TMPDIR:-/tmp}/openems_stm32h5_build"
mkdir -p "${BUILD_DIR}"

cd "${ROOT_DIR}"

# ── Source files for STM32H562 firmware ──────────────────────────────────────
STM32_SRCS=(
    src/main.cpp
    src/hal/system.cpp
    src/hal/timer.cpp
    src/hal/adc.cpp
    src/hal/can.cpp
    src/hal/uart.cpp
    src/hal/flexnvm.cpp
    src/drv/ckp.cpp
    src/drv/sensors.cpp
    src/engine/fuel_calc.cpp
    src/engine/ign_calc.cpp
    src/engine/table3d.cpp
    src/engine/auxiliaries.cpp
    src/engine/ecu_sched.cpp
    src/engine/cycle_sched.cpp
    src/engine/knock.cpp
    src/engine/quick_crank.cpp
    src/app/tuner_studio.cpp
    src/app/can_stack.cpp
)

COMMON_FLAGS=(
    -std=c++17
    -DTARGET_STM32H562
    -Isrc
    -Wall
    -Wextra
    -fno-exceptions
    -fno-rtti
)

# ── Try cross-compiler first ──────────────────────────────────────────────────
ARM_CXX="${ARM_CXX:-arm-none-eabi-g++}"

if command -v "${ARM_CXX}" &>/dev/null; then
    echo "==> [build_stm32h5] cross-compile with ${ARM_CXX}"
    "${ARM_CXX}" \
        "${COMMON_FLAGS[@]}" \
        -mcpu=cortex-m33 \
        -mthumb \
        -mfpu=fpv5-sp-d16 \
        -mfloat-abi=hard \
        -O2 \
        -c \
        "${STM32_SRCS[@]}" \
        -o /dev/null 2>&1 || {
        echo "[WARN] arm-none-eabi-g++ compile failed — check output above"
        exit 1
    }
    echo "[PASS] STM32H562 firmware compile-check passed (arm-none-eabi-g++)"
else
    echo "[INFO] ${ARM_CXX} not found — skipping cross-compile"
    echo "[INFO] Running host-compiler syntax check instead (g++ -DEMS_HOST_TEST)"

    CXX_BIN="${CXX:-g++}"
    echo "==> [build_stm32h5] syntax check with ${CXX_BIN}"
    "${CXX_BIN}" \
        -std=c++17 \
        -DEMS_HOST_TEST \
        -Isrc \
        -Wall -Wextra \
        -fsyntax-only \
        "${STM32_SRCS[@]}"
    echo "[PASS] Syntax check passed (EMS_HOST_TEST mode)"
fi

echo ""
echo "build_stm32h5.sh: all checks passed."
