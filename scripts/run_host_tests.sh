#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${TMPDIR:-/tmp}/openems_host_tests"
CXX_BIN="${CXX:-g++}"
CXXFLAGS=(-std=c++17 -DEMS_HOST_TEST -Isrc)

mkdir -p "${BUILD_DIR}"

run_test() {
  local name="$1"
  shift
  local exe="${BUILD_DIR}/${name}"
  echo ""
  echo "==> [${name}] build"
  "${CXX_BIN}" "${CXXFLAGS[@]}" "$@" -o "${exe}"
  echo "==> [${name}] run"
  "${exe}"
}

cd "${ROOT_DIR}"

echo "Running all host tests with ${CXX_BIN}"

run_test test_ckp \
  test/drv/test_ckp.cpp \
  src/drv/ckp.cpp

run_test test_scheduler \
  test/drv/test_scheduler.cpp \
  src/drv/scheduler.cpp

run_test test_sensors \
  test/drv/test_sensors.cpp \
  src/drv/sensors.cpp \
  src/hal/adc.cpp

run_test test_ftm_arithmetic \
  test/hal/test_ftm_arithmetic.cpp

run_test test_fuel \
  test/engine/test_fuel.cpp \
  src/engine/fuel_calc.cpp \
  src/engine/table3d.cpp

run_test test_ign \
  test/engine/test_ign.cpp \
  src/engine/ign_calc.cpp \
  src/engine/table3d.cpp

run_test test_auxiliaries \
  test/engine/test_auxiliaries.cpp \
  src/engine/auxiliaries.cpp \
  src/engine/table3d.cpp

run_test test_iacv \
  test/engine/test_iacv.cpp \
  src/engine/auxiliaries.cpp \
  src/engine/table3d.cpp

run_test test_knock \
  test/engine/test_knock.cpp \
  src/engine/knock.cpp \
  src/hal/flexnvm.cpp

run_test test_ts_protocol \
  test/app/test_ts_protocol.cpp \
  src/app/tuner_studio.cpp \
  src/app/can_stack.cpp \
  src/hal/can.cpp \
  src/engine/fuel_calc.cpp \
  src/engine/ign_calc.cpp \
  src/engine/table3d.cpp

run_test test_can \
  test/app/test_can.cpp \
  src/app/can_stack.cpp \
  src/hal/can.cpp

run_test test_flexnvm \
  test/hal/test_flexnvm.cpp \
  src/hal/flexnvm.cpp

echo ""
echo "All host tests passed."
