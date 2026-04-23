#!/usr/bin/env bash
# OpenEMS-v3 Host-Based Unit Test Runner
# Runs all 17 test suites for the active v3 project (openems-v3/)
# Legacy v1.1 and v2.2 tests are NOT run (reference versions only)

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
V3_DIR="${ROOT_DIR}/openems-v3"
BUILD_DIR="${TMPDIR:-/tmp}/openems_host_tests"
CXX_BIN="${CXX:-g++}"
CXXFLAGS=(-std=c++17 -DEMS_HOST_TEST -I"${V3_DIR}/src")

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

echo "Running OpenEMS-v3 host tests (17 suites) with ${CXX_BIN}"

run_test test_ckp \
  openems-v3/test/drv/test_ckp.cpp \
  openems-v3/src/drv/ckp.cpp

run_test test_sensors \
  openems-v3/test/drv/test_sensors.cpp \
  openems-v3/src/drv/sensors.cpp \
  openems-v3/src/hal/adc.cpp

run_test test_sensors_validation \
  openems-v3/test/drv/test_sensors_validation.cpp \
  openems-v3/src/drv/sensors.cpp \
  openems-v3/src/hal/adc.cpp

run_test test_ftm_arithmetic \
  openems-v3/test/hal/test_ftm_arithmetic.cpp

run_test test_fuel \
  openems-v3/test/engine/test_fuel.cpp \
  openems-v3/src/engine/fuel_calc.cpp \
  openems-v3/src/engine/table3d.cpp

# test_fuel_calc_assertions intentionally exercises out-of-range paths that
# assert in debug builds; run in release mode to validate clamping behavior.
echo ""
echo "==> [test_fuel_calc_assertions] build"
"${CXX_BIN}" -std=c++17 -DEMS_HOST_TEST -DNDEBUG -I"${V3_DIR}/src" \
  openems-v3/test/engine/test_fuel_calc_assertions.cpp \
  openems-v3/src/engine/fuel_calc.cpp \
  openems-v3/src/engine/table3d.cpp \
  -o "${BUILD_DIR}/test_fuel_calc_assertions"
echo "==> [test_fuel_calc_assertions] run"
"${BUILD_DIR}/test_fuel_calc_assertions"

run_test test_ign \
  openems-v3/test/engine/test_ign.cpp \
  openems-v3/src/engine/ign_calc.cpp \
  openems-v3/src/engine/table3d.cpp

run_test test_quick_crank \
  openems-v3/test/engine/test_quick_crank.cpp \
  openems-v3/src/engine/quick_crank.cpp

run_test test_auxiliaries \
  openems-v3/test/engine/test_auxiliaries.cpp \
  openems-v3/src/engine/auxiliaries.cpp \
  openems-v3/src/engine/table3d.cpp

run_test test_iacv \
  openems-v3/test/engine/test_iacv.cpp \
  openems-v3/src/engine/auxiliaries.cpp \
  openems-v3/src/engine/table3d.cpp

run_test test_knock \
  openems-v3/test/engine/test_knock.cpp \
  openems-v3/src/engine/knock.cpp \
  openems-v3/src/hal/flexnvm.cpp

run_test test_ts_protocol \
  openems-v3/test/app/test_ts_protocol.cpp \
  openems-v3/test/app/stub_ecu_sched_ivc.cpp \
  openems-v3/src/app/tuner_studio.cpp \
  openems-v3/src/app/can_stack.cpp \
  openems-v3/src/hal/can.cpp \
  openems-v3/src/engine/fuel_calc.cpp \
  openems-v3/src/engine/ign_calc.cpp \
  openems-v3/src/engine/table3d.cpp

run_test test_can \
  openems-v3/test/app/test_can.cpp \
  openems-v3/src/app/can_stack.cpp \
  openems-v3/src/hal/can.cpp

run_test test_flexnvm \
  openems-v3/test/hal/test_flexnvm.cpp \
  openems-v3/src/hal/flexnvm.cpp

# test_ecu_sched: compiled as C++ (was MISRA-C module)
echo ""
echo "==> [test_ecu_sched] build"
g++ -std=c++17 -DEMS_HOST_TEST -I"${V3_DIR}/src" \
  openems-v3/test/engine/test_ecu_sched.c \
  openems-v3/src/engine/ecu_sched.cpp \
  -o "${BUILD_DIR}/test_ecu_sched"
echo "==> [test_ecu_sched] run"
"${BUILD_DIR}/test_ecu_sched"

# Additional ecu_sched regression suite
echo ""
echo "==> [test_ecu_sched_fixes] build"
g++ -std=c++17 -DEMS_HOST_TEST -I"${V3_DIR}/src" \
  openems-v3/test/engine/test_ecu_sched_fixes.cpp \
  openems-v3/src/engine/ecu_sched.cpp \
  -o "${BUILD_DIR}/test_ecu_sched_fixes"
echo "==> [test_ecu_sched_fixes] run"
"${BUILD_DIR}/test_ecu_sched_fixes"

echo ""
echo "==> [test_pipeline_backbone] build"
g++ -std=c++17 -DEMS_HOST_TEST -I"${V3_DIR}/src" \
  openems-v3/test/engine/test_pipeline_backbone.cpp \
  openems-v3/src/drv/ckp.cpp \
  openems-v3/src/engine/cycle_sched.cpp \
  openems-v3/src/engine/ecu_sched.cpp \
  openems-v3/src/engine/fuel_calc.cpp \
  openems-v3/src/engine/ign_calc.cpp \
  openems-v3/src/engine/table3d.cpp \
  -o "${BUILD_DIR}/test_pipeline_backbone"
echo "==> [test_pipeline_backbone] run"
"${BUILD_DIR}/test_pipeline_backbone"

echo ""
echo "All host tests passed."
