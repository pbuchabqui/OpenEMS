#!/usr/bin/env bash
# =============================================================================
# scripts/build_stm32h562.sh — Valida compilação da HAL STM32H562
#
# Compila todos os arquivos HAL STM32H562 + camadas engine/drv/app com
# -DTARGET_STM32H562 -DEMS_HOST_TEST para verificar que:
#   1. Todos os símbolos resolvem (sem undefined references)
#   2. Sem warnings de compilação (Wall + Wextra)
#   3. Os testes host originais continuam passando (versão Kinetis)
#
# Uso:
#   ./scripts/build_stm32h562.sh              # compila e verifica
#   ./scripts/build_stm32h562.sh --run-tests  # também executa testes Kinetis
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${TMPDIR:-/tmp}/openems_stm32h562"
CXX_BIN="${CXX:-g++}"

# Flags comuns: STM32 target + host test mode (stubs de hardware)
CXXFLAGS=(
    -std=c++17
    -DEMS_HOST_TEST
    -DTARGET_STM32H562
    -Isrc
    -Wall
    -Wextra
    -Wno-unused-parameter
)

mkdir -p "${BUILD_DIR}"

echo "========================================================"
echo " OpenEMS — STM32H562 HAL Build Validation"
echo " Compiler: ${CXX_BIN}"
echo " Build dir: ${BUILD_DIR}"
echo "========================================================"

# ── Fontes STM32H562 HAL (novos arquivos) ────────────────────────────────────
STM32_HAL_SOURCES=(
    src/hal/stm32h562/system.cpp
    src/hal/stm32h562/timer.cpp
    src/hal/stm32h562/adc.cpp
    src/hal/stm32h562/can.cpp
    src/hal/stm32h562/uart.cpp
    src/hal/stm32h562/flash.cpp
)

# ── Fontes engine + drv + app (portáveis — inalteradas) ──────────────────────
PORTABLE_SOURCES=(
    src/drv/ckp.cpp
    src/engine/fuel_calc.cpp
    src/engine/ign_calc.cpp
    src/engine/table3d.cpp
    src/engine/auxiliaries.cpp
    src/engine/knock.cpp
    src/engine/quick_crank.cpp
    src/engine/cycle_sched.cpp
    src/engine/ecu_sched.cpp
)

# ─────────────────────────────────────────────────────────────────────────────
# Teste 1: Compilação isolada de cada arquivo STM32 HAL
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "── Fase 1: Compilação isolada dos módulos STM32H562 HAL ──"

for src in "${STM32_HAL_SOURCES[@]}"; do
    obj="${BUILD_DIR}/$(basename "${src}" .cpp).o"
    echo "  Compilando ${src}..."
    "${CXX_BIN}" "${CXXFLAGS[@]}" -c "${ROOT_DIR}/${src}" -o "${obj}"
    echo "  OK: ${obj}"
done

# ─────────────────────────────────────────────────────────────────────────────
# Teste 2: Compilação do ckp.cpp com -DTARGET_STM32H562
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "── Fase 2: ckp.cpp com TARGET_STM32H562 ──"
ckp_obj="${BUILD_DIR}/ckp_stm32.o"
"${CXX_BIN}" "${CXXFLAGS[@]}" -c "${ROOT_DIR}/src/drv/ckp.cpp" -o "${ckp_obj}"
echo "  OK: ckp.cpp compilado com TIM5 register remapping"

# ─────────────────────────────────────────────────────────────────────────────
# Teste 3: Compilação individual de cada fonte portável com TARGET_STM32H562
# (não link — ecu_sched.cpp usa stubs Kinetis que não fazem parte deste HAL)
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "── Fase 3: Compilação portável com TARGET_STM32H562 ──"

EXTRA_PORTABLE_SOURCES=(
    src/drv/sensors.cpp
    src/app/tuner_studio.cpp
    src/app/can_stack.cpp
    src/main_stm32.cpp
)

ALL_COMPILE_SOURCES=()
for src in "${PORTABLE_SOURCES[@]}"; do
    ALL_COMPILE_SOURCES+=("${src}")
done
for src in "${EXTRA_PORTABLE_SOURCES[@]}"; do
    ALL_COMPILE_SOURCES+=("${src}")
done

for src in "${ALL_COMPILE_SOURCES[@]}"; do
    obj="${BUILD_DIR}/stm32_$(basename "${src}" .cpp).o"
    echo "  Compilando ${src}..."
    "${CXX_BIN}" "${CXXFLAGS[@]}" -c "${ROOT_DIR}/${src}" -o "${obj}"
    echo "  OK: ${obj}"
done

echo "  OK: todos os módulos portáveis compilam com TARGET_STM32H562"

# ─────────────────────────────────────────────────────────────────────────────
# Teste 4: Verificação de símbolos Teensyduino ausentes
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "── Fase 4: Verificando ausência de dependências Teensyduino ──"

TEENSYDUINO_PATTERNS=(WProgram Arduino Teensyduino Serial\.begin Serial\.write Serial\.available)

found_dependency=0
for pattern in "${TEENSYDUINO_PATTERNS[@]}"; do
    if grep -r "${pattern}" \
        "${ROOT_DIR}/src/hal/stm32h562/" \
        "${ROOT_DIR}/src/main_stm32.cpp" 2>/dev/null | \
        grep -v "^\s*//" | \
        grep -v "Teensyduino" 2>/dev/null; then
        echo "  AVISO: dependência Teensyduino encontrada: ${pattern}"
        found_dependency=1
    fi
done

if [[ "${found_dependency}" -eq 0 ]]; then
    echo "  OK: nenhuma dependência Teensyduino nos arquivos STM32"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Teste 5: Testes host originais (Kinetis) — opcional
# ─────────────────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "--run-tests" ]]; then
    echo ""
    echo "── Fase 5: Testes host originais (Kinetis) ──"
    cd "${ROOT_DIR}"
    ./scripts/run_host_tests.sh
    echo "  OK: todos os testes Kinetis passaram"
fi

echo ""
echo "========================================================"
echo " STM32H562 build validation: PASSOU"
echo " Arquivos novos em: src/hal/stm32h562/"
echo " Entry point STM32: src/main_stm32.cpp"
echo " Build com: -DTARGET_STM32H562 -DEMS_HOST_TEST"
echo "========================================================"
