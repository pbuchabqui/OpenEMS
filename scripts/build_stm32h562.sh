#!/usr/bin/env bash
# =============================================================================
# scripts/build_stm32h562.sh - valida o build STM32H562 ativo
#
# Este script e intencionalmente STM32-only. Ele nao compila caminhos antigos,
# nao executa testes de compatibilidade e nao aceita aliases de perifericos de
# outras plataformas.
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

echo "========================================================"
echo " OpenEMS - STM32H562 build validation"
echo " Root: ${ROOT_DIR}"
echo "========================================================"

echo ""
echo "-- Compilacao do firmware --"
make firmware

echo ""
echo "========================================================"
echo " STM32H562 build validation: PASSOU"
echo "========================================================"
