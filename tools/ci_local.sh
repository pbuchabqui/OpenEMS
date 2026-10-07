#!/usr/bin/env bash
# OpenEMS local CI: secrets, host tests, virtual-engine precision, TunerStudio
# ini, firmware for both boards (WERROR), include layering lint.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

export WERROR="${WERROR:-1}"
echo "=== OpenEMS ci-local (WERROR=${WERROR}) ==="

make secrets-check
make host-test WERROR="$WERROR"
make host-test-vgt6 WERROR="$WERROR"
make host-test-knock-hw WERROR="$WERROR"
make precision-test
make ini-check
make firmware-rgt6 WERROR="$WERROR"
make firmware-vgt6 WERROR="$WERROR"
make lint-includes LINT_PHASE=A LINT_ERROR=1
make lint-includes LINT_PHASE=B LINT_ERROR=1

echo "=== ci-local: OK ==="
