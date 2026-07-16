#!/usr/bin/env bash
# OpenEMS local CI — Stage1 (default): secrets + host/firmware WERROR.
# Lint stages (A/B) only when LINT_ERROR=1 after layering PRs.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

STAGE="${1:-1}"
export WERROR="${WERROR:-1}"
export LINT_ERROR="${LINT_ERROR:-0}"

echo "=== OpenEMS ci-local Stage${STAGE} (WERROR=${WERROR} LINT_ERROR=${LINT_ERROR}) ==="

make secrets-check

case "$STAGE" in
  1)
    # Post PR-11/12: ban ENGINE→app + engine regs allowlist (phase B)
    make host-test WERROR="$WERROR"
    make host-test-vgt6 WERROR="$WERROR"
    make firmware-rgt6 WERROR="$WERROR"
    make firmware-vgt6 WERROR="$WERROR"
    make lint-includes LINT_PHASE=A LINT_ERROR=1
    make lint-includes LINT_PHASE=B LINT_ERROR=1
    ;;
  2)
    make host-test WERROR="$WERROR"
    make host-test-vgt6 WERROR="$WERROR"
    make firmware-rgt6 WERROR="$WERROR"
    make firmware-vgt6 WERROR="$WERROR"
    make lint-includes LINT_PHASE=A LINT_ERROR=1
    make lint-includes LINT_PHASE=B LINT_ERROR=1
    ;;
  3)
    make host-test WERROR="$WERROR"
    make host-test-vgt6 WERROR="$WERROR"
    make firmware-rgt6 WERROR="$WERROR"
    make firmware-vgt6 WERROR="$WERROR"
    make lint-includes LINT_PHASE=A LINT_ERROR=1
    make lint-includes LINT_PHASE=B LINT_ERROR=1
    ;;
  *)
    echo "Usage: $0 [1|2|3]" >&2
    exit 2
    ;;
esac

echo "=== ci-local Stage${STAGE}: OK ==="
