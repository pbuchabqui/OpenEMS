#!/usr/bin/env bash
# Rebuild clean PCB scaffold (placement-first, no spaghetti).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 scripts/layout_clean.py
echo ""
echo "Open:"
echo "  kicad $ROOT/openems_interface_v1.kicad_pro"
echo "Then PCB Editor → Home → Edit → Fill All Zones"
