#!/usr/bin/env bash
# Full PCB scaffold rebuild (does not regenerate schematics).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 scripts/build_pcb_placement.py
echo "Done. Open: $ROOT/openems_interface_v1.kicad_pro"
echo "Then: Pcbnew → Edit → Fill All Zones → Inspect → DRC"
