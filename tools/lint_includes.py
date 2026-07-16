#!/usr/bin/env python3
"""Layer include lint for OpenEMS (hygiene program).

Phase A: ban #include \"app/ in src/engine and src/drv
Phase B: ban #include \"hal/regs.h in src/engine (allowlist file optional)

Exit 0 always if LINT_ERROR is 0 (warn-only); non-zero if errors and LINT_ERROR=1.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_INC = re.compile(r'#\s*include\s+"app/')
REGS_INC = re.compile(r'#\s*include\s+"hal/regs\.h"')


def scan(paths: list[Path], pattern: re.Pattern[str]) -> list[tuple[Path, int, str]]:
    hits: list[tuple[Path, int, str]] = []
    for p in paths:
        try:
            text = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            if pattern.search(line):
                hits.append((p, i, line.strip()))
    return hits


def load_allowlist(path: Path | None) -> set[str]:
    if path is None or not path.is_file():
        return set()
    out: set[str] = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        out.add(s)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", choices=("A", "B"), default="A")
    ap.add_argument(
        "--error",
        action="store_true",
        help="exit 1 on hits (or set LINT_ERROR=1)",
    )
    ap.add_argument(
        "--allowlist",
        type=Path,
        default=ROOT / "tools" / "include_allowlist.txt",
    )
    args = ap.parse_args()
    as_error = args.error or os.environ.get("LINT_ERROR", "0") == "1"

    if args.phase == "A":
        files = list((ROOT / "src" / "engine").rglob("*.[ch]pp")) + list(
            (ROOT / "src" / "engine").rglob("*.h")
        )
        files += list((ROOT / "src" / "drv").rglob("*.[ch]pp")) + list(
            (ROOT / "src" / "drv").rglob("*.h")
        )
        hits = scan(files, APP_INC)
        label = 'ENGINE/DRV → #include "app/'
    else:
        # Phase B: engine only for regs.h
        files = list((ROOT / "src" / "engine").rglob("*.[ch]pp")) + list(
            (ROOT / "src" / "engine").rglob("*.h")
        )
        allow = load_allowlist(args.allowlist)
        raw = scan(files, REGS_INC)
        hits = [(p, i, l) for p, i, l in raw if str(p.relative_to(ROOT)) not in allow]
        label = 'ENGINE → #include "hal/regs.h" (allowlist-filtered)'

    if not hits:
        print(f"lint-includes phase {args.phase}: OK ({label})")
        return 0

    print(f"lint-includes phase {args.phase}: {len(hits)} hit(s) — {label}", file=sys.stderr)
    for p, i, line in hits:
        print(f"  {p.relative_to(ROOT)}:{i}: {line}", file=sys.stderr)

    if as_error:
        return 1
    print("(warn-only; set LINT_ERROR=1 to fail)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
