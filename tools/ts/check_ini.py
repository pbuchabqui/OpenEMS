#!/usr/bin/env python3
"""Static checks for tools/ts/openems.ini (no TunerStudio needed).

- every dialog field / curve / table / gauge / datalog entry names a defined
  constant or output channel;
- no two constants of the same page overlap in bytes;
- page 0 engine-config ranges match the firmware limits (engine_config.cpp).
Exit status 1 on any finding.
"""
import re
import sys

INI = sys.argv[1] if len(sys.argv) > 1 else "tools/ts/openems.ini"
SIZES = {"U08": 1, "S08": 1, "U16": 2, "S16": 2, "U32": 4, "S32": 4, "F32": 4}

# Firmware limits (raw units) — must match kFields in engine_config.cpp.
FW_LIMITS = {
    "displacement": (200, 10000),
    "injectorFlow": (50, 3000),
    "stoichAfr": (900, 1800),
    "mapRef": (50, 250),
    "triggerOffset": (0, 719),
    "eoiLeadDeg": (0, 719),
}

errors = []
consts, chans, cmds = {}, set(), set()
section, page = None, None
for raw in open(INI, encoding="utf-8"):
    line = raw.split(";", 1)[0].strip()
    if not line:
        continue
    m = re.match(r"^\[(\w+)\]", line)
    if m:
        section = m.group(1)
        continue
    if section == "Constants":
        m = re.match(r"^page\s*=\s*(\d+)", line)
        if m:
            page = int(m.group(1))
            continue
        m = re.match(r"^(\w+)\s*=\s*(scalar|array|bits)\s*,\s*(\w+)\s*,\s*(\d+)\s*,(.*)$", line)
        if m:
            name, kind, typ, off, rest = m.groups()
            off = int(off)
            n = 1
            if kind == "array":
                dims = re.search(r"\[(\d+)(?:x(\d+))?\]", rest)
                n = int(dims.group(1)) * int(dims.group(2) or 1)
            size = SIZES[typ] * n if kind != "bits" else SIZES[typ]
            parts = [p.strip() for p in rest.split(",")]
            consts[name] = (page, off, size, kind, typ, parts)
    elif section == "OutputChannels":
        m = re.match(r"^(\w+)\s*=", line)
        if m:
            chans.add(m.group(1))
    elif section == "ControllerCommands":
        m = re.match(r'^(\w+)\s*=\s*"(.*)"', line)
        if m:
            cmds.add(m.group(1))
            body = re.sub(r"\\x[0-9A-Fa-f]{2}", "#", m.group(2))
            if len(body) != 5 or not body.startswith("T"):
                errors.append(f"{m.group(1)}: output-test command must be 'T' + 4 bytes")

names = set(consts) | chans
section = None
for n, raw in enumerate(open(INI, encoding="utf-8"), 1):
    line = raw.split(";", 1)[0].strip()
    m = re.match(r"^\[(\w+)\]", line)
    if m:
        section = m.group(1)
        continue
    refs = []
    if section == "UserDefined":
        m = re.match(r'^field\s*=\s*"[^"]*"\s*,\s*(\w+)', line)
        if m:
            refs.append(m.group(1))
        m = re.match(r'^commandButton\s*=\s*"[^"]*"\s*,\s*(\w+)', line)
        if m and m.group(1) not in cmds:
            errors.append(f"line {n}: command '{m.group(1)}' is not in [ControllerCommands]")
    elif section in ("CurveEditor", "TableEditor"):
        m = re.match(r"^(xBins|yBins|zBins)\s*=\s*(\w+)(?:\s*,\s*(\w+))?", line)
        if m:
            refs += [r for r in m.groups()[1:] if r]
    elif section == "GaugeConfigurations":
        m = re.match(r"^\w+\s*=\s*(\w+)\s*,", line)
        if m and not line.startswith("gaugeCategory"):
            refs.append(m.group(1))
    elif section == "Datalog":
        m = re.match(r"^entry\s*=\s*(\w+)", line)
        if m:
            refs.append(m.group(1))
    for r in refs:
        if r not in names:
            errors.append(f"line {n}: '{r}' is not a defined constant / channel")

by_page = {}
for name, (pg, off, size, kind, typ, parts) in consts.items():
    if kind == "bits":
        continue
    by_page.setdefault(pg, []).append((off, off + size, name))
for pg, items in by_page.items():
    items.sort()
    for (a0, a1, an), (b0, b1, bn) in zip(items, items[1:]):
        if b0 < a1:
            errors.append(f"page {pg}: {an} [{a0},{a1}) overlaps {bn} [{b0},{b1})")

for name, (lo, hi) in FW_LIMITS.items():
    if name not in consts:
        errors.append(f"{name}: missing")
        continue
    parts = consts[name][5]
    # scalar: units, scale, translate, lo, hi, digits
    scale, lo_ini, hi_ini = float(parts[1]), float(parts[3]), float(parts[4])
    got = (round(lo_ini / scale), round(hi_ini / scale))
    if got != (lo, hi):
        errors.append(f"{name}: ini range {got} != firmware {(lo, hi)}")

for e in errors:
    print("ini-check:", e)
print(f"ini-check: {len(consts)} constants, {len(chans)} channels, {len(errors)} findings")
sys.exit(1 if errors else 0)
