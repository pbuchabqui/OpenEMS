#!/usr/bin/env python3
"""Confere a netlist do KiCad da placa de interface VGT6 contra os documentos.

Uso: python3 tools/interface_netcheck.py   (precisa de kicad-cli no PATH)

Verifica:
  - cada pino de P1/P2 tem a net que `docs/hw/interface_vgt6_nets.md` da ao pino do MCU
    (mapa pino do header -> pino do MCU vem de `docs/hw/weact_h562_schematic.md`);
  - pinos do SD da WeAct (PC8..PC12, PD2), PA8, PB11, PA13, PA14, VB e PC13 ficam sem conexao;
  - nenhuma net se chama literalmente GND; PGND e SGND sao nets distintas;
  - unidades de CI multi-unit completamente colocadas (checado pelas referencias na netlist);
  - ordem de vias de J1A..J9 contra a tabela 3 do doc de nets;
  - nenhuma net de um unico no, fora os pinos explicitamente NC.
"""
import os, re, subprocess, sys, tempfile
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCH = os.environ.get("OPENEMS_SCH") or os.path.join(ROOT, "hardware", "interface_vgt6", "interface_vgt6.kicad_sch")
WEACT = os.path.join(ROOT, "docs", "hw", "weact_h562_schematic.md")
NETS = os.path.join(ROOT, "docs", "hw", "interface_vgt6_nets.md")

# nomes de doc que a carrier deixa sem conexao de proposito
NC_NAMES = {"USB_DM", "USB_DP", "SWDIO", "SWCLK", "NRST", "KNOCK_MCU"}
NC_PINS = {"PC8", "PC9", "PC10", "PC11", "PC12", "PD2", "PA8", "PB11", "PA13", "PA14", "VB", "PC13", "PE3", "PA5"}


def header_maps():
    """P1/P2: pino do header -> sinal da WeAct"""
    out = {"P1": {}, "P2": {}}
    cur = None
    for line in open(WEACT, encoding="utf-8"):
        m = re.match(r"###\s+(P[12])\b", line)
        if m:
            cur = m.group(1)
            continue
        if cur and line.startswith("|"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if len(cells) == 4 and re.match(r"^\d+ / \d+$", cells[0]):
                for pins, sig in ((cells[0], cells[1]), (cells[2], cells[3])):
                    a, b = pins.split(" / ")
                    sa, sb = sig.split(" / ")
                    out[cur][int(a)] = sa.strip()
                    out[cur][int(b)] = sb.strip()
        if line.startswith("## ") and cur and "P2" not in line and "Conectores" not in line:
            pass
    return out


def doc_nets():
    """pino do MCU -> nome de net, a partir das tabelas 2.x do doc de nets"""
    res = {}
    for line in open(NETS, encoding="utf-8"):
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 2:
            continue
        names = re.findall(r"`([^`]+)`", cells[0])
        pins = [p.strip() for p in cells[1].split(",")]
        pins = [re.match(r"^(P[A-E]\d+)", p).group(1) if re.match(r"^P[A-E]\d+", p) else None for p in pins]
        if names and pins and all(pins) and len(names) == len(pins):
            for n, p in zip(names, pins):
                res[p] = n
        elif names and pins and pins[0] and len(names) == 1:
            res[pins[0]] = names[0]
    return res


def doc_connectors():
    """J1A..J9: lista ordenada de nets por via, da tabela 3 do doc de nets"""
    res = {}
    for line in open(NETS, encoding="utf-8"):
        cells = [c.strip() for c in line.strip().strip("|").split("|")] if line.startswith("|") else []
        if len(cells) != 4 or not re.match(r"^J\d+[AB]?$", cells[0]) or not cells[2].isdigit():
            continue
        out = []
        for tok in re.sub(r"\([^)]*\)", "", cells[3]).split(","):
            tok = tok.strip()
            m = re.match(r"^`([^`]+)`\s*\.\.\s*`([^`]+)`$", tok)
            if m:
                a, b = m.groups()
                pa, pb = re.match(r"^(.*?)(\d+)(.*)$", a), re.match(r"^(.*?)(\d+)(.*)$", b)
                for n in range(int(pa.group(2)), int(pb.group(2)) + 1):
                    out.append("%s%d%s" % (pa.group(1), n, pa.group(3)))
                continue
            m = re.match(r"^`([^`]+)`(?:\s*×(\d+))?$", tok)
            if m:
                out += [m.group(1)] * int(m.group(2) or 1)
        if len(out) != int(cells[2]):
            res[cells[0]] = None
        else:
            res[cells[0]] = out
    return res


def export_netlist():
    tmp = tempfile.mkdtemp()
    xml = os.path.join(tmp, "n.xml")
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml", "-o", xml, SCH],
                   check=True, capture_output=True)
    return ET.parse(xml).getroot()


def main():
    root = export_netlist()
    nets = {}
    for n in root.find("nets"):
        nets[n.get("name")] = [(x.get("ref"), x.get("pin")) for x in n.findall("node")]
    pin2net = {}
    for name, nodes in nets.items():
        for ref, pin in nodes:
            pin2net[(ref, pin)] = name
    errs = []

    hdr = header_maps()
    exp = doc_nets()
    for tag in ("P1", "P2"):
        for pin, sig in sorted(hdr[tag].items()):
            got = pin2net.get((tag, str(pin)), "<ausente>")
            connected = not got.startswith("unconnected") and got != "<ausente>"
            if sig == "GND":
                want = "SGND"
            elif sig == "VREF+":
                want = "VREF+"
            elif sig in NC_PINS:
                want = None
            elif sig in exp and exp[sig] not in NC_NAMES:
                want = exp[sig]
            else:
                want = None
            if want is None:
                if connected:
                    errs.append("%s pino %d (%s) deveria ficar sem conexao, tem net %s" % (tag, pin, sig, got))
            elif got != want:
                errs.append("%s pino %d (%s): esperado %s, netlist %s" % (tag, pin, sig, want, got))

    for ref, want in doc_connectors().items():
        if want is None:
            errs.append("%s: tabela 3 do doc nao fecha com o numero de vias" % ref)
            continue
        for i, net in enumerate(want, 1):
            got = pin2net.get((ref, str(i)), "<ausente>")
            if got != net:
                errs.append("%s via %d: esperado %s, netlist %s" % (ref, i, net, got))
        extra = [pin for (r, pin) in pin2net if r == ref and int(pin) > len(want)]
        if extra:
            errs.append("%s tem vias alem das %d do doc: %s" % (ref, len(want), sorted(extra)))

    names = set(nets)
    if "GND" in names:
        errs.append("existe net chamada GND (use PGND ou SGND)")
    for must in ("PGND", "SGND"):
        if must not in names:
            errs.append("net %s ausente" % must)
    # PGND e SGND nao podem compartilhar nenhum no
    if set(nets.get("PGND", [])) & set(nets.get("SGND", [])):
        errs.append("PGND e SGND compartilham pinos")

    # redes de um no so (fora NC)
    for name, nodes in nets.items():
        if len(nodes) == 1 and not name.startswith("unconnected"):
            errs.append("net de um no so: %s %s" % (name, nodes[0]))

    # multi-unit: o 74AHCT125 tem 14 pinos; todos precisam aparecer (as 5 unidades colocadas)
    comps = {c.get("ref"): c for c in root.find("components")}
    seen = {}
    for name, nodes in nets.items():
        for ref, pin in nodes:
            seen.setdefault(ref, set()).add(pin)
    for ref, c in comps.items():
        ls = c.find("libsource")
        if ls is not None and ls.get("part") == "74AHCT125" and len(seen.get(ref, ())) != 14:
            errs.append("%s: 74AHCT125 com %d/14 pinos na netlist (unidade nao colocada?)" % (ref, len(seen.get(ref, ()))))

    print("componentes:", len(comps), " nets:", len(nets))
    for e in errs:
        print("ERRO:", e)
    print("netcheck: %d problema(s)" % len(errs))
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
