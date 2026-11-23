"""Gera interface_vgt6.kicad_pcb (primeira passada): contorno, furos, 4 camadas, footprints com nets e posicionamento por bloco.

Uso (a partir de hardware/interface_vgt6/): python3 gen/pcbgen.py
Sem roteamento: o resultado é um PCB posicionado, com ratsnest, para você rotear no KiCad.
Como o esquemático, o gerador é de uma passada: depois de mexer na placa no KiCad, não rode de novo.
"""
import json, os, subprocess, sys, tempfile
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
SCH = os.path.join(OUT, "interface_vgt6.kicad_sch")
PCB = os.path.join(OUT, "interface_vgt6.kicad_pcb")
FPDIR = "/usr/share/kicad/footprints"
MM = pcbnew.FromMM

# ---- geometria (mm, origem no canto superior esquerdo da placa, y para baixo) ---------------------
BW, BH = 170.0, 138.0
# WeAct: 38,62 x 69,10; coordenadas dos pinos lidas do PDF de contorno (vista de cima)
WEACT_W, WEACT_H = 38.62, 69.10
WX0 = (BW - WEACT_W) / 2          # borda esquerda da WeAct
WY0 = 22.0                        # borda superior da WeAct
HOLES = [(2.8, 2.8), (33.28, 2.8), (2.8, 66.3), (33.28, 66.3)]   # relativos ao canto da WeAct
P1_COL = {"odd": 4.064, "even": 1.524}      # x dos pinos ímpar/par de P1, relativo à borda esquerda da WeAct
P2_COL = {"odd": 37.084, "even": 34.544}
ROW0 = 7.874                               # y da 1ª fila, relativo à borda superior da WeAct


CURSOR = {}


def pt(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def load_fp(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), name)
    if fp is None:
        raise SystemExit("footprint não encontrado: " + fpid)
    return fp


def netlist():
    d = tempfile.mkdtemp()
    xml = os.path.join(d, "n.xml")
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml", "-o", xml, SCH], check=True, capture_output=True)
    root = ET.parse(xml).getroot()
    comps = {}
    for c in root.find("components"):
        sp = c.find("sheetpath")
        ls = c.find("libsource")
        comps[c.get("ref")] = dict(fp=c.findtext("footprint"), value=c.findtext("value"), part=(ls.get("part") if ls is not None else ""),
                                   sheet=(sp.get("names") if sp is not None else "/").strip("/"), pins={})
    for n in root.find("nets"):
        name = n.get("name")
        if name.startswith("unconnected"):
            continue
        for node in n.findall("node"):
            comps[node.get("ref")]["pins"][node.get("pin")] = name
    return comps


def bbox_mm(fp):
    b = fp.GetBoundingBox(False)
    return pcbnew.ToMM(b.GetX()), pcbnew.ToMM(b.GetY()), pcbnew.ToMM(b.GetWidth()), pcbnew.ToMM(b.GetHeight())


def pad_centroid(fp):
    ps = [p.GetPosition() for p in fp.Pads()]
    return (pcbnew.ToMM(sum(p.x for p in ps) / len(ps)), pcbnew.ToMM(sum(p.y for p in ps) / len(ps)))


def move_bbox_to(fp, x, y):
    """põe o canto superior esquerdo da caixa envolvente em (x, y)"""
    bx, by, _, _ = bbox_mm(fp)
    pos = fp.GetPosition()
    fp.SetPosition(pcbnew.VECTOR2I(pos.x + MM(x - bx), pos.y + MM(y - by)))


# ---- regiões de posicionamento por folha (x0, y0, x1, y1) ---------------------------------------
REGIONS = {
    "Alimentação": (4, 16, 62, 80),
    "Saídas INJ/IGN": (4, 82, 62, 123),
    "Entradas": (108, 16, 166, 66),
    "ETB e auxiliares": (108, 68, 166, 123),
    "CAN e headers": (66, 95, 106, 123),
}
# conectores de borda: ref -> (borda, x do canto esquerdo da caixa)
EDGE_ORDER = {"top": ["J1A", "J1B", "J4", "J5", "J6"], "bottom": ["J3", "J2", "J7", "J8"]}
EDGE = {r: (e, None) for e, refs in EDGE_ORDER.items() for r in refs}
EDGE_GAP = 2.0
HEADERS = {"P1": (P1_COL, "left"), "P2": (P2_COL, "right")}


def place_edge(fp, edge, x):
    """x=None: usa o cursor sequencial da borda"""
    if x is None:
        x = CURSOR.get(edge, 3.0)
    cx, cy = pad_centroid(fp)
    bx, by, bw, bh = bbox_mm(fp)
    up, down = cy - by, by + bh - cy
    want_up = edge == "top"
    fp.SetOrientationDegrees(0 if ((up >= down) == want_up) else 180)
    bx, by, bw, bh = bbox_mm(fp)
    y = 1.0 if edge == "top" else BH - bh - 1.0
    move_bbox_to(fp, x, y)
    CURSOR[edge] = x + bw + EDGE_GAP
    if CURSOR[edge] - EDGE_GAP > BW - 2.0:
        print("AVISO: conectores da borda %s passam da largura da placa (%.1f > %.1f)" % (edge, CURSOR[edge] - EDGE_GAP, BW))


GND_NETS = {"PGND", "SGND"}
RAIL_SET = {"+5V", "+5V_F", "+5V_SENS", "+3V3", "VBAT_PROT", "VBAT_ETB", "VBAT_AUX", "VBAT_INJ", "VBAT_IGN"}
SPECIAL = {"BUCK_SW", "BUCK_BST", "ETB_ILIM", "SENS_ILIM", "VR_INP", "VR_INN", "CKP_VR_OUT"}


def make_clusters(comps, per_sheet_refs):
    """associa passivos de desacoplamento/ajuste ao CI dono: ref -> ref do CI"""
    owner = {}
    load = {}
    for sheet, refs in per_sheet_refs.items():
        ics = [r for r in refs if r.startswith("U")]
        for r in sorted(refs):
            c = comps[r]
            if not (r[0] in "RCL") or len(c["pins"]) != 2 or c["part"] == "C_Polarized":
                continue
            nets = list(c["pins"].values())
            sp = [n for n in nets if n in SPECIAL]
            cand = []
            if sp:
                cand = [u for u in ics if sp[0] in comps[u]["pins"].values()]
            elif r[0] == "C" and any(n in GND_NETS for n in nets):
                rail = [n for n in nets if n in RAIL_SET]
                if rail:
                    cand = [u for u in ics if rail[0] in comps[u]["pins"].values()]
            if cand:
                u = min(cand, key=lambda x: load.get(x, 0))
                owner[r] = u
                load[u] = load.get(u, 0) + 1
    return owner


def layout_cluster(anchor, sats, gap=0.6):
    """satélites em fileiras logo abaixo do CI (posições relativas); devolve lista de fps"""
    ax, ay, aw, ah = bbox_mm(anchor)
    x, y, rowh = ax, ay + ah + gap, 0.0
    maxw = max(aw, 8.0) * 1.8
    for fp in sorted(sats, key=lambda f: f.GetReference()):
        _, _, w, h = bbox_mm(fp)
        if x + w - ax > maxw:
            x, y, rowh = ax, y + rowh + gap, 0.0
        move_bbox_to(fp, x, y)
        x += w + gap
        rowh = max(rowh, h)
    return [anchor] + sats


def cluster_bbox(members):
    xs = [bbox_mm(f) for f in members]
    x0 = min(b[0] for b in xs)
    y0 = min(b[1] for b in xs)
    x1 = max(b[0] + b[2] for b in xs)
    y1 = max(b[1] + b[3] for b in xs)
    return x0, y0, x1 - x0, y1 - y0


def shift(members, dx, dy):
    for f in members:
        p = f.GetPosition()
        f.SetPosition(pcbnew.VECTOR2I(p.x + MM(dx), p.y + MM(dy)))


def pack_clusters(clusters, region, gap=1.2):
    x0, y0, x1, y1 = region
    x, y, rowh = x0, y0, 0.0
    overflow = []
    for members in sorted(clusters, key=lambda m: -(cluster_bbox(m)[2] * cluster_bbox(m)[3])):
        bx, by, w, h = cluster_bbox(members)
        if x + w > x1:
            x, y, rowh = x0, y + rowh + gap, 0.0
        if y + h > y1:
            overflow += [m.GetReference() for m in members]
        shift(members, x - bx, y - by)
        x += w + gap
        rowh = max(rowh, h)
    return overflow


def shelf_pack(parts, region, gap=1.2):
    x0, y0, x1, y1 = region
    x, y, rowh = x0, y0, 0.0
    overflow = []
    for fp in sorted(parts, key=lambda f: -(bbox_mm(f)[2] * bbox_mm(f)[3])):
        _, _, w, h = bbox_mm(fp)
        if x + w > x1:
            x, y, rowh = x0, y + rowh + gap, 0.0
        if y + h > y1:
            overflow.append(fp.GetReference())
        move_bbox_to(fp, x, y)
        x += w + gap
        rowh = max(rowh, h)
    return overflow


def main():
    comps = netlist()
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    nets = {}
    for c in comps.values():
        for n in c["pins"].values():
            if n not in nets:
                ni = pcbnew.NETINFO_ITEM(board, n)
                board.Add(ni)
                nets[n] = ni

    per_sheet, fixed = {}, []
    order = [r for e in EDGE_ORDER.values() for r in e]
    keys = order + [r for r in sorted(comps) if r not in order]
    for ref in keys:
        c = comps[ref]
        fp = load_fp(c["fp"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.Reference().SetLayer(pcbnew.F_Fab)  # designador na Fab: silk limpo sobre pads de peças coladas
        if ref in HEADERS:
            # fileira ímpar à direita (vista de cima do contorno da WeAct): espelha o x dos pads do soquete padrão
            for p in fp.Pads():
                q = p.GetPosition()
                p.SetPosition(pcbnew.VECTOR2I(-q.x, q.y))
        for p in fp.Pads():
            n = c["pins"].get(p.GetNumber())
            if n:
                p.SetNet(nets[n])
        board.Add(fp)
        if ref in HEADERS:
            cols, side = HEADERS[ref]
            # posição do pino 1 (fileira ímpar, 1ª linha); o soquete tem pino 1 no (0,0) e pino 2 em x = +2,54 (agora -2,54)
            x1 = WX0 + cols["odd"]
            fp.SetPosition(pt(x1, WY0 + ROW0))
            fixed.append(ref)
        elif ref in EDGE:
            place_edge(fp, *EDGE[ref])
            fixed.append(ref)
        elif ref.startswith("H"):
            pass
        else:
            per_sheet.setdefault(c["sheet"], []).append(fp)

    # furos M3 nos pontos de fixação da WeAct
    holes = [f for f in board.GetFootprints() if f.GetReference().startswith("H")]
    for fp, (hx, hy) in zip(sorted(holes, key=lambda f: f.GetReference()), HOLES):
        fp.SetPosition(pt(WX0 + hx, WY0 + hy))

    overflow = []
    refs_by_sheet = {sh: [f.GetReference() for f in parts] for sh, parts in per_sheet.items()}
    owner = make_clusters(comps, refs_by_sheet)
    for sheet, parts in per_sheet.items():
        byref = {f.GetReference(): f for f in parts}
        sats = {}
        for r, u in owner.items():
            if r in byref:
                sats.setdefault(u, []).append(byref[r])
        used = {f.GetReference() for fl in sats.values() for f in fl}
        clusters = []
        for r, f in byref.items():
            if r in used:
                continue
            clusters.append(layout_cluster(f, sats[r]) if r in sats else [f])
        overflow += pack_clusters(clusters, REGIONS.get(sheet, (4, 62, 62, 104)))
    # NetTie na fronteira PGND/SGND, junto às entradas
    nt = [f for f in board.GetFootprints() if f.GetReference().startswith("NT")]
    for f in nt:
        f.SetPosition(pt(64.0, 12.0))

    # contorno
    outline = pcbnew.PCB_SHAPE(board)
    outline.SetShape(pcbnew.SHAPE_T_RECT)
    outline.SetStart(pt(0, 0))
    outline.SetEnd(pt(BW, BH))
    outline.SetLayer(pcbnew.Edge_Cuts)
    outline.SetWidth(MM(0.1))
    board.Add(outline)

    # planos internos: L2 = PGND (oeste) e SGND (resto); L3 fica livre para polígonos de potência
    def zone(layer, net, rect):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(nets[net])
        z.SetIsFilled(False)
        x0, y0, x1, y1 = rect
        ol = z.Outline()
        ol.NewOutline()
        for px, py in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            ol.Append(MM(px), MM(py))
        z.SetMinThickness(MM(0.2))
        board.Add(z)
        return z

    zone(pcbnew.In1_Cu, "PGND", (0, 0, 64, BH))
    zone(pcbnew.In1_Cu, "SGND", (64, 0, BW, BH))
    zone(pcbnew.In2_Cu, "+5V", (64, 0, BW, BH)) if False else None
    # zonas ficam sem preenchimento: o KiCad preenche ao abrir (B) e o arquivo não carrega polígonos obsoletos

    board.Save(PCB)

    # regras de rede (largura por classe) no .kicad_pro
    pro = os.path.join(OUT, "interface_vgt6.kicad_pro")
    d = json.load(open(pro))
    d["net_settings"] = {
        "classes": [
            {"name": "Default", "clearance": 0.15, "track_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3},
            {"name": "Potencia", "clearance": 0.3, "track_width": 3.0, "via_diameter": 1.0, "via_drill": 0.5},
            {"name": "Potencia_media", "clearance": 0.25, "track_width": 1.0, "via_diameter": 0.8, "via_drill": 0.4},
        ],
        "meta": {"version": 4},
        "netclass_patterns": [
            {"netclass": "Potencia", "pattern": "VBAT_*"},
            {"netclass": "Potencia_media", "pattern": "INJ*_CON"},
            {"netclass": "Potencia_media", "pattern": "ETB_M*"},
            {"netclass": "Potencia_media", "pattern": "+5V"},
            {"netclass": "Potencia_media", "pattern": "PGND"},
        ],
    }
    json.dump(d, open(pro, "w"), indent=2)
    print("footprints:", len(list(board.GetFootprints())), "nets:", len(nets))
    if overflow:
        print("AVISO: não coube na região:", overflow)


if __name__ == "__main__":
    main()
