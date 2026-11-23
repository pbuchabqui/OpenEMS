"""Gerador minimo de .kicad_sch (KiCad 10) a partir das bibliotecas oficiais.

Conexoes por global label / simbolo de alimentacao sobre um fio curto (stub) em cada pino.
Cada pino de cada unidade colocada precisa ser atribuido (net) ou declarado NC: o gerador falha se sobrar.
"""
import os, re, uuid, copy

SYM_DIR = "/usr/share/kicad/symbols"
FP_DIR = "/usr/share/kicad/footprints"
SCH_VERSION = "20251024"


class Q(str):
    """string entre aspas"""


def tokenize(txt):
    i, n = 0, len(txt)
    while i < n:
        c = txt[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            yield c
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while txt[j] != '"':
                if txt[j] == "\\":
                    buf.append(txt[j:j + 2]); j += 2
                else:
                    buf.append(txt[j]); j += 1
            yield Q("".join(buf))
            i = j + 1
        else:
            j = i
            while j < n and txt[j] not in " \t\r\n()":
                j += 1
            yield txt[i:j]
            i = j


def parse(txt):
    stack, cur = [], []
    for t in tokenize(txt):
        if t == "(" and not isinstance(t, Q):
            stack.append(cur); cur = []
        elif t == ")" and not isinstance(t, Q):
            node = cur; cur = stack.pop(); cur.append(node)
        else:
            cur.append(t)
    return cur[0]


def dump(node, ind=0):
    pad = "\t" * ind
    if not isinstance(node, list):
        return ('"%s"' % node) if isinstance(node, Q) else str(node)
    if all(not isinstance(c, list) for c in node):
        return pad + "(" + " ".join(dump(c) for c in node) + ")"
    head = [c for c in node if not isinstance(c, list)]
    # atomos iniciais ficam na primeira linha
    k = 0
    while k < len(node) and not isinstance(node[k], list):
        k += 1
    out = pad + "(" + " ".join(dump(c) for c in node[:k])
    for c in node[k:]:
        if isinstance(c, list):
            out += "\n" + dump(c, ind + 1)
        else:
            out += " " + dump(c)
    return out + "\n" + pad + ")"


_libs = {}


def lib(name):
    if name not in _libs:
        with open(os.path.join(SYM_DIR, name + ".kicad_sym")) as f:
            _libs[name] = parse(f.read())
    return _libs[name]


def find_sym(libname, symname):
    for c in lib(libname)[1:]:
        if isinstance(c, list) and c and c[0] == "symbol" and c[1] == symname:
            return c
    raise KeyError("%s:%s" % (libname, symname))


def prop(node, name):
    for c in node:
        if isinstance(c, list) and c and c[0] == "property" and c[1] == name:
            return c
    return None


def flatten(libname, symname):
    node = copy.deepcopy(find_sym(libname, symname))
    ext = None
    for c in node:
        if isinstance(c, list) and c and c[0] == "extends":
            ext = c[1]
    if ext is None:
        return node
    parent = flatten(libname, ext)
    # renomeia sub-simbolos do pai
    for c in parent:
        if isinstance(c, list) and c and c[0] == "symbol":
            c[1] = Q(c[1].replace(ext, symname, 1) if c[1].startswith(ext + "_") else c[1])
    parent[1] = Q(symname)
    for c in node:
        if isinstance(c, list) and c and c[0] == "property":
            old = prop(parent, c[1])
            if old is not None:
                parent[parent.index(old)] = c
            else:
                parent.append(c)
    return parent


class SymDef:
    def __init__(self, libname, symname):
        self.lib, self.name = libname, symname
        self.node = flatten(libname, symname)
        self._scan()

    def _scan(self):
        self.pins = {}  # unit -> list of dict
        self.units = set()
        for c in self.node:
            if isinstance(c, list) and c and c[0] == "symbol":
                m = re.match(r".*_(\d+)_(\d+)$", c[1])
                u = int(m.group(1))
                for p in c:
                    if isinstance(p, list) and p and p[0] == "pin":
                        at = [x for x in p if isinstance(x, list) and x[0] == "at"][0]
                        nm = [x for x in p if isinstance(x, list) and x[0] == "name"][0][1]
                        num = [x for x in p if isinstance(x, list) and x[0] == "number"][0][1]
                        hid = any(isinstance(x, list) and x[0] == "hide" and x[1] == "yes" for x in p) or "hide" in p
                        self.pins.setdefault(u, []).append(dict(
                            x=float(at[1]), y=float(at[2]), a=int(float(at[3])), name=str(nm), num=str(num),
                            type=p[1], hide=hid))
                        if u:
                            self.units.add(u)

    def unit_pins(self, unit):
        return self.pins.get(0, []) + self.pins.get(unit, [])

    def embedded(self):
        n = copy.deepcopy(self.node)
        n[1] = Q("%s:%s" % (self.lib, self.name))
        return n


_defs = {}


def symdef(libname, symname):
    k = (libname, symname)
    if k not in _defs:
        _defs[k] = SymDef(libname, symname)
    return _defs[k]


def U():
    return str(uuid.uuid4())


GND_NETS = {"PGND", "SGND", "GND"}
RAIL_NETS = {"+5V", "+5V_SENS", "+3V3", "VDDA", "VREF+", "VBAT_RAW_A", "VBAT_RAW_B", "VBAT_PROT",
             "VBAT_INJ", "VBAT_IGN", "VBAT_ETB", "VBAT_AUX", "VBAT_MON"} | GND_NETS


def eff(size=1.27, hide=False, just=None):
    e = ["effects", ["font", ["size", size, size]]]
    if just:
        e.append(["justify"] + just.split())
    if hide:
        e.append(["hide", "yes"])
    return e


class Counter:
    def __init__(self):
        self.n = {}

    def next(self, prefix):
        self.n[prefix] = self.n.get(prefix, 0) + 1
        return "%s%d" % (prefix, self.n[prefix])


refs = Counter()


class _P:
    n = 0

    def bump(self):
        self.n += 1
        return self.n


PWRN = _P()


class Sheet:
    def __init__(self, project, root_uuid, own_uuid, title, page, paper="A3", is_root=False):
        self.project, self.root_uuid, self.own_uuid = project, root_uuid, own_uuid
        self.title, self.page, self.paper, self.is_root = title, page, paper, is_root
        self.items, self.libsyms, self.errors = [], {}, []
        self.pwr_n = 0
        self.nets_seen = {}

    @property
    def path(self):
        return "/" + self.root_uuid + ("" if self.is_root else "/" + self.own_uuid)

    def _lib(self, d):
        key = "%s:%s" % (d.lib, d.name)
        if key not in self.libsyms:
            self.libsyms[key] = d.embedded()

    def _inst(self, ref, unit):
        return ["instances", ["project", Q(self.project), ["path", Q(self.path), ["reference", Q(ref)], ["unit", unit]]]]

    def _symbol(self, d, ref, value, fp, x, y, unit, pinids, mirror=None, show=True, rot=0, extra=None, vsize=1.27):
        self._lib(d)
        n = ["symbol", ["lib_id", Q("%s:%s" % (d.lib, d.name))], ["at", x, y, rot]]
        if mirror:
            n.append(["mirror", mirror])
        n += [["unit", unit], ["exclude_from_sim", "no"], ["in_bom", "yes" if not ref.startswith("#") else "no"],
              ["on_board", "yes" if not ref.startswith("#") else "no"], ["dnp", "no"], ["uuid", Q(U())]]
        ph = ref.startswith("#")
        n.append(["property", Q("Reference"), Q(ref), ["at", x + 2.54, y - 2.54, 0], eff(1.27, hide=ph or not show, just="left")])
        n.append(["property", Q("Value"), Q(value), ["at", x + 2.54, y + 2.54, 0], eff(vsize, hide=not show, just="left")])
        n.append(["property", Q("Footprint"), Q(fp), ["at", x, y, 0], eff(1.27, hide=True)])
        n.append(["property", Q("Datasheet"), Q(""), ["at", x, y, 0], eff(1.27, hide=True)])
        n.append(["property", Q("Description"), Q(""), ["at", x, y, 0], eff(1.27, hide=True)])
        if extra:
            for k, v in extra.items():
                n.append(["property", Q(k), Q(v), ["at", x, y, 0], eff(1.27, hide=True)])
        for pid in pinids:
            n.append(["pin", Q(pid), ["uuid", Q(U())]])
        n.append(self._inst(ref, unit))
        self.items.append(n)

    def wire(self, x1, y1, x2, y2):
        self.items.append(["wire", ["pts", ["xy", x1, y1], ["xy", x2, y2]], ["stroke", ["width", 0], ["type", "default"]], ["uuid", Q(U())]])

    def label(self, net, x, y, ang):
        self.items.append(["global_label", Q(net), ["shape", "bidirectional"], ["at", x, y, ang],
                           ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left" if ang in (0, 90) else "right"]],
                           ["uuid", Q(U())]])

    def noconn(self, x, y):
        self.items.append(["no_connect", ["at", x, y], ["uuid", Q(U())]])

    def text(self, s, x, y, size=2.0):
        self.items.append(["text", Q(s), ["exclude_from_sim", "no"], ["at", x, y, 0],
                           ["effects", ["font", ["size", size, size]], ["justify", "left", "bottom"]], ["uuid", Q(U())]])

    def rail(self, net, x, y, flag=False):
        """simbolo de alimentacao com o pino em (x,y)"""
        base = "GND" if net in GND_NETS else "+5V"
        d = symdef("power", base)
        self._symbol(d, "#PWR%04d" % PWRN.bump(), net, "", x, y, 1, ["1"], show=True, vsize=1.0)
        if flag:
            f = symdef("power", "PWR_FLAG")
            self._symbol(f, "#FLG%04d" % PWRN.bump(), "PWR_FLAG", "", x, y, 1, ["1"], show=False)

    def attach(self, net, px, py, outward, flag=False):
        """stub de 2.54 mm ate o ponto de conexao e depois label ou rail. outward em graus (direcao para fora do corpo)."""
        dx, dy = {0: (1, 0), 180: (-1, 0), 90: (0, -1), 270: (0, 1)}[outward]
        ex, ey = round(px + dx * 2.54, 2), round(py + dy * 2.54, 2)
        self.wire(px, py, ex, ey)
        if net in RAIL_NETS:
            self.rail(net, ex, ey, flag)
        else:
            lab_ang = {0: 0, 180: 180, 90: 90, 270: 270}[outward]
            self.label(net, ex, ey, lab_ang)
        self.nets_seen.setdefault(net, 0)
        self.nets_seen[net] += 1

    def part(self, libname, symname, ref_prefix, value, fp, x, y, nets, unit=1, nc=(), mirror=None, ref=None, flags=(), extra=None, hide_pwr_ok=True):
        d = symdef(libname, symname)
        ref = ref or refs.next(ref_prefix) if ref is None else ref
        if fp:
            fpl, fpn = fp.split(":")
            if not os.path.exists(os.path.join(FP_DIR, fpl + ".pretty", fpn + ".kicad_mod")):
                self.errors.append("footprint inexistente: %s" % fp)
        pins = d.unit_pins(unit)
        self._symbol(d, ref, value, fp, x, y, unit, [p["num"] for p in pins], mirror=mirror, extra=extra)
        used = set()
        for p in pins:
            key = p["num"] if p["num"] in nets else (p["name"] if p["name"] in nets else None)
            px = x + (-p["x"] if mirror == "y" else p["x"])
            py = round(y - p["y"], 2)
            px = round(px, 2)
            out = {0: 180, 180: 0, 90: 270, 270: 90}[p["a"]]
            if mirror == "y":
                out = {0: 0, 180: 180}.get(p["a"], out) if p["a"] in (0, 180) else out
                out = 0 if p["a"] == 0 else (180 if p["a"] == 180 else out)
            if key is not None:
                used.add(key)
                self.attach(nets[key], px, py, out, flag=(nets[key] in flags or p["num"] in flags))
            elif p["num"] in nc or p["name"] in nc:
                self.noconn(px, py)
            elif p["hide"]:
                continue
            else:
                self.errors.append("%s unit %s pino %s(%s) sem net" % (ref, unit, p["num"], p["name"]))
        for k in nets:
            if k not in used:
                self.errors.append("%s: net para pino inexistente %r" % (ref, k))
        return ref

    def write(self, path, children=()):
        sym = ["lib_symbols"] + list(self.libsyms.values())
        top = ["kicad_sch", ["version", SCH_VERSION], ["generator", Q("eeschema")], ["generator_version", Q("10.0")],
               ["uuid", Q(self.root_uuid if self.is_root else self.own_uuid)], ["paper", Q(self.paper)],
               ["title_block", ["title", Q(self.title)], ["company", Q("OpenEMS")]], sym]
        top += self.items
        top += list(children)
        top.append(["sheet_instances", ["path", Q("/"), ["page", Q(str(self.page))]]]) if self.is_root else None
        top = [t for t in top if t is not None]
        top.append(["embedded_fonts", "no"])
        with open(path, "w") as f:
            f.write(dump(top) + "\n")


def renumber(libname, symname, newname, mapping, newlib="OpenEMS"):
    """copia de um simbolo com os numeros de pino trocados (ex.: Q_NMOS D/G/S -> 2/1/3 do DPAK)"""
    k = (newlib, newname)
    if k in _defs:
        return _defs[k]
    d = SymDef(libname, symname)
    for c in d.node:
        if isinstance(c, list) and c and c[0] == "symbol":
            c[1] = Q(c[1].replace(symname, newname, 1))
            for p in c:
                if isinstance(p, list) and p and p[0] == "pin":
                    for x in p:
                        if isinstance(x, list) and x[0] == "number":
                            x[1] = Q(mapping.get(str(x[1]), str(x[1])))
    d.node[1] = Q(newname)
    p = prop(d.node, "Value")
    if p:
        p[2] = Q(newname)
    d.name, d.lib = newname, newlib
    for u in d.pins:
        for pin in d.pins[u]:
            pin["num"] = mapping.get(pin["num"], pin["num"])
    _defs[k] = d
    return d


def custom(name, text, newlib="OpenEMS"):
    """simbolo escrito a mao (s-expression) registrado como OpenEMS:name"""
    d = SymDef.__new__(SymDef)
    d.lib, d.name = newlib, name
    d.node = parse(text)
    d._scan()
    _defs[(newlib, name)] = d
    return d


def box_symbol(name, refp, w, left=(), right=(), top=(), bottom=(), fpdesc=""):
    """caixa retangular; cada pino (num, nome, tipo). w em mm (multiplo de 2.54)."""
    h = max(len(left), len(right), 1) * 2.54 + 2.54
    h = max(h, 7.62)
    ef = "(effects (font (size 1.27 1.27)))"
    out = ['(symbol "%s" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)' % name,
           '(property "Reference" "%s" (at 0 %.2f 0) %s)' % (refp, h / 2 + 2.54, ef),
           '(property "Value" "%s" (at 0 %.2f 0) %s)' % (name, -h / 2 - 2.54, ef),
           '(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
           '(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
           '(property "Description" "%s" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))' % fpdesc,
           '(symbol "%s_0_1" (rectangle (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.254) (type default)) (fill (type background))))' % (name, -w / 2, h / 2, w / 2, -h / 2),
           '(symbol "%s_1_1"' % name]
    def pins(lst, x, ang, vertical=False):
        for i, (num, nm, ty) in enumerate(lst):
            y = h / 2 - 2.54 - i * 2.54
            out.append('(pin %s line (at %.2f %.2f %d) (length 2.54) (name "%s" %s) (number "%s" %s))' % (ty, x, y, ang, nm, ef, num, ef))
    pins(left, -w / 2 - 2.54, 0)
    pins(right, w / 2 + 2.54, 180)
    for i, (num, nm, ty) in enumerate(top):
        x = (i - (len(top) - 1) / 2) * 5.08
        out.append('(pin %s line (at %.2f %.2f 270) (length 2.54) (name "%s" %s) (number "%s" %s))' % (ty, x, h / 2 + 2.54, nm, ef, num, ef))
    for i, (num, nm, ty) in enumerate(bottom):
        x = (i - (len(bottom) - 1) / 2) * 5.08
        out.append('(pin %s line (at %.2f %.2f 90) (length 2.54) (name "%s" %s) (number "%s" %s))' % (ty, x, -h / 2 - 2.54, nm, ef, num, ef))
    out.append(")")
    out.append(")")
    return custom(name, "\n".join(out))


def write_syslib(path, names):
    syms = []
    for n in names:
        node = copy.deepcopy(_defs[("OpenEMS", n)].node)
        syms.append(node)
    top = ["kicad_symbol_lib", ["version", SCH_VERSION], ["generator", Q("openems")], ["generator_version", Q("10.0")]] + syms
    with open(path, "w") as f:
        f.write(dump(top) + "\n")
