#!/usr/bin/env python3
"""Higiene da documentação do OpenEMS: impede que documento obsoleto volte a enganar.

Todo documento em docs/ (e hardware/README.md) declara um STATUS no cabeçalho:

    > **Status:** vigente · validado 2026-11-17
    > **Status:** histórico — <por quê, e onde está a fonte atual>
    > **Status:** fora-de-escopo — <por quê>

Verificações (documentos `vigente` levam todas; `histórico`/`fora-de-escopo` só a do cabeçalho):

  STATUS    cabeçalho de status ausente/inválido; `vigente` sem data; validação com mais de 120 dias (aviso)
  REF       caminho de arquivo citado (`src/...`, `docs/...`, link relativo) que não existe
  MAKE      `make alvo` que não existe no Makefile
  SYMBOL    constante/função citada em `crases` que não existe em src/, test/ ou tools/
  REMOVED   tecnologia removida/fora do projeto citada como se estivesse em uso
  VOLATILE  contagem de testes ou hash de commit no texto (envelhece sozinho): aponte para o CI
  FACT      fato já provado errado (tools/doc_facts.txt) reintroduzido

Uso: python3 tools/doc_lint.py [--error] [--only CHECK[,CHECK]] [arquivos...]
Sem --error só avisa (exit 0). `make doc-check` roda com --error e entra no `make ci-local`.
"""
from __future__ import annotations

import argparse
import datetime as dt
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

STATUS_RE = re.compile(r"^>\s*\*\*Status:\*\*\s*(vigente|hist[óo]rico|fora-de-escopo)\b(.*)$", re.I)
DATE_RE = re.compile(r"validado\s+(\d{4}-\d{2}-\d{2})")
MAX_AGE_DAYS = 120

# Onde o cabeçalho de status é obrigatório.
def needs_status(rel: str) -> bool:
    return rel.startswith("docs/") or rel == "hardware/README.md"

# Linhas que declaram, elas mesmas, que a menção é histórica/removida.
HISTORIC_LINE = re.compile(
    r"hist[óo]ric|remov|fora d[eo]|anulad|saiu|n[ãa]o [ée] usad|n[ãa]o faz(?:em)? parte|abandon|"
    r"obsolet|deprecad|substitu|antigo|era \b|ex-|descartad|rejeitad|desativad|n[ãa]o [ée] (a )?base",
    re.I)

# Tecnologias que NÃO fazem parte do projeto (decisão do usuário, 2026-11-17; ROADMAP.md "Escopo").
REMOVED_TOKENS = re.compile(r"\b(TLE8888|MC33810|L9960T?|CJ125|TPS65381A?|MT6835|tle8888_regs)\b")

VOLATILE_RES = [
    (re.compile(r"\b\d{3,5}\s+PASS\b"), "contagem de testes (envelhece; cite o comando, não o número)"),
    (re.compile(r"\b(?:main|HEAD|origin/main)\s+(?:em\s+)?`?[0-9a-f]{7,40}`?"), "hash de ponta de branch no texto (envelhece); hash de commit histórico é permitido"),
    (re.compile(r"Atualizado em \d{4}-\d{2}-\d{2}, `?main`? em"), "estado de branch/commit no texto"),
]

PATH_RE = re.compile(
    r"(?<![\w/.-])((?:src|test|tools|docs|hardware|linker|kicad|\.claude)/[A-Za-z0-9_./+-]*[A-Za-z0-9_]"
    r"(?:\.[A-Za-z0-9]+)?)")
SYMBOL_RE = re.compile(
    r"`((?:k[A-Z][A-Za-z0-9]{3,}|g_[a-z][A-Za-z0-9_]*|"
    r"(?:ecu_sched|ckp|nvm|etb|fuel|ign|sensors|torque|auxiliaries|misfire|knock|output_test|"
    r"out_pin|burn|chip)_[a-z0-9_]+))(?:\(\))?`")
MAKE_RE = re.compile(r"\bmake\s+([a-z][a-z0-9_-]+)")


_ALL_FILES: list[str] | None = None


def path_exists(p: str) -> bool:
    """Existe exatamente; ou, se não tem extensão, é prefixo de algum arquivo (src/app/datalog -> datalog.cpp)."""
    global _ALL_FILES
    if (ROOT / p).exists() or (ROOT / p.rstrip("/")).exists():
        return True
    if re.search(r"\.[A-Za-z0-9]{1,5}$", p):
        return False  # tem extensão: tem de existir de verdade
    if _ALL_FILES is None:
        out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=True)
        _ALL_FILES = out.stdout.splitlines()
    return any(f.startswith(p) for f in _ALL_FILES)


def symbol_exists(sym: str, tokens: set[str]) -> bool:
    if sym in tokens:
        return True
    # família com sufixo omitido (kProtectDis -> kProtectDisOil ...) só para nomes longos
    return len(sym) >= 8 and any(t.startswith(sym) for t in tokens)


def tracked_md() -> list[str]:
    out = subprocess.run(["git", "ls-files", "*.md"], cwd=ROOT, capture_output=True, text=True, check=True)
    return [p for p in out.stdout.split() if "/.venv/" not in p and not p.startswith((".cave/", "graphify-out/"))]


def make_targets() -> set[str]:
    t: set[str] = set()
    for line in (ROOT / "Makefile").read_text(encoding="utf-8", errors="replace").splitlines():
        m = re.match(r"^([A-Za-z0-9_.-]+)\s*:(?!=)", line)
        if m:
            t.add(m.group(1))
    return t


def code_tokens() -> set[str]:
    toks: set[str] = set()
    ident = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
    for sub in ("src", "test", "tools", "linker"):
        for p in (ROOT / sub).rglob("*"):
            if p.is_file() and p.suffix in {".cpp", ".h", ".c", ".py", ".ini", ".ld", ".sh", ".txt", ".toml", ".js", ".html"} \
                    and ".venv" not in p.parts:
                try:
                    toks.update(ident.findall(p.read_text(encoding="utf-8", errors="ignore")))
                except OSError:
                    pass
    return toks


def load_facts() -> list[tuple[re.Pattern[str], str]]:
    facts: list[tuple[re.Pattern[str], str]] = []
    f = ROOT / "tools" / "doc_facts.txt"
    if not f.exists():
        return facts
    for line in f.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        pat, _, why = line.partition("  ->  ")
        facts.append((re.compile(pat), why or "fato já provado errado"))
    return facts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--error", action="store_true", help="exit != 0 se houver erro")
    ap.add_argument("--only", default="", help="lista de checks separada por vírgula")
    ap.add_argument("files", nargs="*")
    args = ap.parse_args()
    only = {c.strip().upper() for c in args.only.split(",") if c.strip()}

    def on(check: str) -> bool:
        return not only or check in only

    files = args.files or tracked_md()
    targets = make_targets()
    tokens = code_tokens()
    facts = load_facts()
    today = dt.date.today()
    problems: list[str] = []
    warnings: list[str] = []

    def err(rel: str, ln: int, check: str, msg: str) -> None:
        problems.append(f"{rel}:{ln}: [{check}] {msg}")

    for rel in files:
        path = ROOT / rel
        if not path.exists():
            continue
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()

        status = None
        for i, line in enumerate(lines[:25], 1):
            m = STATUS_RE.match(line)
            if m:
                status = (m.group(1).lower().replace("ó", "o"), m.group(2), i)
                break

        if needs_status(rel) and on("STATUS"):
            if status is None:
                err(rel, 1, "STATUS", "falta '> **Status:** vigente|histórico|fora-de-escopo' nas primeiras 25 linhas")
            elif status[0] == "vigente":
                d = DATE_RE.search(status[1])
                if not d:
                    err(rel, status[2], "STATUS", "'vigente' precisa de 'validado AAAA-MM-DD'")
                else:
                    age = (today - dt.date.fromisoformat(d.group(1))).days
                    if age > MAX_AGE_DAYS:
                        warnings.append(f"{rel}:{status[2]}: [STATUS] validado há {age} dias (> {MAX_AGE_DAYS}); revalide")
            elif len(status[1].strip(" —-:·")) < 8:
                err(rel, status[2], "STATUS", "'histórico/fora-de-escopo' precisa dizer o porquê e onde está a fonte atual")

        # Só documentos vigentes (ou sem status, p.ex. README/CLAUDE/tools) levam as checagens de conteúdo.
        if status is not None and status[0] != "vigente":
            continue

        in_code = False
        for i, line in enumerate(lines, 1):
            if line.lstrip().startswith("```"):
                in_code = not in_code
            historic = bool(HISTORIC_LINE.search(line))

            if on("REMOVED") and not historic and REMOVED_TOKENS.search(line):
                err(rel, i, "REMOVED", f"'{REMOVED_TOKENS.search(line).group(1)}' não faz parte do projeto; "
                                       "marque a linha como histórica ou remova")
            if on("VOLATILE") and not historic:
                for rx, why in VOLATILE_RES:
                    if rx.search(line):
                        err(rel, i, "VOLATILE", why)
            if on("FACT"):
                for rx, why in facts:
                    if rx.search(line) and not historic:
                        err(rel, i, "FACT", why)
            if historic:
                continue
            if on("REF"):
                for m in PATH_RE.finditer(line):
                    p = m.group(1).rstrip(".,;:)")
                    nxt = line[m.end():m.end() + 1]
                    if nxt in ("*", "{") or any(c in p for c in "*{}<>") or "AAAA" in p or "NNN" in p:
                        continue  # glob / placeholder
                    if not path_exists(p):
                        err(rel, i, "REF", f"caminho inexistente: {p}")
            if on("MAKE"):
                for m in MAKE_RE.finditer(line):
                    if m.group(1) not in targets:
                        err(rel, i, "MAKE", f"alvo inexistente no Makefile: make {m.group(1)}")
            if on("SYMBOL") and not in_code:
                for m in SYMBOL_RE.finditer(line):
                    if not symbol_exists(m.group(1), tokens):
                        err(rel, i, "SYMBOL", f"símbolo não encontrado em src/test/tools: {m.group(1)}")

    for w in warnings:
        print("aviso:", w)
    for p in problems:
        print(p)
    print(f"doc-lint: {len(files)} arquivos, {len(problems)} problemas, {len(warnings)} avisos")
    return 1 if (problems and args.error) else 0


if __name__ == "__main__":
    sys.exit(main())
