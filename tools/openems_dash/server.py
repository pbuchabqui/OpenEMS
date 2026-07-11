#!/usr/bin/env python3
"""
server.py — backend do dashboard de calibração OpenEMS.

Uma thread é dona exclusiva da porta serial: faz poll do comando 'A' a ~30Hz
e intercala requests de página (fila) entre polls. Telemetria sai por
WebSocket; páginas por REST.

Uso:
    python3 server.py [--port /dev/ttyACM0] [--http-port 8000] [--rate 30]
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import queue
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

import protocol as proto

BASE = Path(__file__).parent
LOGS = BASE / "logs"

app = FastAPI(title="OpenEMS Dashboard")


@app.middleware("http")
async def no_cache_api(request, call_next):
    # /api/* reflete estado ao vivo da ECU (RAM g_pageN via 'r') — nunca deve
    # ser servido da cache HTTP do browser (ex.: cliques repetidos em
    # "Reload" mostrando valores desactualizados apesar do firmware devolver
    # o byte certo a cada pedido).
    response = await call_next(request)
    if request.url.path.startswith("/api/"):
        response.headers["Cache-Control"] = "no-store"
    return response


class SerialWorker(threading.Thread):
    """Dona exclusiva da serial: poll 'A' + fila de requests."""

    def __init__(self, port: str | None, rate_hz: float):
        super().__init__(daemon=True)
        self.port_arg = port
        self.period = 1.0 / rate_hz
        self.link: proto.OpenEMSLink | None = None
        self.requests: queue.Queue = queue.Queue()
        self.latest: dict | None = None
        self.connected = False
        self.error: str | None = None
        self.info: dict = {}
        # datalog
        self._log_lock = threading.Lock()
        self._log_writer = None
        self._log_file = None
        self.log_path: str | None = None

    def submit(self, fn):
        """Executa fn(link) na thread serial; devolve resultado (bloqueante)."""
        done = threading.Event()
        box = {}

        def job(link):
            try:
                box["result"] = fn(link)
            except Exception as e:  # noqa: BLE001 — devolvido ao caller
                box["error"] = e
            done.set()

        self.requests.put(job)
        if not done.wait(timeout=5.0):
            raise TimeoutError("serial worker não respondeu em 5s")
        if "error" in box:
            raise box["error"]
        return box.get("result")

    def _connect(self):
        try:
            self.link = proto.OpenEMSLink(self.port_arg)
            self.info = {
                "port": self.link.port,
                "signature": self.link.query(),
                "fw": self.link.fw_version(),
                "protocol": self.link.protocol_version(),
            }
            self.connected = True
            self.error = None
        except Exception as e:  # noqa: BLE001
            self.link = None
            self.connected = False
            self.error = str(e)

    def run(self):
        while True:
            if self.link is None:
                self._connect()
                if self.link is None:
                    time.sleep(0.3)
                    continue
            t0 = time.monotonic()
            try:
                rt = self.link.read_realtime()
                d = rt.to_dict()
                d["t"] = time.time()
                self.latest = d
                self._log_row(d)
                # intercalar requests pendentes
                while not self.requests.empty():
                    self.requests.get_nowait()(self.link)
            except Exception as e:  # noqa: BLE001 — placa desconectou/timeout
                self.error = str(e)
                self.connected = False
                try:
                    self.link.close()
                except Exception:  # noqa: BLE001
                    pass
                self.link = None
                continue
            dt = time.monotonic() - t0
            if dt < self.period:
                time.sleep(self.period - dt)

    # ── datalog ─────────────────────────────────────────────────────────
    def log_start(self) -> str:
        with self._log_lock:
            if self._log_writer:
                return self.log_path
            LOGS.mkdir(exist_ok=True)
            name = time.strftime("openems_%Y%m%d_%H%M%S.csv")
            self.log_path = str(LOGS / name)
            self._log_file = open(self.log_path, "w", newline="")
            self._log_writer = None  # header na primeira linha
            return self.log_path

    def log_stop(self) -> str | None:
        with self._log_lock:
            path = self.log_path
            if self._log_file:
                self._log_file.close()
            self._log_file = None
            self._log_writer = None
            self.log_path = None
            return path

    def _log_row(self, d: dict):
        with self._log_lock:
            if self._log_file is None:
                return
            flat = {k: v for k, v in d.items() if not isinstance(v, dict)}
            flat.update({f"st_{k}": int(v) for k, v in d["status"].items()})
            if self._log_writer is None:
                self._log_writer = csv.DictWriter(self._log_file, fieldnames=list(flat))
                self._log_writer.writeheader()
            self._log_writer.writerow(flat)


worker: SerialWorker = None   # type: ignore[assignment]
# O estimulador ESP32 tem interface própria (página web embarcada no ESP32,
# http://<ip-esp32>/) — não é controlado por este dashboard.


# ── REST ────────────────────────────────────────────────────────────────────

@app.get("/api/info")
def api_info():
    return {"connected": worker.connected, "error": worker.error, **worker.info,
            "axes": {"rpm": proto.RPM_AXIS, "map_kpa": proto.MAP_AXIS_KPA},
            "grid_pages": {p: {"name": m["name"], "unit": m["unit"], "fmt": m["fmt"]}
                           for p, m in proto.GRID_PAGES.items()},
            "logging": worker.log_path is not None}


@app.get("/api/pages/{page}")
def api_read_page(page: int):
    buf = worker.submit(lambda l: l.read_page(page))
    if page in proto.GRID_PAGES:
        return {"page": page, "grid": proto.GRID_PAGES[page]["decode"](buf)}
    if page == 8:
        return {"page": page, "pedal_maps": proto.decode_pedal_maps(buf),
                "modes": proto.PEDAL_MAP_MODES, "axis": proto.PEDAL_MAP_AXIS}
    if page == 9:
        return {"page": page, "boost_map": proto.decode_boost_map(buf),
                "rpm_axis": proto.BOOST_RPM_AXIS, "gear_labels": proto.BOOST_GEAR_LABELS}
    if page in proto.FIELD_PAGES:
        return {"page": page, "fields": proto.decode_fields(page, buf)}
    return {"page": page, "raw": buf.hex()}


@app.put("/api/pages/{page}/cells")
def api_write_cells(page: int, body: dict):
    """body: {"cells": [{"row": r, "col": c, "value": v}, ...]}  (grid pages)
            ou {"fields": {"nome": valor_ou_lista, ...}}          (field pages)"""
    if page in proto.GRID_PAGES:
        meta = proto.GRID_PAGES[page]
        writes = []
        for cell in body["cells"]:
            off = (cell["row"] * 16 + cell["col"]) * meta["cell_size"]
            writes.append((off, meta["encode"](cell["value"])))
    elif page == 9:
        rows = body.get("boost_map")
        if not rows or len(rows) != 7:
            return JSONResponse({"error": "boost_map: esperado 7 marchas"}, status_code=400)
        encoded = proto.encode_boost_map(rows)
        worker.submit(lambda l: l.write_page_ram(9, 0, encoded))
        worker.submit(lambda l: l.burn_page(9))
        return {"ok": True, "written": 1}
    elif page == 8:
        maps = body.get("pedal_maps")
        if not maps or len(maps) != 4:
            return JSONResponse({"error": "pedal_maps: esperado 4 modos"}, status_code=400)
        encoded = proto.encode_pedal_maps(maps)
        worker.submit(lambda l: l.write_page_ram(8, 0, encoded))
        worker.submit(lambda l: l.burn_page(8))
        return {"ok": True, "written": 1}
    elif page in proto.FIELD_PAGES:
        writes = [proto.encode_field(page, name, vals)
                  for name, vals in body["fields"].items()]
    else:
        return JSONResponse({"error": f"página {page} não editável"}, status_code=400)

    def do(link):
        for off, data in writes:
            link.write_page_ram(page, off, data)

    worker.submit(do)
    return {"ok": True, "written": len(writes)}


# kFlashWriteSafeRpmX10 (engine/constants.h) — erase/program de flash pode
# congelar o fetch por ~120µs (errata ES0565); firmware recusa o burn (ACK
# de erro, sem detalhe) acima deste RPM. Checamos aqui antes de tentar para
# dar uma mensagem clara em vez de relayar só o ACK genérico.
FLASH_WRITE_SAFE_RPM = 300

@app.post("/api/pages/{page}/burn")
def api_burn(page: int):
    latest = worker.latest
    if latest and latest.get("rpm", 0) > FLASH_WRITE_SAFE_RPM:
        return JSONResponse(
            {"error": f"burn bloqueado: motor a {latest['rpm']} RPM "
                      f"(limite {FLASH_WRITE_SAFE_RPM} RPM — erase/program de "
                      f"flash pode travar o scheduler; pare o motor e repita)"},
            status_code=409)
    try:
        worker.submit(lambda l: l.burn_page(page))
    except Exception as e:  # noqa: BLE001 — devolvido à UI em vez de 500 opaco
        return JSONResponse({"error": f"burn página {page} falhou: {e}"},
                            status_code=502)
    return {"ok": True}


# ── teste de saídas ──────────────────────────────────────────────────────────
# Guard: nenhum comando de teste passa com motor girando (RPM > 0). O firmware
# tem o mesmo bloqueio (enter NAK + aborto imediato no poll de 2ms) — aqui é só
# para dar mensagem clara em vez do ACK genérico.

def _output_test_rpm_guard():
    latest = worker.latest
    if latest and latest.get("rpm", 0) > 0:
        return JSONResponse(
            {"error": f"teste de saídas bloqueado: motor a {latest['rpm']} RPM "
                      f"(só disponível com motor parado)"},
            status_code=409)
    return None


def _output_test_call(fn, desc: str):
    try:
        worker.submit(fn)
    except Exception as e:  # noqa: BLE001
        return JSONResponse({"error": f"{desc} falhou: {e}"}, status_code=502)
    return {"ok": True}


@app.post("/api/output_test/enter")
def api_output_test_enter():
    guard = _output_test_rpm_guard()
    if guard:
        return guard
    return _output_test_call(lambda l: l.test_enter(), "armar teste")


@app.post("/api/output_test/exit")
def api_output_test_exit():
    return _output_test_call(lambda l: l.test_exit(), "desarmar teste")


@app.post("/api/output_test/keepalive")
def api_output_test_keepalive():
    return _output_test_call(lambda l: l.test_keepalive(), "keepalive")


@app.get("/api/output_test/status")
def api_output_test_status():
    try:
        return worker.submit(lambda l: l.test_status())
    except Exception as e:  # noqa: BLE001
        return JSONResponse({"error": f"status falhou: {e}"}, status_code=502)


@app.post("/api/output_test/fire")
def api_output_test_fire(body: dict):
    guard = _output_test_rpm_guard()
    if guard:
        return guard
    kind = body.get("kind")
    cyl = int(body.get("cyl", -1))
    us = int(body.get("us", 0))
    if kind not in ("inj", "ign") or not 0 <= cyl <= 3 or us <= 0:
        return JSONResponse({"error": "fire: esperado kind=inj|ign, cyl 0-3, us>0"},
                            status_code=400)
    if kind == "inj":
        return _output_test_call(lambda l: l.test_fire_inj(cyl, us), f"INJ{cyl+1}")
    return _output_test_call(lambda l: l.test_fire_ign(cyl, us), f"IGN{cyl+1}")


@app.post("/api/output_test/set")
def api_output_test_set(body: dict):
    guard = _output_test_rpm_guard()
    if guard:
        return guard
    target = body.get("target")
    value = int(body.get("value", 0))
    calls = {
        "pump":    lambda l: l.test_pump(value != 0),
        "fan":     lambda l: l.test_fan(value != 0),
        "vvt_exh": lambda l: l.test_vvt(0, value),
        "vvt_int": lambda l: l.test_vvt(1, value),
        "etb":     lambda l: l.test_etb(value),
        "ewg":     lambda l: l.test_ewg(value),
    }
    if target not in calls:
        return JSONResponse({"error": f"target desconhecido: {target}"},
                            status_code=400)
    return _output_test_call(calls[target], f"set {target}={value}")


@app.get("/api/debug/counters")
def api_debug_counters():
    try:
        return worker.submit(lambda l: l.read_debug())
    except Exception as e:  # noqa: BLE001
        return JSONResponse({"error": f"debug counters: {e}"}, status_code=502)


@app.get("/api/can_rx_map")
def api_can_rx_map_get():
    return {"signals": proto.can_rx_map_get(), "signal_names": proto.CAN_RX_SIGNALS}


@app.put("/api/can_rx_map/{signal}")
def api_can_rx_map_set(signal: str, body: dict):
    try:
        proto.can_rx_map_set(signal, body)
    except ValueError as e:
        from fastapi.responses import JSONResponse
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True, "signal": signal, "config": proto.can_rx_map_get()[signal]}


@app.get("/api/wbo2_can_id")
def api_wbo2_can_id_get():
    buf = worker.submit(lambda l: l.read_page(0))
    fields = proto.decode_fields(0, buf)
    return {"id": int(fields.get("wbo2_can_id", 0x180))}


@app.put("/api/wbo2_can_id")
def api_wbo2_can_id_set(body: dict):
    can_id = int(body.get("id", 0x180)) & 0x7FF
    off, data = proto.encode_field(0, "wbo2_can_id", can_id)
    worker.submit(lambda l: l.write_page_ram(0, off, data))
    return {"ok": True, "id": can_id}


@app.post("/api/ltft/reset")
def api_ltft_reset():
    size = proto.PAGE_SIZES[10]
    zeros = b"\x00" * size
    def do_reset(l):
        l.write_page_ram(10, 0, zeros[:256])
        l.write_page_ram(10, 256, zeros[256:])
        l.burn_page(10)
    worker.submit(do_reset)
    return {"ok": True}


@app.get("/api/dirty")
def api_dirty():
    return {"mask": worker.submit(lambda l: l.dirty_mask())}


@app.post("/api/log/start")
def api_log_start():
    return {"path": worker.log_start()}


@app.post("/api/log/stop")
def api_log_stop():
    return {"path": worker.log_stop()}


def _nearest_bin(axis: list, v: float) -> int:
    """Índice da célula mais próxima de v no eixo (para binning de tuning)."""
    return min(range(len(axis)), key=lambda i: abs(axis[i] - v))


def _format_corr_table(label: str, axis: list, values: list, axis_unit: str,
                       val_unit: str) -> list[str]:
    """Mini-tabela 1D para seção de correções."""
    md = [f"**{label}** ({val_unit})", ""]
    hdr = "| " + " | ".join(f"{a}{axis_unit}" for a in axis) + " |"
    sep = "|" + "---|" * len(axis)
    row = "| " + " | ".join(str(v) for v in values) + " |"
    return md + [hdr, sep, row, ""]


@app.get("/api/log/export")
def api_log_export(rows: int = 120, min_samples: int = 8):
    """Relatório Markdown de tuning a partir do último log: pontos de operação
    na grade RPM×MAP (regime permanente), lambda medido vs alvo por célula com
    correção de VE sugerida, qualidade dos dados, e série reamostrada."""
    files = sorted(LOGS.glob("*.csv"))
    if not files:
        return JSONResponse({"error": "sem logs"}, status_code=404)
    path = files[-1]
    with open(path) as fh:
        data = list(csv.DictReader(fh))
    if not data:
        return JSONResponse({"error": "log vazio"}, status_code=404)

    num_keys = [k for k in data[0] if k not in ("t",) and not k.startswith("st_")]
    st_keys = [k for k in data[0] if k.startswith("st_")]
    t0, t1 = float(data[0]["t"]), float(data[-1]["t"])
    dur = t1 - t0

    md = [f"# OpenEMS datalog — {path.name}", ""]
    md += [f"- Amostras: {len(data)}  ·  Duração: {dur:.1f}s  ·  "
           f"Taxa: {len(data)/dur:.1f} Hz" if dur > 0 else f"- Amostras: {len(data)}",
           f"- Início (unix): {t0:.2f}", ""]

    # ── leitura de calibração da ECU ──────────────────────────────────────
    engine_params = None
    ve_grid = None
    spark_grid = None
    target_grid = None
    corr_5 = None
    corr_6 = None
    ltft_data = None
    try:
        engine_params = proto.decode_fields(0, worker.submit(lambda l: l.read_page(0)))
        ve_grid = proto.decode_grid_u8(worker.submit(lambda l: l.read_page(1)))
        spark_grid = proto.decode_grid_i8(worker.submit(lambda l: l.read_page(2)))
        target_grid = proto.decode_grid_i16(worker.submit(lambda l: l.read_page(4)))
        corr_5 = proto.decode_fields(5, worker.submit(lambda l: l.read_page(5)))
        corr_6 = proto.decode_fields(6, worker.submit(lambda l: l.read_page(6)))
        ltft_data = proto.decode_ltft(worker.submit(lambda l: l.read_page(10)))
    except Exception:  # noqa: BLE001 — ECU desconectada: seções omitidas
        pass

    # ── parâmetros do motor ───────────────────────────────────────────────
    if engine_params:
        md += ["## Parâmetros do motor", ""]
        ep = engine_params
        md += [
            f"- Cilindrada: {ep.get('displacement_cc', '?')} cc",
            f"- Injetores: {ep.get('injector_flow_cc_min', '?')} cc/min",
            f"- AFR estequiométrico: {ep.get('stoich_afr_x100', 0):.2f}",
            f"- MAP referência: {ep.get('map_ref_bar_x100', 0):.2f} bar",
            f"- IVC ABDC: {ep.get('ivc_abdc_deg', '?')}°",
            f"- EOI target: {ep.get('default_eoi_lead_deg', '?')}° BTDC (fim da injeção)",
            "",
        ]

    # ── tabelas de correção 1D ────────────────────────────────────────────
    if corr_5:
        md += ["## Tabelas de correção", ""]
        md += _format_corr_table(
            "CLT fuel correction", corr_5["clt_corr_axis_x10"], corr_5["clt_corr_x256"],
            "°C", "factor x256")
        md += _format_corr_table(
            "IAT fuel correction", corr_5["iat_corr_axis_x10"], corr_5["iat_corr_x256"],
            "°C", "factor x256")
        md += _format_corr_table(
            "Warmup enrichment", corr_5["warmup_corr_axis_x10"], corr_5["warmup_corr_x256"],
            "°C", "factor x256")
        md += _format_corr_table(
            "Dead time vs VBatt", corr_5["vbatt_corr_axis_mv"],
            corr_5["injector_dead_time_us"], "V", "ms")
        md += _format_corr_table(
            "Dwell vs VBatt", corr_5["dwell_vbatt_axis_mv"],
            corr_5["dwell_ms_x10_table"], "V", "ms")

    if corr_6:
        md += _format_corr_table(
            "X-tau wall fraction (Q8)", corr_6["xtau_clt_axis_x10"],
            corr_6["xtau_x_fraction_q8"], "°C", "Q8")
        md += _format_corr_table(
            "X-tau time constant", corr_6["xtau_clt_axis_x10"],
            corr_6["xtau_tau_cycles"], "°C", "cycles")

    # ── estatísticas por canal ────────────────────────────────────────────
    md += ["## Estatísticas por canal", "",
           "| canal | min | max | média |", "|---|---|---|---|"]
    for k in num_keys:
        vals = [float(r[k]) for r in data]
        md.append(f"| {k} | {min(vals):g} | {max(vals):g} | {sum(vals)/len(vals):.2f} |")

    # ── análise de tuning: binning RPM×MAP em regime permanente ─────────
    running = [r for r in data if float(r["rpm"]) > 300]
    md += ["", "## Análise de tuning (regime permanente)", ""]
    if not running:
        md += ["**Motor parado durante todo o log** — sem dados de tuning. "
               "Grave um log com o motor em funcionamento (ou HIL ativo)."]
    else:
        # regime permanente: ΔRPM < 200 e ΔTPS < 2% entre amostras vizinhas
        steady = []
        for i, r in enumerate(running):
            if i == 0 or i == len(running) - 1:
                continue
            prv, nxt = running[i - 1], running[i + 1]
            if (abs(float(r["rpm"]) - float(prv["rpm"])) < 200
                    and abs(float(nxt["rpm"]) - float(r["rpm"])) < 200
                    and abs(float(r["tps_pct"]) - float(prv["tps_pct"])) < 2):
                steady.append(r)
        pct = 100 * len(steady) / len(running)
        md += [f"- Amostras com motor girando: {len(running)} · em regime "
               f"permanente: {len(steady)} ({pct:.0f}%) — transientes descartados",
               f"- Células com < {min_samples} amostras são omitidas "
               f"(estatística não confiável)", ""]

        cells: dict = {}
        for r in steady:
            key = (_nearest_bin(proto.MAP_AXIS_KPA, float(r["map_kpa"])),
                   _nearest_bin(proto.RPM_AXIS, float(r["rpm"])))
            cells.setdefault(key, []).append(r)

        rows_out = []
        for (mi, ri), rs in sorted(cells.items()):
            if len(rs) < min_samples:
                continue
            lam = sum(float(r["lambda_x1000"]) for r in rs) / len(rs)
            ve = sum(float(r["ve"]) for r in rs) / len(rs)
            stft = sum(float(r["stft_pct"]) for r in rs) / len(rs)
            adv = sum(float(r["advance_deg"]) for r in rs) / len(rs)
            pw = sum(float(r["pw_ms"]) for r in rs) / len(rs)
            clt_avg = sum(float(r["clt_c"]) for r in rs) / len(rs)
            iat_avg = sum(float(r["iat_c"]) for r in rs) / len(rs)
            line = (f"| {proto.RPM_AXIS[ri]} | {proto.MAP_AXIS_KPA[mi]} | {len(rs)} "
                    f"| {lam/1000:.3f} ")
            if target_grid:
                tgt = target_grid[mi][ri]
                err = 100 * (lam - tgt) / tgt if tgt else 0
                sug = lam / tgt if tgt else 1
                line += (f"| {tgt/1000:.3f} | {err:+.1f}% | x{sug:.3f} ")
            if ve_grid:
                line += f"| {ve_grid[mi][ri]} "
            if spark_grid:
                line += f"| {spark_grid[mi][ri]} "
            if ltft_data:
                line += f"| {ltft_data['ltft_pct'][mi][ri]} "
            line += (f"| {ve:.0f} | {stft:+.1f} | {adv:.1f} | {pw:.2f} "
                     f"| {clt_avg:.0f} | {iat_avg:.0f} |")
            rows_out.append(line)

        if rows_out:
            hdr = "| RPM | MAP kPa | n | lambda "
            sep = "|---|---|---|---"
            if target_grid:
                hdr += "| lambda_alvo | erro_lambda | fator_VE "
                sep += "|---|---|---"
            if ve_grid:
                hdr += "| VE_cal "
                sep += "|---"
            if spark_grid:
                hdr += "| Spark_cal "
                sep += "|---"
            if ltft_data:
                hdr += "| LTFT% "
                sep += "|---"
            hdr += "| VE_med | STFT% | advance | PW_ms | CLT | IAT |"
            sep += "|---|---|---|---|---|---|"
            md += [hdr, sep] + rows_out
            md += ["", "**Como usar**: lambda medido > alvo = mistura pobre -> multiplicar "
                   "a celula da tabela VE pelo fator 'fator_VE' (= lambda_medido/lambda_alvo). "
                   "Confiar apenas em celulas com n alto e STFT estavel."]
            if target_grid is None:
                md += ["", "(ECU desconectada no export — colunas de alvo omitidas)"]
        else:
            md += [f"Nenhuma celula atingiu {min_samples} amostras em regime "
                   "permanente — log curto demais ou operacao so em transiente."]

        # ── calibração atual (células visitadas) ──────────────────────────
        visited = {k for k, rs in cells.items() if len(rs) >= min_samples}
        if visited and (ve_grid or spark_grid):
            md += ["", "## Calibracao atual (celulas visitadas)", ""]
            if ve_grid:
                md += ["### Tabela VE (page 1, uint8)", "",
                       "| MAP (bar x100) | RPM | VE_cal |",
                       "|---|---|---|"]
                for mi, ri in sorted(visited):
                    md.append(f"| {proto.MAP_AXIS_KPA[mi]} | {proto.RPM_AXIS[ri]} "
                              f"| {ve_grid[mi][ri]} |")
                md.append("")
            if spark_grid:
                md += ["### Tabela Spark (page 2, int8 BTDC)", "",
                       "| MAP (bar x100) | RPM | Spark_cal |",
                       "|---|---|---|"]
                for mi, ri in sorted(visited):
                    md.append(f"| {proto.MAP_AXIS_KPA[mi]} | {proto.RPM_AXIS[ri]} "
                              f"| {spark_grid[mi][ri]} |")
                md.append("")
            if target_grid:
                md += ["### Tabela Lambda alvo (page 4, x1000)", "",
                       "| MAP (bar x100) | RPM | lambda_alvo_x1000 |",
                       "|---|---|---|"]
                for mi, ri in sorted(visited):
                    md.append(f"| {proto.MAP_AXIS_KPA[mi]} | {proto.RPM_AXIS[ri]} "
                              f"| {target_grid[mi][ri]} |")
                md.append("")
            if ltft_data:
                md += ["### LTFT multiplicativo (page 10, int8 %)", "",
                       "| MAP (bar x100) | RPM | LTFT% |",
                       "|---|---|---|"]
                for mi, ri in sorted(visited):
                    md.append(f"| {proto.MAP_AXIS_KPA[mi]} | {proto.RPM_AXIS[ri]} "
                              f"| {ltft_data['ltft_pct'][mi][ri]} |")
                md.append("")

        # confiabilidade
        warn = []
        if any(r["st_SENSOR_FAULT"] == "1" for r in running):
            warn.append("SENSOR_FAULT ativo em parte do log — dados suspeitos")
        if any(r["st_WBO2_FAULT"] == "1" for r in running):
            warn.append("WBO2_FAULT — lambda NAO confiavel nesses trechos")
        if not all(r["st_FULL_SYNC"] == "1" for r in running):
            warn.append("trechos sem FULL_SYNC — RPM/fase nao confiaveis")
        d_late = float(running[-1]["late_events"]) - float(running[0]["late_events"])
        if d_late > 0:
            warn.append(f"{d_late:.0f} eventos de agendamento atrasado durante o log")
        md += ["", "## Confiabilidade", ""]
        md += [f"- WARN {w}" for w in warn] if warn else ["- OK sem faults, FULL_SYNC continuo"]

    md += ["", "## Transicoes de status", ""]
    transitions = []
    prev = None
    for r in data:
        cur = {k: r[k] for k in st_keys}
        if prev is not None and cur != prev:
            changed = [f"{k.removeprefix('st_')}={cur[k]}" for k in st_keys if cur[k] != prev[k]]
            transitions.append(f"- t+{float(r['t'])-t0:.2f}s: " + ", ".join(changed))
        prev = cur
    md += transitions if transitions else ["(nenhuma — status constante: " +
        ", ".join(f"{k.removeprefix('st_')}={data[0][k]}" for k in st_keys) + ")"]

    step = max(1, len(data) // rows)
    sampled = data[::step]
    md += ["", f"## Serie temporal (reamostrada 1:{step}, {len(sampled)} linhas)", "",
           "| t(s) | " + " | ".join(num_keys) + " |",
           "|" + "---|" * (len(num_keys) + 1)]
    for r in sampled:
        md.append(f"| {float(r['t'])-t0:.2f} | " +
                  " | ".join(f"{float(r[k]):g}" for k in num_keys) + " |")

    # ── prompt para LLM ──────────────────────────────────────────────────
    md += ["", "## Prompt para LLM", "", """
Voce e um engenheiro de calibracao de ECU analisando um datalog do OpenEMS.
O relatorio acima contem:
1. Parametros do motor (cilindrada, injetores, AFR estequiometrico)
2. Tabelas de correcao 1D (CLT, IAT, warmup, dead time, X-tau)
3. Binning RPM x MAP em regime permanente com lambda medido vs alvo
4. Snapshot das tabelas de calibracao (VE, Spark, Lambda alvo, LTFT) para as celulas visitadas
5. Indicadores de confiabilidade (faults, sync, late events)

### Instrucoes de analise

1. **Prioridade**: corrija VE primeiro (fuel), spark depois (ignition).
2. **Formato de saida**: para cada celula que precisa correcao, retorne uma tabela:

| page | row (MAP idx) | col (RPM idx) | valor_atual | valor_novo | justificativa |
|---|---|---|---|---|---|

   - page 1 = VE (uint8), page 2 = Spark (int8 BTDC)
   - row = indice MAP (0=20 bar x100, 15=300 bar x100)
   - col = indice RPM (0=500, 15=8000)

3. **Regras de decisao**:
   - So confie em celulas com n >= {min_samples} amostras
   - Se STFT esta estavel (|STFT| < 5%), a correcao e mais confiavel
   - Se LTFT ja esta compensando (ex: LTFT = +5%), o VE base precisa de correcao maior
   - Considere CLT/IAT medios do bin: se fora do nominal (20-40C), as correcoes 1D podem estar erradas
   - Se houver SENSOR_FAULT ou WBO2_FAULT, ignore esses trechos

4. **Limites de seguranca**:
   - Delta VE maximo: +/-15% do valor atual por iteracao
   - Delta Spark maximo: +/-3 graus por iteracao
   - Nunca sugira lambda alvo < 0.750 (risco de detonacao)
   - Resolucao do WBO2: +/-0.004 lambda (o2_d4 x 4) — nao persiga erros menores que isso

5. **Correcao VE**: se lambda_medido > lambda_alvo, a mistura esta pobre.
   VE_novo = VE_atual * (lambda_medido / lambda_alvo).
   Se LTFT ja esta em +X%, considere: VE_novo = VE_atual * (1 + LTFT/100) * (lambda_medido / lambda_alvo).

6. **Diagnostico de correcoes 1D**: se TODAS as celulas num range de CLT mostram o mesmo erro
   de lambda, o problema pode ser na tabela de correcao CLT (page 5) em vez de nas celulas VE individuais.
""".strip()]

    out = path.with_suffix(".md")
    out.write_text("\n".join(md) + "\n")
    return FileResponse(out, filename=out.name, media_type="text/markdown")


@app.get("/api/log/download")
def api_log_download():
    files = sorted(LOGS.glob("*.csv"))
    if not files:
        return JSONResponse({"error": "sem logs"}, status_code=404)
    return FileResponse(files[-1], filename=files[-1].name)


# ── WebSocket telemetria ────────────────────────────────────────────────────

@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    await ws.send_text(json.dumps(
        {"connected": worker.connected, "error": worker.error}))
    last_t = 0.0
    try:
        while True:
            d = worker.latest
            if d is not None and d["t"] != last_t:
                last_t = d["t"]
                await ws.send_text(json.dumps(
                    {**d, "connected": worker.connected, "error": worker.error}))
            elif not worker.connected:
                await ws.send_text(json.dumps(
                    {"connected": False, "error": worker.error}))
                await asyncio.sleep(0.5)
            await asyncio.sleep(worker.period / 2)
    except WebSocketDisconnect:
        pass


app.mount("/", StaticFiles(directory=BASE / "static", html=True), name="static")


def main():
    global worker  # noqa: PLW0603
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="porta serial (default: auto /dev/ttyACM*)")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--rate", type=float, default=30.0, help="taxa de poll em Hz")
    args = ap.parse_args()

    worker = SerialWorker(args.port, args.rate)
    worker.start()
    print(f"Dashboard: http://localhost:{args.http_port}")
    uvicorn.run(app, host="127.0.0.1", port=args.http_port, log_level="warning")


if __name__ == "__main__":
    main()
