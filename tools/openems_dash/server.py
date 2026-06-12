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
                    time.sleep(1.0)
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


worker: SerialWorker = None  # type: ignore[assignment]


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


@app.post("/api/pages/{page}/burn")
def api_burn(page: int):
    worker.submit(lambda l: l.burn_page(page))
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


@app.get("/api/log/export")
def api_log_export(rows: int = 120):
    """Exporta o último log como relatório Markdown amigável p/ análise por LLM:
    metadados, estatísticas por canal, transições de status e série temporal
    reamostrada para no máximo `rows` linhas."""
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
           f"- Início (unix): {t0:.2f}", "",
           "## Estatísticas por canal", "",
           "| canal | min | max | média |", "|---|---|---|---|"]
    for k in num_keys:
        vals = [float(r[k]) for r in data]
        md.append(f"| {k} | {min(vals):g} | {max(vals):g} | {sum(vals)/len(vals):.2f} |")

    md += ["", "## Transições de status", ""]
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
    md += ["", f"## Série temporal (reamostrada 1:{step}, {len(sampled)} linhas)", "",
           "| t(s) | " + " | ".join(num_keys) + " |",
           "|" + "---|" * (len(num_keys) + 1)]
    for r in sampled:
        md.append(f"| {float(r['t'])-t0:.2f} | " +
                  " | ".join(f"{float(r[k]):g}" for k in num_keys) + " |")

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
