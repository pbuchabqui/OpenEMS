#!/usr/bin/env python3
"""
OpenEMS serial <-> WebSocket bridge.

Serve a UI estática de scripts/ui/ via HTTP e faz proxy bidirecional
entre o browser (WebSocket) e a placa (UART/serial).

Uso:
    python3 scripts/bridge.py --port /dev/ttyUSB0
    python3 scripts/bridge.py --port COM3 --baud 115200 --http-port 8080 --ws-port 8765

Dependências:
    pip install pyserial websockets
"""

import argparse
import asyncio
import http.server
import os
import threading
import serial


try:
    import websockets
except ImportError:
    raise SystemExit("Instale dependências: pip install pyserial websockets")


# ── Estado global ─────────────────────────────────────────────────────────────

_serial_port: serial.Serial | None = None
_client_queues: set[asyncio.Queue] = set()
_loop: asyncio.AbstractEventLoop | None = None


# ── Thread de leitura serial ──────────────────────────────────────────────────

def _serial_reader(port: str, baud: int) -> None:
    """Lê serial em thread dedicada; faz broadcast para todas as filas asyncio."""
    global _serial_port
    try:
        _serial_port = serial.Serial(port, baud, timeout=0.02)
        print(f"Serial aberta: {port} @ {baud} baud")
    except serial.SerialException as e:
        print(f"ERRO ao abrir serial: {e}")
        return

    while True:
        try:
            data = _serial_port.read(256)
        except serial.SerialException:
            break
        if data and _loop is not None:
            _loop.call_soon_threadsafe(_broadcast, data)


def _broadcast(data: bytes) -> None:
    """Chamada no loop asyncio: enfileira bytes para cada cliente WS conectado."""
    for q in list(_client_queues):
        try:
            q.put_nowait(data)
        except asyncio.QueueFull:
            pass  # cliente lento; descarta


# ── Handler WebSocket ─────────────────────────────────────────────────────────

async def _ws_handler(ws) -> None:
    q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=64)
    _client_queues.add(q)
    addr = ws.remote_address
    print(f"WS conectado: {addr}")

    async def send_loop() -> None:
        while True:
            data = await q.get()
            try:
                await ws.send(data)
            except websockets.exceptions.ConnectionClosed:
                break

    send_task = asyncio.create_task(send_loop())

    try:
        async for msg in ws:
            if _serial_port is None or not _serial_port.is_open:
                continue
            if isinstance(msg, bytes):
                _serial_port.write(msg)
            else:
                try:
                    _serial_port.write(bytes.fromhex(msg))
                except ValueError:
                    pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        send_task.cancel()
        _client_queues.discard(q)
        print(f"WS desconectado: {addr}")


# ── Servidor HTTP estático ────────────────────────────────────────────────────

class _SilentHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):  # suprime log por request
        pass


def _start_http(http_port: int, ui_dir: str) -> None:
    os.chdir(ui_dir)
    server = http.server.HTTPServer(('', http_port), _SilentHandler)
    print(f"UI disponível em: http://localhost:{http_port}")
    server.serve_forever()


# ── Main ──────────────────────────────────────────────────────────────────────

async def _main(args: argparse.Namespace) -> None:
    global _loop
    _loop = asyncio.get_running_loop()

    ui_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ui')
    os.makedirs(ui_dir, exist_ok=True)

    http_thread = threading.Thread(
        target=_start_http, args=(args.http_port, ui_dir), daemon=True
    )
    http_thread.start()

    serial_thread = threading.Thread(
        target=_serial_reader, args=(args.port, args.baud), daemon=True
    )
    serial_thread.start()

    print(f"WS escutando em: ws://localhost:{args.ws_port}")
    async with websockets.serve(_ws_handler, '', args.ws_port):
        await asyncio.Future()  # executa indefinidamente


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='OpenEMS serial/WebSocket bridge')
    p.add_argument('--port',      default='/dev/ttyUSB0', help='Porta serial')
    p.add_argument('--baud',      type=int, default=115200, help='Baud rate')
    p.add_argument('--ws-port',   type=int, default=8765,  help='Porta WebSocket')
    p.add_argument('--http-port', type=int, default=8080,  help='Porta HTTP para UI')
    try:
        asyncio.run(_main(p.parse_args()))
    except KeyboardInterrupt:
        print("\nBridge encerrada.")
