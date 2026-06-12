# OpenEMS Dashboard

Dashboard web de telemetria e calibração em tempo real para a ECU OpenEMS
(STM32H562, USB CDC).

## Requisitos

```bash
pip install fastapi uvicorn pyserial
```

## Uso

```bash
python3 server.py                      # auto-detecta /dev/ttyACM*
python3 server.py --port /dev/ttyACM0 --http-port 8000 --rate 30
```

Abrir <http://localhost:8000>.

## Funcionalidades

- **Telemetria** ~30 Hz via WebSocket: RPM, MAP, TPS, λ, PW, avanço, CLT/IAT,
  VE, STFT + strip-charts (janela 60 s, uPlot) + LEDs de status
  (FULL_SYNC, faults, sched) + diagnóstico de loop.
- **Editores de tabela** VE (pág. 1), Spark (pág. 2), Lambda target (pág. 4):
  grid 16×16 com heatmap, eixos reais RPM×MAP, edição por célula,
  célula corrente do motor destacada ao vivo.
  - *Enviar (RAM)* → comando `x` (só RAM, tuning ao vivo, só células alteradas)
  - *Burn → flash* → comando `b` (persiste)
- **Parâmetros** (págs. 5/6/7): correções 1D, dead time, dwell, AE, X-Tau,
  crank, dwell 2D — formulário de campos nomeados, mesmo fluxo enviar/burn.
- **Datalog** CSV em `logs/` com todos os campos do realtime.

## Arquitetura

- `protocol.py` — biblioteca do protocolo serial (mestre-único, thread-safe);
  codecs das páginas espelhando `src/app/ui_protocol.cpp`.
- `server.py` — FastAPI; uma thread dona da serial faz poll de `A` a 30 Hz e
  intercala leituras/escritas de página; WS `/ws/telemetry` + REST `/api/...`.
- `static/` — frontend vanilla JS + uPlot vendorizado (sem build step).

> A página 7 (dwell 2D) requer firmware com o fix de acesso à página 7
> (`ui_protocol.cpp`) — ver commit correspondente.
