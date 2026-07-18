# OpenEMS Calibration Cockpit

Dashboard web de telemetria e calibração em tempo real para a ECU OpenEMS
(STM32H562, USB CDC). Frontend redesenhado: cockpit escuro denso, navegação
por grupos (Tables / Maps / Engine / Diag), helpers puras testáveis.

## Requisitos

Debian/Ubuntu bloqueia `pip install` no Python do sistema (PEP 668).
Use um **venv** local (recomendado):

```bash
cd tools/openems_dash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Alternativas:

```bash
# pacotes Debian (sem venv)
sudo apt install python3-fastapi python3-uvicorn python3-serial

# ou override (não recomendado)
pip install --break-system-packages fastapi uvicorn pyserial
```

## Uso

```bash
cd tools/openems_dash
source .venv/bin/activate          # se usou venv
python server.py                   # auto-detecta /dev/ttyACM*
python server.py --port /dev/ttyACM0 --http-port 8000 --rate 30
```

Abrir <http://localhost:8000>.

Sem activar o venv:

```bash
./.venv/bin/python server.py
```

## Funcionalidades

- **Status bar** ~30 Hz: gauges primários (RPM, MAP, TPS, λ, λtgt, PW, Ign, CLT)
  + chips SYNC / SEQ / LIMP / WBO2 / REV / FAULT / TC / BENCH.
- **Bench-mode** (sidebar): comando `B` — CLT=90°C, IAT=25°C, λ=1.000 simulados
  e timeouts CKP relaxados para HIL (sem sondas / WBO2).
- **Telemetria**: gauges secundários, strip-charts 60 s (uPlot),
  osciloscópio CKP/CMP, diagnóstico de loop.
- **Editores de tabela** VE (pág. 1), Spark (pág. 2), Lambda target (pág. 4):
  grid 20×20 com heatmap, eixos RPM×MAP, Trace vs Manual, auto-write RAM.
  - *Read* → relê a página da ECU (RAM)
  - *Write* → escreve pendentes na RAM (edições já vão ao teclar)
  - *Save* → grava flash (`b`); badge “não gravado em flash”
- **Atalhos (grelha, modo Manual):**
  - `A` / `Z` (ou `+`/`-`) — step nas células seleccionadas
  - `H` / `V` — interpolação linear horizontal/vertical
  - `X` — undo da última operação
  - Setas / Shift+setas / arraste — navegação e selecção
- **Maps**: Pedal (pág. 8), Boost (pág. 9), Learn/LTFT (pág. 12).
- **Parâmetros** (págs. 0/5/6/7): correções 1D, dead time, dwell, AE, X-Tau,
  crank, CAN RX — formulário com filtro, Write/Save.
- **Output tests**: injectors/coils/ETB/EWG (motor parado).
- **Datalog** CSV em `logs/` com todos os campos do realtime.

## Arquitetura

- `protocol.py` — protocolo serial (mestre-único); codecs das páginas.
- `server.py` — FastAPI; thread serial poll `A` @ 30 Hz; WS `/ws/telemetry` + REST.
- `static/` — frontend sem build step:
  - `index.html` + `style.css` — shell / design system
  - `js/helpers.js` — helpers puras (heatmap, axis, clamp, format)
  - `app.js` — telemetria, grelhas, params, maps, output tests
  - `uplot.min.*` — charts vendorizados

## Testes (helpers puras)

```bash
node --test tests/helpers.test.js
```

> A página 7 (dwell 2D) requer firmware com o fix de acesso à página 7
> (`ui_protocol.cpp`) — ver commit correspondente.
