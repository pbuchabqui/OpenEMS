# OpenEMS ESP32 Combined — CKP Generator + Scope + Ardu-Stim Features

Gerador de sinais CKP/CMP (60-2) para bancada HIL do OpenEMS, com scope lógico
de 10 canais e funcionalidades Ardu-Stim integradas.

## Hardware

- **ESP32** (qualquer variante: ESP32, ESP32-S3, ESP32-C3, etc.)
- **RMT hardware** para CKP (GPIO2) e CMP (GPIO4) — jitter ~zero, CPU load ~zero
- **LEDC PWM** para sensores analógicos (MAP, TPS, CLT, IAT, etc.)
- **Wire loopback** GPIO2→GPIO34 para captura CKP no scope

## Ligações ao STM32H562

| ESP32 GPIO | STM32 | Função |
|---|---|---|
| GPIO 2 → | PA0 | CKP output (60-2, RMT) |
| GPIO 4 → | PA1 | CMP output (1 pulso/720°, RMT) |
| GPIO 2 → | GPIO34 | Loopback interno p/ scope CKP |
| GPIO 32 ← | PE9 | IGN1 |
| GPIO 33 ← | PE11 | IGN2 |
| GPIO 25 ← | PE13 | IGN3 (disabled=MAP PWM) |
| GPIO 26 ← | PE14 | IGN4 (disabled=TPS PWM) |
| GPIO 27 ← | PC6 | INJ1 |
| GPIO 14 ← | PC7 | INJ2 |
| GPIO 12 ← | PC8 | INJ3 |
| GPIO 13 ← | PC9 | INJ4 |
| GND — | GND | Obrigatório |

## Funcionalidades

### CKP Generator (Ardu-Stim Features)
- **Geração RMT 60-2**: CKP (58 dentes + gap) + CMP (1 pulso por 720°)
- **Modos RPM**: FIXED, SWEEP (varrimento linear entre RPMs), POT (potenciómetro no ADC)
- **Compressão**: modulação senoidal de RPM em baixas rotações (simula tranco de arranque)
- **Inversão de polaridade**: CKP e/ou CMP invertidos via comando
- **Persistência NVS**: config guardada entre power cycles
- **Presets**: IDLE, CRANK, CRUISE, WOT, COAST (RPM + sensores)
- **Protocolo binário Ardu-Stim**: compatível com GUI Electron original

### Scope Lógico (10 canais)
- IGN1-4, INJ1-4, CKP (loopback), CMP (virtual)
- Modos: LIVE table, EDGE log, PULSE log, WAVE bar
- Timing analysis 720°: sequência de ignição, ângulo de avanço, spacing

## Protocolo Serial

### Texto (115200 baud, compatível hil_test.py)
```
RPM <n>            Define RPM
+ / -              RPM ± 100
0-9                Presets
MODE FIXED         RPM fixo
MODE SWEEP <l> <h> <u>  Varrimento linear
MODE POT           Potenciómetro
COMPRESS ON/OFF    Compressão
INVERT CRANK/CAM   Inverter polaridade
STATUS             Estado completo
SAVE               Guardar em NVS
IDLE/CRANK/...     Presets
MAP/TPS/CLT/IAT    Sensores
l/e/p/w/t/s/r/?    Scope commands
```

### Binário (Ardu-Stim compatível, usado pela GUI Electron)
Comandos: `c` `C` `L` `n` `N` `p` `P` `R` `r` `s` `S` `X`

## Controlo via WiFi

O ESP32 liga-se à tua rede WiFi (modo Station) e expõe uma página web com
sliders para RPM, MAP e TPS — acessível de qualquer browser (telemóvel, PC)
na mesma rede.

1. Edita `WIFI_SSID` e `WIFI_PASSWORD` no topo de `esp32_combined.ino` com as
   credenciais da tua rede antes do upload.
2. Após o boot, a porta série (115200 baud) imprime o IP atribuído, ex:
   `[WIFI] Ligado! IP: 192.168.1.42`
3. Abre `http://<IP>/` no browser — sliders para RPM (50–9000), MAP (0–300 kPa)
   e TPS (0–100%), com leitura em tempo real.

Endpoints HTTP (também usáveis via curl/script):
```
GET /              Página com sliders
GET /set?rpm=2500  Define RPM (também aceita map=, tps=, combináveis)
GET /status         {"rpm":...,"map":...,"tps":...} em JSON
```

O WiFi corre em paralelo ao protocolo série — ambos podem ser usados ao
mesmo tempo. Se a ligação WiFi falhar (timeout de 15s), o firmware continua
normalmente apenas por série.

## Build

### PlatformIO
```bash
cd tools/esp32_combined
pio run -e esp32dev          # Build
pio run -e esp32dev -t upload  # Upload
pio device monitor -b 115200   # Monitor
```

### Arduino IDE
Abrir `esp32_combined.ino` como sketch normal. As dependências (Preferences.h, rmt_tx.h, ledc.h) fazem parte do core ESP32 Arduino.

## GUI Electron

```bash
cd tools/esp32_combined/ardustim_gui
npm install
pip install esptool    # necessário para firmware upload
npm start
```

A GUI comunica via protocolo binário Ardu-Stim. Necessita `esptool.py` no PATH para
upload de firmware para o ESP32.

## Ficheiros

| Ficheiro | Descrição |
|---|---|
| `esp32_combined.ino` | Firmware principal (sketch único) |
| `web_page.h` | Página HTML/JS de controlo WiFi (RPM/MAP/TPS) |
| `platformio.ini` | Build config PlatformIO |
| `ardustim_gui/` | GUI Electron adaptada do Ardu-Stim |

## Test HIL

```bash
cd tools/hil_test
python hil_test.py --mode regression   # Regressão (texto serial)
python hil_test.py --mode timing       # Timing analysis 720°
```
