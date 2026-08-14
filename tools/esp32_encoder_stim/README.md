# OpenEMS ESP32 Encoder Stim

Estimulador de bancada para firmware `EMS_MT6835_ENCODER=1` **sem chip MT6835**.

Gera quadratura **A/B** (4096 PPR → 16384 counts/volta no TIM2) e **CMP**
(1 pulso / 720° em PC6), mais sensores analógicos no protocolo StimLink.

O `esp32_combined` / `esp32_stimulator` (CKP 60-2 em PA0/PA1) **não** serve
para este build — pinos e domínio de sincronismo são outros.

## Cablagem mínima (VGT6)

```
ESP32 GPIO2  → STM32 PA0   (quadratura A — TIM2_CH1)
ESP32 GPIO4  → STM32 PA1   (quadratura B — TIM2_CH2)
ESP32 GPIO5  → STM32 PC6   (CMP came — TIM3_CH1, captura na descida)
ESP32 GND    → STM32 GND   (obrigatório)
```

### Sensores (opcional — mesma tabela do stimulator legado)

| ESP32 | STM32 | Sinal | Notas |
|-------|-------|-------|-------|
| GPIO25 | PA3 | MAP | DAC, ligação directa |
| GPIO26 | PA4 | TPS | DAC, ligação directa |
| GPIO13 | PB0 | CLT | LEDC + RC 10 kΩ / 100 nF |
| GPIO12 | PB1 | IAT | LEDC + RC |
| GPIO14/27 | APP | APP1/2 | LEDC + RC |
| GPIO16/19 | ETB | ETB_TPS1/2 | LEDC + RC |
| GPIO18/17 | PC0/PC5 | FUEL/OIL | LEDC + RC |

### Scope opcional (IGN/INJ)

| ESP32 | STM32 | Canal |
|-------|-------|-------|
| GPIO32 | PE9 | IGN1 |
| GPIO33 | PE11 | IGN2 |
| GPIO34 | PE0 | INJ1 (GPIO input-only) |
| GPIO35 | PE2 | INJ2 (GPIO input-only) |

IGN3/4 e INJ3/4 não cabem sem remapar LEDC — use osciloscópio ou segundo ESP32.

## Firmware STM32

`EMS_MT6835_ENCODER=1 make firmware-vgt6` **não liga a flag** — o Makefile
nunca a referencia, o build corre em silêncio como produção. Editar
`src/hal/board_pinout.h:39-40` temporariamente (`#define
EMS_MT6835_ENCODER 0` → `1`):

```bash
make clean && WERROR=1 make firmware-vgt6
# MT6835_HW_PRESENT=0 (default) — sem SPI; TIM2 conta só AB externo
# reverter board_pinout.h antes de commitar (git diff deve ficar vazio)
```

Após DFU: **power-cycle** (não só `:leave`).

## Flash do ESP32

Arduino IDE / PlatformIO, board ESP32 Dev Module, upload
`esp32_encoder_stim.ino`.

WiFi (opcional):

```bash
cp wifi_credentials.example.h wifi_credentials.h
# editar SSID/PASS → TCP :3333 (StimLink)
```

## Comandos (serial 115200 ou TCP :3333)

Compatível com [`tools/lib/stim_link.py`](../lib/stim_link.py):

| Comando | Efeito |
|---------|--------|
| `RPM <50-6000>` | Velocidade da quadratura |
| `CMP_TOOTH <0-16383>` | Offset do flanco CMP no ciclo 720° |
| `MAP` `TPS` `CLT` `IAT` `APP` `FUEL` `OIL` `ETB` | Sensores |
| `IDLE` `CRANK` `CRUISE` `WOT` `COAST` | Presets |
| `STATUS` / `SCOPE` / `?` | Diagnóstico |

Default `CMP_TOOTH=8192` (meio da 2.ª volta de cambota).

## Validação

### Osciloscópio (só ESP32)

1. `RPM 600` → frequência em A ≈ `4096 × 10 = 40,96 kHz`.
2. CMP: 1 pulso a cada `2 × (60/600) = 0,2 s` (idle HIGH, LOW ~200 µs).

### STM32 + ESP32

1. Ligar AB + CMP + GND; flash encoder; `IDLE` / `RPM 800`.
2. Snapshot UART: `rpm_x10 > 0` (HALF_SYNC sem fase calibrada).
3. Comando `D`: `late_event_count` TIM2 estável em regime.
4. Scope PE9/PE0: wasted-spark / inj em presync.

### Smoke HIL

```bash
python3 tools/hil_test/hil_encoder_test.py \
  --stm32 /dev/ttyACM0 --stim /dev/ttyUSB0 --bench-clt-iat
```

PASS: ping ECU, RPM > 0, late estável em IDLE ~30 s.

## Limites

- **RPM máx 6000** (`kRpmMax`); se jitter RMT falhar na bancada, baixar para 4000.
- Sem SPI MT6835: `TIM2_CNT` arranca em 0 (sem absoluto no key-on).
- Sequencial fino (`FULL_SYNC`) exige `EMS_MT6835_CMP_PHASE_CALIBRATED=1` medido + `CMP_TOOTH` alinhado.

## Relação com outros tools

| Tool | Uso |
|------|-----|
| `esp32_stimulator` / `esp32_combined` | CKP 60-2 + CMP em PA0/PA1 — build **sem** encoder |
| **`esp32_encoder_stim`** | AB + CMP PC6 — build **com** `EMS_MT6835_ENCODER=1` |
| Módulo MT6835 real | Bring-up final (SPI + AB do sensor) — ver fork doc |
