# OpenEMS ESP32 Combined — gerador CKP/CMP + sensores + scope

Gerador de sinais para a bancada HIL do OpenEMS (WeAct H562 VGT6): CKP/CMP 60-2
por RMT, MAP/TPS por DAC, sensores lentos por PWM e scope lógico de INJ/IGN.
Roteiro de bancada: `docs/bench_test_manual.md`.

## Ligações

| ESP32 | STM32 (VGT6) | Função |
|---|---|---|
| GPIO 2 → | PA0 | CKP (60-2, RMT) |
| GPIO 4 → | PA1 | CMP (1 pulso/720°, dente 5, RMT) |
| GPIO 2 → GPIO 34 | — | loopback do CKP para o scope |
| GPIO 26 → | PA3 | MAP (DAC2, direto) |
| GPIO 25 → | PA4 | TPS (DAC1, direto) |
| GPIO 21 → | PB0 | CLT (PWM + RC) |
| GPIO 22 → | PB1 | IAT (PWM + RC) |
| GPIO 23 → | PC0 | APP1 (PWM + RC) |
| GPIO 17 → | PC1 | OIL (PWM + RC) |
| GPIO 18 → | PC4 | FUEL (PWM + RC) |
| GPIO 16 → | PA2 | ETB TPS1 (PWM + RC) |
| GPIO 19 → | PC5 | ETB TPS2, invertido (PWM + RC) |
| GPIO 32 ← | PE9 | IGN1 |
| GPIO 33 ← | PE11 | IGN2 |
| GPIO 27 ← | PE0 | INJ1 |
| GPIO 14 ← | PE2 | INJ2 |
| GPIO 12 ← | PE4 | INJ3 |
| GPIO 13 ← | PE6 | INJ4 |
| GND | GND | obrigatório |

PWM+RC: série 10 kΩ + 100 nF para GND por canal. IGN3/IGN4 não têm entrada livre
(GPIO25/26 são os DACs): use o osciloscópio. APP2 (PC2) não tem saída própria;
só ligue o mesmo fio se a calibração do APP2 aceitar a mesma tensão. Pinos STM32
= `src/hal/out_pins.h` (`BOARD=vgt6`). Nenhum GPIO está ao mesmo tempo no scope e
no PWM (GPIO 12/13/14 eram os dois até o conflito ser corrigido).

## Comandos (115200 baud, ou TCP 3333 por WiFi; mesmo parser)

```
RPM <n>                    RPM exato (50-9000), com rampa
SLEW <n>                   passo máximo da rampa, % por 50 ms (1-30, padrão 2)
MAP <kPa> / TPS <%> / CLT <C> / IAT <C> / APP <%> / FUEL <bar*10> / OIL <bar*10> / ETB <%>
IDLE | CRANK | CRUISE | WOT | COAST    presets de RPM + sensores
PROFILE TAPER | PROFILE STOP           partida -> 1700 rpm -> 850 rpm (taper do ETB)
DAC25 <0-255> / DAC26 <0-255>          tensão crua nos DACs
STATUS                                 estado
l e p w t d s r ?                      scope (live, edges, pulsos, waveform, timing, 720, stats, reset, ajuda)
```

`PROFILE TAPER`: CRANK 200 rpm por 3 s, depois 1700 rpm por 1 s, depois 850 rpm. Cada
fase imprime `[PROFILE] <fase> t=<millis>` para correlacionar com a telemetria do dash.
A rampa padrão (2 %/50 ms) leva ~5,4 s de 200 a 1700 rpm, bem mais devagar que uma partida
real: aumente com `SLEW` e confira se o STM mantém o sync (passo maior que 2 % pode perder
sync; valor útil se mede na bancada).

## O que este gerador NÃO cobre

Sem emulação de WBO2 (CAN 0x180) nem de velocidade de roda/marcha por CAN: a malha de
lambda (STFT/LTFT) e o TC não são testáveis só com este ESP32.

## WiFi

Copie `wifi_credentials.example.h` para `wifi_credentials.h` (ignorado pelo git) e
ligue a `src/` se usar PlatformIO. O IP aparece na serial. Página com slider de RPM em
`http://<IP>/` (`/s?r=<rpm>` define, `/s` lê); comandos de texto em TCP 3333.

## Build

```bash
cd tools/esp32_combined
pio run -e esp32dev            # compila
pio run -e esp32dev -t upload  # grava
pio device monitor -b 115200
```

## Teste HIL

```bash
cd tools/hil_test
python hil_test.py --mode regression
python hil_test.py --mode timing
```
