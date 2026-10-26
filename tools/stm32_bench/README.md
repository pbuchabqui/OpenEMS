# OpenEMS STM32 Bench — gerador CKP/CMP + sensores + scope

Ferramenta de teste da OpenEMS (não é firmware da ECU). Porta o `tools/esp32_combined`
para um STM32H562RGT6 (WeAct 64 pinos), sem WiFi. Fala o mesmo protocolo serial, então
`tools/hil_test`, `tools/lib/stim_link.py` e `tools/lib/scope_link.py` devem funcionar
sem mudança por serial.

**Estado: compila (`pio run`); não foi gravado nem validado na bancada.**

## Ligações

Pinos da ECU sob teste = `BOARD=vgt6` (`src/hal/out_pins.h`). Se a ECU for `BOARD=rgt6`:
INJ PA15/PB3/PC10/PC11, IGN PC6-PC9.

| Gerador | ECU | Função |
|---|---|---|
| PA0 → | PA0 | CKP 60-2 (TIM5_CH1 AF2, toggle por compare) |
| PA1 → | PA1 | CMP, 1 pulso/720° no dente 5 (TIM5_CH2 AF2) |
| PA4 → | PA3 | MAP (DAC1_OUT1, direto) |
| PA5 → | PA4 | TPS (DAC1_OUT2, direto) |
| PA6 → | PB0 | CLT (TIM3_CH1, PWM + RC) |
| PA7 → | PB1 | IAT (TIM3_CH2, PWM + RC) |
| PB0 → | PC0 | APP1 (TIM3_CH3, PWM + RC) |
| PB1 → | PC1 | OIL (TIM3_CH4, PWM + RC) |
| PB6 → | PC4 | FUEL (TIM4_CH1, PWM + RC) |
| PB7 → | PA2 | ETB TPS1 (TIM4_CH2, PWM + RC) |
| PB8 → | PC5 | ETB TPS2, invertido (TIM4_CH3, PWM + RC) |
| PC0 ← | PE9 / PE11 / PE13 / PE15 | IGN1 (PC0), IGN2 (PC1), IGN3 (PC2), IGN4 (PC3) |
| PC4 ← | PE0 / PE2 / PE4 / PE6 | INJ1 (PC4), INJ2 (PC5), INJ3 (PB12), INJ4 (PB13) |
| GND | GND | obrigatório |

PWM+RC: série 10 kΩ + 100 nF para GND por canal. APP2 (PC2 da ECU) não tem saída própria.
Conferido no esquemático V1.1 da placa (`WeActStudio.STM32H5_64Pin_CoreBoard`, `Hardware/…V11 SchDoc.pdf`):
todos os pinos acima saem nos headers P1/P2 e nenhum colide com o que a placa usa
(LED azul PB2, tecla PC13, USB-C PA11/PA12, SWD PA13/PA14, slot microSD PC8-PC12/PD2/PA8).
Na H562, PB9 e PB11 são VCAP (NC) e não servem como I/O.

## Comandos (115200, USB CDC)

Iguais aos do `esp32_combined`: `RPM`, `SLEW`, `MAP`/`TPS`/`CLT`/`IAT`/`APP`/`FUEL`/`OIL`/`ETB`,
`IDLE`/`CRANK`/`CRUISE`/`WOT`/`COAST`, `PROFILE TAPER|STOP`, `STATUS` e as letras do scope
(`l e p w t s r ?`). Diferenças: `DAC1 <0-4095>` e `DAC2 <0-4095>` (código cru de 12 bits em
PA4 e PA5) substituem `DAC25`/`DAC26`.

## Build

```bash
cd tools/stm32_bench
pio run -e weact_h562rg
pio run -e weact_h562rg -t upload
pio device monitor -b 115200
```

O board `weact_stm32h5` do PlatformIO aponta para um variant que o framework instalado não
tem; o `platformio.ini` redireciona para `H562R(G-I)T/variant_WEACT_H562RG.h` e define
`ARDUINO_WEACT_H562RG`.

## O que mudou em relação ao esp32_combined

- **Gerador:** TIM5 de 32 bits a 1 MHz com CH1/CH2 em toggle por compare; a ISR só carrega
  o próximo CCR. CKP e CMP partilham o contador, então a fase CKP↔CMP não depende de RPM
  e o truque de arranque simultâneo do RMT não é necessário. O CMP continua 1 µs depois do CKP.
- **Rampa de RPM:** o período do dente é amostrado a cada dente (antes, a tabela do RMT era
  reescrita ao vivo). Sem o teto de 32767 µs do RMT, a rampa vale também abaixo de ~80 rpm.
- **Scope:** entradas por EXTI carimbadas com o mesmo TIM5. CKP/CMP entram no buffer direto da
  ISR do gerador, com o instante exato da borda (CCR), então não há fio de loopback.
  IGN3/IGN4 passam a ser medidos (no ESP32 colidiam com os DACs).
- **Sensores:** MAP/TPS passam de DAC de 8 bits para o DAC1 de 12 bits. As conversões estão em
  escala nominal com trims neutros (`kMapGainPermille` etc.): a calibração do ESP32 era do
  silício dele e não vale aqui.
- **WiFi/TCP/página web removidos.** `hil_test.py --stim tcp:...` e `usb_stress.py` não
  funcionam com esta ferramenta; use a porta serial.
- Corrigido: `+`, `-`, `0`-`9` só alteravam `g_rpm` e o gerador os ignorava.

## A validar na bancada

1. Forma de onda de CKP/CMP no analisador lógico (60-2, gap, CMP no dente 5, +1 µs).
2. Latência da ISR de TIM5 com USB ativo (meio dente a 9000 rpm ≈ 55 µs).
3. Faixa útil do DAC com o buffer ligado (ele pode não chegar aos trilhos) e os trims de MAP/TPS.
4. Que o `Serial.printf` do scope não bloqueia sem host USB conectado.
