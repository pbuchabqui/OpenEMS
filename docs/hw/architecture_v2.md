# Arquitectura v2 — CIs dedicados no lugar do hub TLE8888

> **Decisão (2026-08-07).** A ECU passa de um hub único (TLE8888-2QK) para CIs dedicados
> por função. Origem: especificação externa, verificada bloco a bloco antes de adoptada.

## Os CIs

| Função | v2 | Substitui |
|---|---|---|
| PMIC / SBC | **TI TPS65381A-Q1** — pre-buck 6 V, LDO 3,3 V, LDO 5 V VTRACK ratiométrico, window watchdog | reguladores do TLE8888 |
| INJ / IGN | **NXP MC33810** — 4× low-side 4,5 A + 4× gate driver de bobina smart | direct drive do TLE8888 |
| ETB + EWG | **ST L9960T** — ponte dupla, PWM+DIR + SPI | BTS7960 (ETB) e EWG diferido |
| Wideband O2 | **Bosch CJ125** + LSU 4.9, aquecedor por HITFET BTS3134D | — (era WBO2 externo por CAN) |
| CAN | **FDCAN1 único** + expansor de I/O por SPI | transceiver do TLE8888 |

## ⚠️ O que esta decisão ANULA

Estas estavam fechadas e deixam de valer. Registado aqui para não voltarem por inércia:

| Decisão anulada | Era |
|---|---|
| TLE8888-2QK como hub | `a5f95fb` |
| INJ/IGN por direct drive do TLE | `a5f95fb` |
| CKP pela interface VR do TLE8888 | — |
| CAN pelo transceiver do TLE8888 | `netlist_v1.md` Bloco 9 |
| EWG diferido para a v2 | `34b40e3` — **agora é populado** |
| `-2QK` escolhida por trazer o watchdog desactivado | o TPS65381A tem window watchdog **obrigatório** |
| ~~Rejeição do DRV8701 por ser de 2 pinos~~ | o L9960T é igualmente 2 pinos e **foi aceite** |

## O que SOBREVIVE intacto

VGT6 / LQFP100 · H562 soldado · AMPSEAL 35+23 · montagem na cabine · USB isolado ·
4 camadas · VREF+ = VDDA filtrado · WeAct só bancada · knock diferido ·
**VBATT em `PC3`/INP13** · **scheduler congelado**.

## Porque o scheduler NÃO sai do congelamento

O MC33810 tem entradas paralelas directas para o caminho crítico — a mesma forma que
justificou o direct drive do TLE8888. Como o GPIOE tem 15 pinos bondados, os 8 inputs
ficam **nos mesmos pinos de hoje** (`PE0/2/4/6` + `PE9/11/13/15`).

`kOutBsrrPin[] = {4,6,0,2, 15,13,11,9}` **não muda**. `out_pin_write` mantém a escrita
BSRR única e atómica. O jitter de ~0,4 µs validado em hardware mantém-se.

## Três problemas que a spec original tinha e foram corrigidos

1. **`FDCAN2` não existe** no H562VGTx LQFP100 — derrubava o pilar da "rede auxiliar".
   Resolvido com barramento único + expansor por SPI.
2. **Matriz de pinos com 25 de 45 números errados**, incluindo `VBAT_SENSE` no `VDDA`.
   Descartada e refeita em [`pinout_v2.md`](pinout_v2.md), validada contra o symbol.
3. **CKP/CMP em `PE9`/`PE10`** — sem TIM2/TIM3 nesses pinos, e colidia com IGN1.
   Voltaram a `PA0`/`PA1`, onde o TIM5 os põe no timebase do scheduler.

## Dívida de firmware (enumerada, NÃO implementada)

Nada disto foi escrito. É um projecto próprio, a dimensionar deliberadamente.

| Item | Estado |
|---|---|
| `src/hal/tle8888.cpp` + `tle8888_regs.h` | **reformam-se** — ~20 KB de driver, reescrito contra o DS Rev 1.2 há dias |
| Driver TPS65381A-Q1 | novo — **e o window watchdog tem de ser servido**, ao contrário da `-2QK` |
| Driver MC33810 | novo — SPI de configuração; o caminho de timing continua a ser GPIO directo |
| Driver L9960T | novo |
| Driver CJ125 | novo — e `sensors.h` volta a ter caminho analógico de lambda (`o2_mv` tinha sido removido de propósito) |
| `etb_driver.cpp` | **reescrita** de 3 pinos (PWM+IN1+IN2) para 2 (PWM+DIR) + SPI |
| `ewg_driver.cpp` | tirar a guarda `EMS_EWG_POPULATED 0`, PID com realimentação real em `PA7` |
| `adc.h` | remapear canais; entram `LAMBDA_UA`/`UR`, `APP1`/`APP2`, `EWG_POS` |
| Expansor de I/O por SPI | novo, sem part number ainda |

## ✅ CKP decidido: Hall, igual ao CMP (2026-08-07)

Com o TLE8888 fora não há interface VR integrada — **CKP passa a Hall**, mesmo
condicionamento do CMP (divisor 10k/3,3k + RC + clamp dual BAT54BRW, da spec §7).
**Isto muda o sensor exigido ao motor**: uma roda dentada lida por VR deixa de servir:
precisa de sensor Hall (ou VR + condicionador externo fora da placa, não avaliado).

A polaridade de captura (front IC Hall = descida; TIM5 hoje só captura subida — ver
memória `cmp-ckp-capture-edge-polarity`) **continua em aberto** e agora aplica-se
também ao CKP, não só ao CMP: os dois canais têm a mesma pergunta.

## O que ficou por decidir

- Polaridade de captura CKP/CMP (acima).
- Part number do expansor de I/O.
- Se `VVT`/`PUMP`/`FAN` precisam de PWM real — o GPIOE não tem timer utilizável.
