# OpenEMS — Netlist da placa de interface v1 (VGT6)

> **O que este ficheiro é:** a camada de **ligação** — cada net nomeada, cada ligação pino-a-pino
> enumerada. É o que se desenha no KiCad sem ter de re-derivar nada, e é o artefacto que a
> **revisão pré-layout** consome.
>
> **O que este ficheiro NÃO é:** justificação de projeto. O *porquê* de cada bloco vive em
> `docs/hw/interface_board_v1.md` (blocos 1–18) e o esquemático elétrico em `docs/wiring_diagram.md`.
> **Não repetir aqui o raciocínio** — autoridade duplicada foi o que produziu o mapa de registadores
> inventado do TLE8888 e a linha stale do pull-down do `PA0`.

## Níveis de confiança dos nomes de pino — ler antes de desenhar

| Tier | O que é | Estado |
|---|---|---|
| **A — verificado** | Pinos do STM32 | ✅ `src/hal/out_pins.h` / `adc.h`. |
| **A — TLE package** | Números LQFP-100 do TLE8888 | ✅ **Datasheet Rev 1.2 §3** + símbolo rusEFI `tle8888qk.lib`. Tabela completa: **`tle8888_pinout.md`**. |
| **C — externo** | LDO, buck, TVS, BTS7960, headers AMPSEAL | ⚠️ Pinout de cada datasheet próprio. Marcado `TBD` onde não confirmado. |

⚠️ **Numeração de cavidade dos AMPSEAL:** as posições abaixo são **alocação lógica**. O mapeamento para
a numeração real de cavidade (3 filas) sai do desenho da TE. Manter o agrupamento; a ordem pode mudar.

---

## Convenção de nomes de net

| Prefixo | Significado |
|---|---|
| `VBAT_*` | Rails de bateria (12 V nominal) |
| `+5V_*` | Alimentação de sensor (trackers do TLE8888) |
| `+3V3`, `VDDA`, `VREF_P` | Rails do MCU |
| `*_GND` | Terras separados (ver bloco 18) |
| `N_*` | Nets internas de nó (divisores, filtros) |
| `J1_*` / `J2_*` | Nets que terminam no conector |

**Terras — quatro pours, unidos num único ponto estrela:** `PGND` (potência) · `SGND` (sinal) ·
`AGND` (referência do ADC, no `VSSA`) · `SHIELD_GND` (blindagens).

---

## Bloco 1 — Árvore de alimentação

| De | Para | Net | Notas |
|---|---|---|---|
| `J2.VBAT+` (×2) | dreno do P-MOSFET | `VBAT_RAW` | 2 pinos em paralelo |
| P-MOSFET | fusível 30 A | `VBAT_PROT` | proteção de polaridade invertida |
| fusível | rail | `VBAT` | bulk 100 µF + 100 nF; **TVS SMBJ24CA** para `PGND` |
| `VBAT` | `TLE8888` **pin 54** BAT (+ **87/90** BATPA/B) | `VBAT` | bulk local 100 µF |
| `VBAT` | entrada do buck | `VBAT` | |
| buck (**≥500 kHz, baixo ripple** — `TBD`, p.ex. TPS54302) | — | `+5V_MAIN` | 10 µF in / 22 µF + 100 nF out |
| `+5V_MAIN` | entrada do LDO | — | |
| **LDO low-noise / high-PSRR** (`TBD`) | — | `+3V3` | ⚠️ **Não AMS1117** (bloco 1) |
| `+3V3` | `MCU.VDD` ×N | `+3V3` | **100 nF por pino VDD** |
| `+3V3` | ferrite → `MCU.VDDA` | `VDDA` | 1 µF ∥ 100 nF |
| `VDDA` | `MCU.VREF+` | `VREF_P` | **VREF+ = VDDA filtrado** (opção (a)) |
| — | `MCU.VSSA` | `AGND` | pour dedicado |
| `TLE8888` **pin 9** `T5V1` | rail de sensor A | `+5V_SENS_A` | tracker |
| `TLE8888` **pin 10** `T5V2` | rail de sensor B | `+5V_SENS_B` | tracker |
| `TLE8888` **pin 20** `VDDIO` | `+3V3` | — | lógica I/O + FCLN strap |
| `TLE8888` PGND **25,50,75** + tab | `PGND` | — | pour potência |

**DNP reservado — opção (c) de VREF+** (ver pendência 3): divisor `+5V_SENS_A` → ~3,0 V + buffer de
baixa impedância → `VREF_P`. Footprint sim, **não popular**.

---

## Bloco 2 — CKP (VR, interface do TLE8888)

| De | Para | Net |
|---|---|---|
| `J1.CKP+` | `TLE8888` **pin 52** `VRIN1` | `CKP_P` |
| `J1.CKP−` | `TLE8888` **pin 51** `VRIN2` | `CKP_N` |
| `J1.CKP_SHLD` | `SHIELD_GND` | `SHIELD_GND` |
| `TLE8888` **pin 21** `VROUT` | **TP-DIG** → jumper 0 Ω → `MCU.PA0` | `CKP_DIG` |

- ⚠️ **Sem rede externa** de resistor série + clamp: o clamp de entrada (50 mA) é interno ao CI.
- ⚠️ **Sem pull-up** em `CKP_DIG` — `VROUT` é **push-pull**. O **pull-down interno do `PA0` mantém-se**
  (é o fix de falso-sync; ver `interface_board_v1.md`).
- **TP-VR**: ponto de teste no par diferencial, antes do CI, para a caracterização do passo 3.
- **Jumper 0 Ω** entre `VROUT` e `PA0`: removido, liberta o nó para o estimulador ESP32.

---

## Bloco 3 — CMP (Hall)

| De | Para | Net |
|---|---|---|
| `J1.CMP_5V` | `+5V_SENS_A` | `+5V_SENS_A` |
| `J1.CMP_GND` | `SGND` | `SGND` |
| `J1.CMP_SIG` | nó de pull-up | `N_CMP_RAW` |
| `+5V_SENS_A` | `N_CMP_RAW` | pull-up **10 k** |
| `N_CMP_RAW` | divisor → `N_CMP_DIV` | **10 k / 15 k** |
| `N_CMP_DIV` | `MCU.PA1` | `CMP_DIG` — RC leve + **TVS 3,3 V** |

🚨 **ABERTO — não desenhar sem fechar:** a polaridade de captura do CMP (e do CKP) está **adiada** à
espera do part number do Hall. Hall open-collector idle HIGH/pulso LOW põe o dente na borda de
**descida**, mas `TIM5` captura só subida. Ver `interface_board_v1.md`. O pull interno do `PA1` muda
para **pull-up** no mesmo movimento — **o `PA0` não muda**.

---

## Blocos 4–5 — Injeção e ignição (direct drive)

Pinos do MCU **tier A**, de `out_pins.h`. Atribuição `IN`→`OUT` é **fixa no silício**.

| Canal | MCU | TLE pin / símbolo | TLE out (package) | Conector |
|---|---|---|---|---|
| INJ1 | `PE0` | **28** IN1 | **59+60** OUT1A+OUT1B (juntos) | `J2.INJ1` |
| INJ2 | `PE2` | **29** IN2 | **61+62** OUT2A+B | `J2.INJ2` |
| INJ3 | `PE4` | **30** IN3 | **63+64** OUT3A+B | `J2.INJ3` |
| INJ4 | `PE6` | **31** IN4 | **65+66** OUT4A+B | `J2.INJ4` |
| IGN1 | `PE9` | **32** IN5 | **96** IGN1 | `J2.IGN1` |
| IGN2 | `PE11` | **33** IN6 | **97** IGN2 | `J2.IGN2` |
| IGN3 | `PE13` | **34** IN7 | **98** IGN3 | `J2.IGN3` |
| IGN4 | `PE15` | **35** IN8 | **99** IGN4 | `J2.IGN4` |
| **INJEN** | `PE1` | **24** INJEN | — | — |
| **IGNEN** | `PE3` | **27** IGNEN | — | — |

- **Sem resistores de pull-down de gate**: os `IN*` têm pull-down interno.
- **Sem clamps de flyback** nos injetores: clamp integrado.
- `OUT1–OUT4` = low-side (A+B em paralelo) → **injetores HI-Z**.
- `IGN1–4` = push-pull **gate IGBT** (20 mA) — validar bobina; não assumir smart-coil 5 V.
- 🚨 **`+12 V` de injetores e bobinas NÃO passa pela placa** — relé no chicote.

---

## Bloco 6 — Analógicos

**Rede padrão** (ratiométricos 0,5–4,5 V): `J1.<sig>` → divisor **10 k / 15 k** → RC **1 k + 100 nF** →
**TVS 3,3 V** → pino do ADC. Retorno em `SGND`; TVS para `AGND`.

| Sinal | MCU (tier A) | Canal | Alimentação | Rede |
|---|---|---|---|---|
| MAP | `PA3` | ADC1 INP15 | `+5V_SENS_A` | padrão ⚠️ ver nota de banda |
| TPS (indep.) | `PA4` | ADC1 INP18 | `+5V_SENS_A` | padrão — **reserva** |
| APP1 | `PC0` | ADC1 INP10 | `+5V_SENS_A` | padrão |
| APP2 | `PC2` | ADC1 INP12 | `+5V_SENS_B` | padrão — ⚠️ **rail diferente do APP1**, para que uma falha de rail não desloque os dois em conjunto e derrote a plausibilidade |
| ETB_TPS1 | `PA2` | ADC1 INP14 | `+5V_SENS_A` | padrão |
| ETB_TPS2 | `PC5` | ADC1 INP8 | `+5V_SENS_B` | padrão |
| P. combustível | `PC4` | ADC2 INP4 | `+5V_SENS_B` | padrão |
| P. óleo | `PC1` | ADC2 INP11 | `+5V_SENS_B` | padrão |
| CLT (NTC) | `PB0` | ADC2 INP9 | pull-up **2,49 k** → `+5V_SENS_A` | + rede padrão |
| IAT (NTC) | `PB1` | ADC2 INP5 | pull-up **2,49 k** → `+5V_SENS_A` | + rede padrão |
| **VBATT** | `PC3` | ADC2 INP13 | — | **interno**: divisor `VBAT` 0–18 V → 0–3,3 V + RC generoso |
| Knock | `PA5` | ADC1 INP19 | — | ❌ **diferido v2** — só footprint, ver bloco 13 |

⚠️ **MAP:** o RC de 1 k + 100 nF dá fc ≈ 1,6 kHz. Verificar contra a banda desejada — o MAP tem conteúdo
rápido por pulsação de coletor, e filtrar demais atrasa a resposta transitória.
⚠️ **VBATT não vai ao conector** — mede o rail interno.

---

## Bloco 8 — ETB (BTS7960 @ 10 kHz)

| De | Para | Net | Notas |
|---|---|---|---|
| `MCU.PE5` (TIM15_CH1 AF4) | `BTS7960.PWM` | `ETB_PWM` | **10 kHz** |
| `MCU.PE7` | `BTS7960.IN1` | `ETB_DIR1` | abrir |
| `MCU.PE8` | `BTS7960.IN2` | `ETB_DIR2` | fechar; **ambos LOW = travagem** |
| `BTS7960.OUT1` | `J2.ETB_M+` ×2 | `ETB_MOTOR_P` | ⚠️ **2 pinos** — stall 8–10 A |
| `BTS7960.OUT2` | `J2.ETB_M−` ×2 | `ETB_MOTOR_N` | ⚠️ **2 pinos** |
| `VBAT` | `BTS7960.VCC` | `VBAT` | alimentação separada, retorno próprio ao ponto estrela |
| — | `BTS7960.GND` | `PGND` | |

Realimentação de posição pelo bloco 6 (`ETB_TPS1`/`ETB_TPS2`), **par afastado do par de potência**.
🚨 O gate de segurança do ETB (mola default-closed, corte de energia duro, batente mecânico, validação
em bancada antes do motor) é requisito de **projeto e de procedimento** — ver bloco 8 do plano.

---

## Bloco 9 — CAN (transceiver integrado)

| De | Para | Net |
|---|---|---|
| `MCU.PB9` (FDCAN1_TX AF9) | `TLE8888` **pin 44** `CANTX` | `CAN_TX` |
| `MCU.PB8` (FDCAN1_RX AF9) | `TLE8888` **pin 43** `CANRX` | `CAN_RX` |
| `TLE8888` **pin 46** `CANH` | `J1.CANH` | `CANH` |
| `TLE8888` **pin 47** `CANL` | `J1.CANL` | `CANL` |
| `CANH`–`CANL` | resistor **120 Ω** com jumper | — |
| `J1.CAN_SHLD` | `SHIELD_GND` | `SHIELD_GND` |
| `+5V` adequado | **pin 45** `V5VCAN` | supply CAN (ver DS) |

---

## Blocos 10–11 — Relés

| De | Para | Net | Package |
|---|---|---|---|
| `MCU.PE10` | IN9 | `PUMP_CMD` | **pin 36** |
| `MCU.PE12` | IN10 | `FAN_CMD` | **pin 37** |
| OUT14 | `J2.PUMP_RLY` | `PUMP_RLY_LS` | **pin 68** |
| OUT15 | `J2.FAN_RLY` | `FAN_RLY_LS` | **pin 67** |
| MR | `J2.MAIN_RLY` | `MAIN_RLY_LS` | **pin 55** — **DNP v1** (key-on) |

Saídas LS para bobina de relé (~200 mA); clamp interno. InConfig/DD no firmware.

---

## Bloco 14 — Flex fuel

| De | Para | Net |
|---|---|---|
| `J1.FLEX_12V` | `VBAT` | `VBAT` |
| `J1.FLEX_GND` | `SGND` | `SGND` |
| `J1.FLEX_SIG` | divisor **10 k / 3,3 k** | `N_FLEX_RAW` |
| `+5V_SENS_B` | `N_FLEX_RAW` | pull-up **10 k** (se coletor aberto) |
| divisor | `MCU.PB5` (EXTI) | `FLEX_DIG` — **TVS 3,3 V**, filtrar com folga |

---

## Bloco 15 — VVT

| De | Para | Net | Package |
|---|---|---|---|
| `MCU.PB6` (TIM4_CH1) | IN11 | `VVT_EXH_CMD` 15 Hz | **pin 38** |
| `MCU.PB7` (TIM4_CH2) | IN12 | `VVT_INT_CMD` 15 Hz | **pin 39** |
| OUT5A+B+C | `J2.VVT_EXH` | `VVT_EXH_LS` | **83+84+85** (juntos) |
| OUT6A+B+C | `J2.VVT_INT` | `VVT_INT_LS` | **92+93+94** (juntos) |

LS 4,5 A + clamp activo. `+12 V` dos solenóides no chicote. Comissionar só o came instrumentado.

---

## Bloco 12 — SPI2 → TLE8888 (single-ended, DS §SPI)

| MCU | Função | TLE pin | Símbolo |
|---|---|---|---|
| `PB12` | CS | **3** | CSN |
| `PB13` | SCK AF5 | **7** | FCLP |
| `PB14` | MISO AF5 | **4** | SDO |
| `PB15` | MOSI AF5 | **5** | SIP |
| — | mode select | **6** SIN | **→ AGND** |
| — | mode select | **8** FCLN | **→ VDDIO (pin 20 / +3V3)** |
| +3V3 | I/O supply | **20** | VDDIO |

🚨 **Strapping SPI obrigatório:** SIN=AGND e FCLN=VDDIO. Sem isto o CI fica em MSC/LVDS e o
SPI parece morto. `PB12`/`PB13` só SPI (bomba/fan em PE10/PE12).

---

## Bloco 16 — USB isolado

| De | Para | Net |
|---|---|---|
| `MCU.PA11` | isolador, lado MCU | `USB_DM` |
| `MCU.PA12` | isolador, lado MCU | `USB_DP` |
| isolador, lado externo | conector USB | `USB_DM_ISO` / `USB_DP_ISO` |
| DC-DC isolado | lado externo | `+5V_USB_ISO` / `USB_GND_ISO` |

⚠️ Isolador **junto ao conector**, não junto ao MCU. Barreira de isolamento **sem pour de terra por
baixo** — é o ponto onde um layout descuidado anula o componente.

---

## Bloco 13 — Knock (footprint, NÃO popular na v1)

`J1.KNOCK_SIG` → footprint TPIC8101 → `MCU.PA5`. Chip select próprio no SPI2 (`TBD`).
Trilha de `PA5` roteada até ao footprint; `J1.KNOCK_SHLD` → `SHIELD_GND`.
⚠️ Front-end tem de entregar **envelope**, não portadora (ADC amostra 1×/dente, abaixo de Nyquist).
Colocar **longe das linhas de bobina e dos solenóides VVT** já na v1 — reservar o espaço custa nada.

---

## Pinos do MCU sem função na v1 — deixar livres

A regra "cada pino tem uma casa" corta nos dois sentidos: além de nenhum pino ficar solto, **nenhum pino
pode ser acionado sem destino**. Esta varredura encontrou um caso real.

| Pino | Situação |
|---|---|
| `PA7`, `PD3`, `PB10` | **EWG** — ✅ **corrigido**. `ewg_driver_init()` reclamava `PA7`/`PD3` como saídas e punha `PB10` em PWM TIM2_CH3, apesar de o EWG estar diferido. Pior: com a realimentação fixada em 0, o PID via erro = demanda − 0 e, ao subir a demanda, **conduzia** os três pinos a fundo — para um estágio que não existe. Guardado com `EMS_EWG_POPULATED 0`, mesmo padrão de `sdmmc_init()`. **Deixar sem ligação na v1.** |
| `PB2` | LED de heartbeat **do coreboard WeAct** — não sai para a placa carrier. Não usar. |
| `PA9`, `PA10` | USART1 de bancada. Não cablados na v1; **reservar como pontos de teste**, não encaminhar ao conector. |
| `PC8`, `PC12`, `PD2` | SDMMC — `sdmmc_init()` nunca é chamado e está guardado. Livres no VGT6, mas **não reutilizar** sem rever o datalog. |

---

## Conector — mapa completo

### J2 — AMPSEAL `770680-1`, 23 vias (potência/atuadores)

| Pos | Net | Notas |
|---|---|---|
| 1, 2 | `VBAT+` | alimentação da placa |
| 3, 4, 5 | `PGND` | |
| 6–9 | `INJ1`–`INJ4` | low-side, ~1 A |
| 10–13 | `IGN1`–`IGN4` | trigger lógico 20 mA |
| 14 | `VVT_EXH` | LS |
| 15 | `VVT_INT` | LS |
| 16 | `PUMP_RLY` | bobina |
| 17 | `FAN_RLY` | bobina |
| 18 | `MAIN_RLY` | bobina |
| 19, 20 | `ETB_MOTOR_P` | ⚠️ duplicado |
| 21, 22 | `ETB_MOTOR_N` | ⚠️ duplicado |
| 23 | *livre* | |

### J1 — AMPSEAL `776164-1`, 35 vias (sinais)

| Pos | Net | Notas |
|---|---|---|
| 1 | `CKP_P` | par trançado blindado |
| 2 | `CKP_N` | |
| 3 | `CKP_SHLD` | dreno só no lado da ECU |
| 4 | `CMP_SIG` | |
| 5 | `CMP_5V` | |
| 6 | `CMP_GND` | |
| 7 | `MAP` | |
| 8 | `CLT` | |
| 9 | `IAT` | |
| 10 | `APP1` | |
| 11 | `APP2` | |
| 12 | `FUEL_PRESS` | |
| 13 | `OIL_PRESS` | |
| 14 | `ETB_TPS1` | |
| 15 | `ETB_TPS2` | |
| 16 | `+5V_SENS_A` | |
| 17 | `+5V_SENS_B` | |
| 18, 19 | `SGND` | |
| 20 | `CANH` | |
| 21 | `CANL` | |
| 22 | `CAN_SHLD` | |
| 23 | `FLEX_12V` | |
| 24 | `FLEX_SIG` | |
| 25 | `FLEX_GND` | |
| 26 | `KNOCK_SIG` | ❌ reservado — **não cablar v1** |
| 27 | `KNOCK_SHLD` | ❌ reservado |
| 28 | `CMP2_SIG` | ❌ reservado — 2º came (VVT dual) |
| 29 | `CMP2_5V` | ❌ reservado |
| 30 | `CMP2_GND` | ❌ reservado |
| 31 | `TPS_INDEP` | ❌ reservado |
| 32–35 | *livres* | |

---

## Verificação de fecho — o que a revisão pré-layout tem de confirmar

Esta é a propriedade que faz disto um esquemático e não prosa: **cada pino tem exatamente uma casa.**

- [ ] **Todo o pino de conector termina numa net**, e toda a net chega a um pino de componente.
      Nenhum pino solto em J1/J2 exceto os marcados *livres*/*reservados*.
- [ ] **Toda a saída do TLE8888 usada tem via de conector** — e nenhuma via de conector aponta para uma
      `OUT` que não foi habilitada em `OEConfig`.
- [ ] **Toda a entrada de sensor tem alimentação e retorno**: `+5V_SENS_*` chega a cada sensor
      ratiométrico, e cada um tem `SGND`.
- [ ] **APP1 e APP2 em rails diferentes** — senão a plausibilidade não deteta falha de rail.
- [ ] **Todo o pino de ADC usado em `adc.h` tem rede** — cruzar a tabela do bloco 6 contra
      `AdcPrimaryChannel` / `AdcSecondaryChannel`. Nenhum canal configurado sem rede, nenhuma rede
      sem canal.
- [ ] **Todo o pino de `out_pins.h` está desenhado** — 8 canais + `INJEN` + `IGNEN`.
- [ ] `PB12`/`PB13` **não** reclamados por nada além do SPI2.
- [ ] Os quatro terras existem como pours separados e unem-se **num único ponto**.
- [ ] Nenhum `+12 V` de injetor, bobina ou solenóide atravessa a placa.

### Bloqueios que têm de fechar ANTES do layout

**1. ✅ Polaridade de borda CMP/CKP — implementada.**
page0[258]: bit0=CKP falling, bit1=CMP falling. Default 0 = subida + pull-down.
`apply_page0_capture_polarity()` no boot (após NVM) e na UI; `tim5_ic_set_capture_polarity()`
faz CCxE→CCxP→CCxE e PUPDR em conjunto. Speeduino: `TrigEdge` / Hall “drags to ground” →
muitas vezes **RISING** no MCU se a rede inverte; Hall OC idle-HIGH típico pede **falling** + pull-up
no bit CMP. **Não bloqueia layout** do condicionamento Hall (pull-up externo no esquema).

**2. ✅ Números físicos TLE8888 LQFP-100** — `docs/hw/tle8888_pinout.md` (DS Rev 1.2 + rusEFI lib).

**3. ✅ Coreboard WeAct** — 38,62×69,10 mm, furos Φ3,2 — `weact_h562_coreboard.md`.  
   ⚠️ **AMPSEAL** housings: desenhos TE ainda em falta.

**4. ⚠️ Part numbers `TBD`:** buck, LDO, isolador USB, DC-DC isolado; bobinas (IGN = gate IGBT 20 mA).

**5. ✅ Relé principal v1:** key-on directo; `J2.MAIN_RLY` → pin 55 MR **DNP**.

**6. 🚨 No esquemático SPI:** SIN(6)=AGND, FCLN(8)=VDDIO — senão o hub não clocka.

---

## ✅ Bug page0 56–76 — **corrigido**

Os bytes 56–76 (trims, cmp_window, anti-jerk, rev limiter, ckp skip) passam por
`apply_page0_trims_driveability()` no boot (`main_stm32`) e na escrita UI.
