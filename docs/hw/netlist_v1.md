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
| **A — verificado** | Pinos do STM32 | ✅ Saem de `src/hal/out_pins.h` e `src/hal/adc.h`, que **compilam e correm**. Usar como estão. |
| **B — funcional** | Pinos do TLE8888 (`IN1`, `OUT1`, `VRIN1`, …) | ⚠️ Os **nomes** são conhecidos e corretos. Os **números físicos do LQFP-100 NÃO estão verificados** — a mesma proveniência de "pino 24/27" para INJEN/IGNEN. **Tirar da tabela de pinout do datasheet ao desenhar.** Não inventar. |
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
| `VBAT` | `TLE8888.VBAT` | `VBAT` | bulk local 100 µF |
| `VBAT` | entrada do buck | `VBAT` | |
| buck (**≥500 kHz, baixo ripple** — `TBD`, p.ex. TPS54302) | — | `+5V_MAIN` | 10 µF in / 22 µF + 100 nF out |
| `+5V_MAIN` | entrada do LDO | — | |
| **LDO low-noise / high-PSRR** (`TBD`) | — | `+3V3` | ⚠️ **Não AMS1117** (bloco 1) |
| `+3V3` | `MCU.VDD` ×N | `+3V3` | **100 nF por pino VDD** |
| `+3V3` | ferrite → `MCU.VDDA` | `VDDA` | 1 µF ∥ 100 nF |
| `VDDA` | `MCU.VREF+` | `VREF_P` | **VREF+ = VDDA filtrado** (opção (a)) |
| — | `MCU.VSSA` | `AGND` | pour dedicado |
| `TLE8888.DVT5V1` | rail de sensor A | `+5V_SENS_A` | tracker, ±10 mV |
| `TLE8888.DVT5V2` | rail de sensor B | `+5V_SENS_B` | tracker |

**DNP reservado — opção (c) de VREF+** (ver pendência 3): divisor `+5V_SENS_A` → ~3,0 V + buffer de
baixa impedância → `VREF_P`. Footprint sim, **não popular**.

---

## Bloco 2 — CKP (VR, interface do TLE8888)

| De | Para | Net |
|---|---|---|
| `J1.CKP+` | `TLE8888.VRIN1` | `CKP_P` |
| `J1.CKP−` | `TLE8888.VRIN2` | `CKP_N` |
| `J1.CKP_SHLD` | `SHIELD_GND` | `SHIELD_GND` |
| `TLE8888.VROUT` | **TP-DIG** → jumper 0 Ω → `MCU.PA0` | `CKP_DIG` |

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

| Canal | MCU | TLE8888 in | TLE8888 out | Conector |
|---|---|---|---|---|
| INJ1 | `PE0` | `IN1` | `OUT1` | `J2.INJ1` |
| INJ2 | `PE2` | `IN2` | `OUT2` | `J2.INJ2` |
| INJ3 | `PE4` | `IN3` | `OUT3` | `J2.INJ3` |
| INJ4 | `PE6` | `IN4` | `OUT4` | `J2.INJ4` |
| IGN1 | `PE9` | `IN5` | `IGN1` | `J2.IGN1` |
| IGN2 | `PE11` | `IN6` | `IGN2` | `J2.IGN2` |
| IGN3 | `PE13` | `IN7` | `IGN3` | `J2.IGN3` |
| IGN4 | `PE15` | `IN8` | `IGN4` | `J2.IGN4` |
| **INJEN** | `PE1` | `INJEN` | — | — |
| **IGNEN** | `PE3` | `IGNEN` | — | — |

- **Sem resistores de pull-down de gate**: os `IN*` têm fonte de corrente de pull-down interna.
- **Sem clamps de flyback** nos injetores: clamp integrado por canal.
- `OUT1–OUT4` = low-side 2,2 A → **injetores de alta impedância**.
- `IGN1–IGN4` = driver de gate push-pull 20 mA → **smart coils** (entrada lógica).
- 🚨 **`+12 V` de injetores e bobinas NÃO passa pela placa** — vem do relé, no chicote. Ver a correção
  no bloco do conector.

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
| `MCU.PB9` (FDCAN1_TX AF9) | `TLE8888.TXD` | `CAN_TX` |
| `MCU.PB8` (FDCAN1_RX AF9) | `TLE8888.RXD` | `CAN_RX` |
| `TLE8888.CANH` | `J1.CANH` | `CANH` |
| `TLE8888.CANL` | `J1.CANL` | `CANL` |
| `CANH`–`CANL` | resistor **120 Ω** com jumper | — |
| `J1.CAN_SHLD` | `SHIELD_GND` | `SHIELD_GND` |

---

## Blocos 10–11 — Relés

| De | Para | Net |
|---|---|---|
| `MCU.PE10` | `TLE8888.IN9` | `PUMP_CMD` |
| `MCU.PE12` | `TLE8888.IN10` | `FAN_CMD` |
| `TLE8888.OUT<bomba>` | `J2.PUMP_RLY` | `PUMP_RLY_LS` |
| `TLE8888.OUT<ventoinha>` | `J2.FAN_RLY` | `FAN_RLY_LS` |
| `TLE8888.<main relay drv>` | `J2.MAIN_RLY` | `MAIN_RLY_LS` |

Saídas low-side para a **bobina** do relé (~200 mA). Sem diodos de roda-livre discretos — clamp interno.
⚠️ `OUT` exatos e o driver de relé principal: **tier B**, confirmar no datasheet.

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

| De | Para | Net |
|---|---|---|
| `MCU.PB6` (TIM4_CH1) | `TLE8888.IN11` | `VVT_EXH_CMD` — **15 Hz** |
| `MCU.PB7` (TIM4_CH2) | `TLE8888.IN12` | `VVT_INT_CMD` — **15 Hz** |
| `TLE8888.OUT5` | `J2.VVT_EXH` | `VVT_EXH_LS` |
| `TLE8888.OUT6` | `J2.VVT_INT` | `VVT_INT_LS` |

Low-side 4,5 A com **clamp ativo 50–60 V** → sem drivers nem diodos discretos. `+12 V` dos solenóides
vem do relé, **no chicote**.
⚠️ Comissionar **só o came instrumentado** (os dois PIDs partilham o único CMP).

---

## Bloco 12 — SPI2 → TLE8888

| MCU (tier A) | Função | TLE8888 |
|---|---|---|
| `PB12` | GPIO out | `CSN` |
| `PB13` | SPI2_SCK AF5 | `SCLK` |
| `PB14` | SPI2_MISO AF5 | `SDO` |
| `PB15` | SPI2_MOSI AF5 | `SDI` |

🚨 **`PB12`/`PB13` não podem ser reclamados por mais nada** — bomba e ventoinha migraram para
`PE10`/`PE12` exatamente por isto. Sem SPI2 o TLE8888 não configura e o fingerprint dá máscara cheia.

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

**1. 🚨 Polaridade de borda CMP/CKP — tem saída conhecida, por implementar.**
Já não é preciso esperar pelo part number do Hall: a polaridade pode ser um **byte de calibração**,
o que tira a decisão do caminho crítico. Desenho apurado em 2026-07-20:
- 1 byte em page0, **offset ≥ 258** (byte mais alto usado é 257; ≥258 fica coberto pelo gate
  `cal_layout_ok` do byte 175, logo blobs antigos leem 0). `bit0` = CKP, `bit1` = CMP.
  **Default `0` = subida = comportamento actual.**
- Nova `tim5_ic_set_capture_polarity()`: **limpar `CCxE` → escrever `CCxP` → repor `CCxE`** (trocar a
  polaridade com a captura activa pode latch-ar uma captura espúria).
- ⚠️ **O pull tem de seguir a polaridade** — com captura na descida o nível seguro inverte e o default
  passa a **pull-up**. Deixar PUPDR dessincronizado reintroduz o falso-sync.
- ⚠️ **Duas armadilhas:** (a) a config carrega **depois** do `tim5_ic_init()` (`main_stm32.cpp:482` vs
  `:513`), logo é preciso re-aplicar no bloco §5; (b) o caminho de boot **salta os bytes 56-65**, ver
  o bug latente abaixo.
- Impacto angular absorvido: o gate temporal do CMP é **invariante à polaridade** e o gate de posição
  **re-ancora sozinho** (±3 dentes). Só `cmp_window_open/close_tooth` é referência fixa — e está
  desligado por defeito.

**2. 🚨 Números físicos de pino do TLE8888** — tier B, tirar da tabela de pinout do datasheet.

**3. ⚠️ Dimensões do coreboard e das caixas AMPSEAL** — ver `README.md` §5 para o que medir.

**4. ⚠️ Part numbers `TBD`:** buck, LDO, isolador USB, DC-DC isolado.

---

## 🐛 Bug latente pré-existente (independente desta placa)

Descoberto ao desenhar o item 1 acima, e **não corrigido**:

**Os bytes 56-65 de page0 não são restaurados da flash no arranque.** `cmp_window_open_tooth`,
`cmp_window_close_tooth` e os trims de combustível por cilindro são serializados e aplicados na escrita
por UI, mas `grep cmp_window src/main_stm32.cpp` → **0 ocorrências**. Ou seja: configuram-se, gravam-se
em flash, e **não sobrevivem a um reboot**.

Corrigir em separado — não é específico da placa de interface, mas qualquer knob novo que copie este
padrão herda o defeito.
