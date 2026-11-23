# Placa de interface VGT6 — especificação do hardware adicional

> **Status:** vigente · validado 2026-11-23 (contra `src/hal/out_pins.h`, `src/hal/etb_driver.cpp`, `src/hal/etb_driver.h`, `src/hal/sd.cpp`, `src/drv/sensors.h`, `docs/hw/pinout.md` e as fichas do MAX9924, TPS2553, AP63205 e DRV8871)

Especificação de uma carrier que recebe a WeAct H562 VGT6 e coloca estágio de potência
e condicionamento atrás dos GPIOs. O esquemático KiCad (5 folhas) está em `hardware/interface_vgt6/`; `tools/interface_netcheck.py` confere os pinos de P1/P2 contra `interface_vgt6_nets.md`. PCB de 4 camadas: L1 sinal e potência, L2 plano `PGND`/`SGND` com junção única, L3 potência, L4 sinal (ainda sem layout). Substitui, para o escopo atual, os planos históricos de placa
(`interface_board_v1.md`, `decisoes_placa_historico.md`), que dependiam de CIs fora do projeto.

## Regras que não mudam

- **Fonte dos pinos:** `docs/hw/pinout.md` e `src/hal/out_pins.h`. Este documento não cria pino novo e não exige mudança de firmware.
- **INJ/IGN são active-high; estado seguro = LOW.** No reset a porta E fica em analógico (flutuante), então cada entrada de driver precisa de pull-down externo (`docs/hw/README.md`).
- **Bancada primeiro.** Nenhum injetor ou bobina real até: Rev X do chip confirmada (`REV_ID` em `DBGMCU_IDCODE`, já lido em 2026-11-17), estágio medido com carga resistiva/LED e relatório de bancada fechado (`docs/ROADMAP.md`, Fase 2).
- **Disponibilidade:** o estoque dos candidatos foi consultado em 2026-11-22 (`interface_vgt6_estoque.md`, perecível). Valores de tensão, corrente e encapsulamento dos candidatos de passivos e MOSFETs ainda vêm da memória do autor e precisam de conferência na ficha; MAX9924, TPS2553, AP63205 e DRV8871 foram conferidos nas fichas.

## Premissas elétricas (confirmar com o hardware real)

| Item | Premissa | Origem |
|---|---|---|
| Alimentação | 12 V nominal, 6–18 V de operação | `kVbattMinMv`/`kVbattMaxMv` em `src/drv/sensors.h` |
| Injetores | alta impedância (~12 Ω), saturados, sem peak-and-hold | firmware só liga/desliga INJ |
| Bobinas | com ignitor integrado, entrada lógica de 5 V | firmware só gera o pulso de dwell |
| CKP / CMP | CKP VR ou Hall (jumper); CMP Hall | decisão do usuário, 2026-11-22 |
| ETB | motor DC, PWM 10 kHz + IN1/IN2 (`etb_pwm_init(10000u)`) | `src/hal/etb_driver.cpp` |
| ADC | 3,3 V, 12 bits, só amostragem regular | `CLAUDE.md` (errata) |
| WBO2 | só por CAN, 500 kbps | `docs/hw/pinout.md`, `docs/ROADMAP.md` |

## Blocos

### 1. Alimentação

| Função | Candidato | Notas |
|---|---|---|
| Fusíveis + porta-fusível lâmina | ATO/ATM 15 A em cada uma das duas entradas (J1A, J1B) | fusíveis por banco: INJ 7,5 A, IGN 10 A, ETB 5 A, aux 10 A (`interface_vgt6_corrente.md`) |
| TVS | SMBJ20A (clamp ≈ 32 V, abaixo dos 40 V do PFET), **depois** do PFET de cada entrada | com a bateria invertida o PFET bloqueia e a TVS não conduz; antes do fusível ela conduziria em direto e queimaria o fusível. Surto acima de ≈ 24 V (jump start) a faz conduzir |
| Proteção de inversão | P-MOSFET 40 V ≥ 30 A (AOD4189) com 10 kΩ de gate para `PGND` e Zener de 15 V gate–fonte, uma por entrada | Schottky só se a queda for aceitável |
| Buck 12 V → 5 V | AP63205 (3,8–32 V, 2 A, 5 V fixo) | **não cobre load dump completo.** Em veículo, trocar por buck de 60 V (família LMR36015/TPS54360) |
| 5 V de sensores | 5 V do buck → ferrite → chave limitadora TPS2553 (SOT-23-6: IN 1, GND 2, EN 3, FAULT 4, ILIM 5, OUT 6) | `RILIM` 75 kΩ → 314–397 mA (`IOSmin`/`IOSmax` da ficha); curto no chicote não derruba o 5 V lógico; LED vermelho no `FAULT` |
| 5 V para a WeAct / 3,3 V | 5 V do buck entra no `VCC` do P4; o LDO da WeAct gera o 3,3 V | o D4 (Schottky) da WeAct bloqueia o retorno para o USB; não fechar o interruptor S1 (`VBUS`–`VCC`) com a carrier alimentando; **não ligar nada em `VB` (P2 pino 1), que é o backup do RTC** |
| VREF+ | só desacoplado na carrier (100 nF + 1 µF); a carrier **não** o alimenta | `VDDA` não sai nos headers; falta conferir na WeAct se `VREF+` já está ligado a `VDDA` (`weact_h562_schematic.md`) |

Aterramento: PGND (injetores, bobinas, ETB, relés) separado de SGND (sensores, MCU), unidos em um ponto junto à entrada. Retorno de bobina e de ETB nunca passa pelo plano de sensores.

### 2. Saídas INJ1–4 e IGN1–4 (PE0/2/4/6 e PE9/11/13/15)

Cadeia por canal: GPIO → buffer 5 V → R série → gate → carga.

- **Buffer:** 2× 74AHCT125 (quad, VIH ≤ 2 V a 5 V de VCC lê 3,3 V). Saída 5 V alimenta direto o gate do FET e a entrada do ignitor.
- **Seguro no reset:** entrada de cada buffer com 10 kΩ para GND, saída com 10 kΩ para GND, `OE#` com pull-up de 10 kΩ para 5 V. Buffer desabilitado = saída em alta impedância = LOW pelos pull-downs.
- **ARM:** chave/jumper puxa `OE#` para GND. LED vermelho acende com `OE#` baixo. Sem ARM, nada sai, mesmo com firmware em falha. O `PE14` lê `OE#` por um divisor 100 kΩ / 150 kΩ (≈ 2,9 V com `OE#` ≈ 4,8 V; um divisor de 10 kΩ carregaria o pull-up e leria abaixo de VIH); o firmware não usa isso hoje.
- **Injetor (INJ1–4):** MOSFET N 60–100 V, logic-level, DPAK (candidatos: IRLR2905 55 V como mínimo; IRLR3110Z 100 V com mais folga). R série de gate 220 Ω (durante o clamp ativo o 74AHCT125 segura o gate em LOW por esse resistor: 33 Ω faria o buffer absorver dezenas de mA; conferir o máximo absoluto de saída e o tempo de subida do gate na bancada), pull-down de gate 10 kΩ, TVS de grampo dreno–gate (ativo) ou dreno–GND com tensão de grampo ≤ 80% de VDSS. Retorno do injetor ao fusível de 7,5 A (`interface_vgt6_corrente.md`).
- **Ignição (IGN1–4):** só o buffer, 33 Ω série, 10 kΩ para GND na saída, TVS/diodo contra surto no conector. A potência fica no ignitor da bobina, alimentada pelo fusível de 15 A.
- **Latência:** 74AHCT125 ~5–8 ns + FET ~100–300 ns, atraso fixo. Medir na bancada com `tools/ign_jitter.py` (T7d) nos pinos do conector, não só nos pinos do MCU.

### 3. ETB (PE5 PWM, PE7 IN1 abrir, PE8 IN2 fechar)

O firmware entrega PWM + dois sinais de direção (`etb_dir_set`, `etb_pwm_set_duty_x10`). Pontes de 2 pinos (IN1/IN2) pedem combinar os três:

- **Ponte H:** DRV8871 (45 V, 3,6 A de pico, HSOP-8, limite de corrente por resistor, IN1/IN2). `RILIM` = 20 kΩ: `ITRIP = 64 / RILIM(kΩ)` dá 3,2 A nominal e 2,95–3,45 A com a tolerância de `VILIM` (59–69) da ficha, sempre abaixo dos 3,6 A de pico. Se o corpo de borboleta travado passar de 3,6 A, subir para DRV8873 (10 A, mais pinos, com IPROPI e nFAULT).
- **Combinação:** duas portas AND (2 × 74AHCT1G08; a biblioteca do KiCad não tem o quad): `IN1_ponte = PWM & DIR_abrir`, `IN2_ponte = PWM & DIR_fechar`. Com PWM em LOW as duas entradas ficam em 0 e a ponte vai a *coast*; é o estado seguro, e a mola do corpo de borboleta leva a lâmina à posição de emergência.
- **Seguro no reset:** 10 kΩ para GND em PE5, PE7 e PE8 e em cada entrada da ponte; `OE#` do ARM também desabilita o AND (buffer 74AHCT125 antes das portas AND).
- **Realimentação:** TPS1/TPS2 (PA2/PC5) pelo condicionamento do bloco 5. Os limites do firmware são `ETB_TPS_ADC_MIN/MAX` = 200/3900 e `ETB_TPS_NORMAL_MIN/MAX` = 400/3700 (`src/hal/etb_driver.h`).
- ⚠️ O comentário em `etb_driver.cpp` cita BTS7960 (limite 25 kHz). A escolha acima não depende dele; se a ponte for outra, o limite de PWM é o dela. O firmware fica em 10 kHz.

### 4. Auxiliares

| Função | Pino | Circuito |
|---|---|---|
| Bomba de combustível | PE10 | MOSFET N logic-level (AO3400A ou 2N7002 se a bobina < 150 mA) + diodo de roda livre (1N4148W ou SS14) + relé 12 V; 10 kΩ de gate para GND |
| Ventoinha | PE12 | idem, relé de 30 A |
| VVT admissão/escape | PB6 / PB7 (TIM4) | AO3400A (especificado para gate de 2,5 V, acionado direto pelos 3,3 V do GPIO) + 100 Ω + 10 kΩ de gate + diodo Schottky SS34 de roda livre (PWM, não só TVS) |
| Pedido de A/C | PB11 | **inacessível:** o `PB11` não sai em nenhum header da WeAct (P1/P2); a entrada de A/C não pode ser ligada pela carrier sem fio direto ao MCU. Fica fora desta revisão |
| Sensor flex fuel | PB5 | pull-up de 3,3 kΩ para 5 V de sensores → Schmitt 74LVC1G17 (entrada tolerante a 5 V) → PB5 |

### 5. Entradas analógicas (ADC 0–3,3 V)

Regra geral: sensor de 5 V ratiométrico → divisor 10 kΩ/20 kΩ (0,5–4,5 V vira 0,33–3,0 V) → R série 1 kΩ → capacitor ao GND → clamp BAT54S para 3V3/GND. Impedância de fonte baixa para a amostragem regular do ADC.

| Canal | Pino | Condicionamento | Banda (cap) |
|---|---|---|---|
| MAP | PA3 | divisor + RC | ≈ 2 kHz (10 nF); precisa seguir a pulsação do coletor |
| TPS | PA4 | divisor + RC | ≈ 500 Hz |
| ETB_TPS1 / ETB_TPS2 | PA2 / PC5 | divisor + RC | ≈ 500 Hz |
| APP1 / APP2 | PC0 / PC2 | divisor + RC (conferir a curva do pedal) | ≈ 500 Hz |
| CLT / IAT | PB0 / PB1 | pull-up de 2,49 kΩ para 3V3 (valor que o ROADMAP usa) + 100 nF | ≈ 100 Hz |
| FUEL / OIL | PC4 / PC1 | divisor + RC, transdutor 0,5–4,5 V | ≈ 100 Hz |
| VBATT | PC3 | 100 kΩ / 22 kΩ (18 V → 3,25 V) + 100 nF | ≈ 100 Hz |

O firmware converte `vbatt_raw_to_mv` com 18 V = fundo de escala (3,3 V); o divisor 100 k / 22 k lê ≈ 1,6 % abaixo (17,7 V em 18 V). Aceitável; calibrar se precisar.
| KNOCK | PA5 | **DNP.** Piezo → capacitor de acoplamento → polarização em VDDA/2 → amplificador rail-to-rail (MCP6002 ou LMV358) → passa-baixa. `EMS_KNOCK_HW_PRESENT` fica 0 até o estágio ser montado e testado | — |

⚠️ Divisor de 2/3 e a curva do sensor real mudam a escala que o firmware assume para FUEL/OIL e para as curvas de CLT/IAT. Caracterize o sensor e grave a escala pela página `0x0D` (`tools/ntc_gen.py`); o `docs/ROADMAP.md` (Fase 3) já descreve o caminho.

### 6. CKP (VR ou Hall) e CMP (Hall) — PA0, PA1

**CKP aceita os dois tipos de sensor**, escolhidos por jumper de 3 vias (`JP_CKP`: 1-2 = VR, 2-3 = Hall). Só um caminho alimenta o PA0 por vez. O J5 tem pinos separados para `CKP_VR+`, `CKP_VR-` e `CKP_HALL`.

- **Caminho VR:** **MAX9924** (µMAX-10), **modo A2** (Fig. 3 da ficha): `ZERO_EN`, `INT_THRS` e `BIAS` em GND, referência interna de 2,5 V, `EXT` e `N.C.` abertos. Limiar adaptativo de 1/3 do pico anterior e comparação no zero-crossing. Entradas: 10 kΩ em série em cada uma (limita a corrente nos diodos ESD internos; máximo absoluto ±20 mA nos demais pinos) e 1 nF entre `IN+` e `IN-`. `VCC` em `+5V_SENS` (faixa 4,5–5,5 V) com 10 nF, 100 nF e 1 µF. **Saída `COUT` em dreno aberto**: pull-up de 10 kΩ para `+3V3`, então o PA0 nunca vê 5 V. Um CKP VR não pode ligar direto ao MCU: a amplitude vai de centenas de mV a dezenas de volts.
- **Caminho Hall:** pull-up de 4,7 kΩ para 5 V de sensores (jumper `JP_CKP_PU` para retirar se a saída for push-pull), R série de 1 kΩ, 100 pF, Schmitt 74LVC1G17 alimentado em 3,3 V (entrada tolerante a 5 V), ESD no conector.
- **Saída do jumper → PA0:** XOR 74LVC1G86 com `JP_POL` (fechado inverte a borda), depois 1 kΩ série. O caminho não selecionado fica desligado pelo jumper: o `COUT` em repouso sobe a 3,3 V pelo pull-up e o Schmitt não força nada. Mantém o pull-down interno do PA0.
- **CMP (PA1): Hall**, mesmo estágio do caminho Hall do CKP (pull-up fixo de 4,7 kΩ, R série, Schmitt), sem condicionador VR e sem jumper de polaridade.
- **Filtro do timer:** o filtro de entrada (≈256 ns) continua valendo como segunda camada.
- ⚠️ **Polaridade e ângulo:** o MAX9924 comuta no zero-crossing e a borda que o TIM5 captura depende do tipo de sensor e da ligação; o Hall tem a polaridade do próprio sensor. Meça no scope qual borda cai no centro do dente em cada modo e confira `src/drv/ckp.cpp` e o offset angular. O XOR com `JP_POL` existe para o caso de a polaridade diferir.
- **Disponibilidade:** o MAX9924 é de um fornecedor só (Analog Devices); estoque melhor em Mouser/DigiKey que em LCSC (`interface_vgt6_estoque.md`). A ficha está em `docs/hw/maxim_max9924_max9927.pdf`.
- **Atraso:** o estágio adiciona atraso fixo (comparador, XOR e RC); entra no erro angular medido (T7d).

### 7. CAN (PB8 RX, PB9 TX)

TJA1051T/3 (alimentado em 5 V, VIO em 3,3 V; candidato de alta disponibilidade), choke de modo comum (ACM2012, pares 1–4 e 2–3), TVS de CAN (SM712 no esquemático; PESD2CAN ou similar), terminação de 120 Ω com jumper `JP_TERM`, pino S (standby) com 10 kΩ para GND. Conector para o WBO2 (CAN-H/CAN-L, 12 V, GND). Bit rate de 500 kbps; o RX do WBO2 e o TX 0x400/0x401/0x402 ainda não foram validados em hardware (`docs/ROADMAP.md`).

### 8. SD

O slot microSD fica na própria WeAct VGT6 (SDMMC1: D0..D3 = PC8..PC11, CK = PC12, CMD = PD2, `src/hal/sd.cpp`); a carrier não leva slot, pull-ups nem alimentação de SD. Só garantir que **PC8..PC12 e PD2 não sejam usados nem carregados** pela carrier: esses pinos não vão a nenhum conector. A existência do slot na placa não consta nos documentos da WeAct deste repositório (`weact_h562_schematic.md`); confirmar no PDF e na placa física.

### 9. USB e SWD

USB-C da WeAct fica como está. Quando houver potência na mesma bancada, isole o laço do PC com **módulo** isolador USB de baixo custo (ex.: base ADuM3160) em vez de integrar isolação na carrier. SWD e reset: `PA13`, `PA14` e `NRST` não saem em P1/P2; só existem no header P5 da WeAct (2×4) e nos botões. A carrier não leva SWD próprio: use o P5 da WeAct por cabo.

### 10. Conectores e mecânica

- Bancada: bornes plugáveis (passo 5,08 mm para potência; 3,5/3,81 mm para sinal), de linha comum e disponibilidade ampla. Veículo (futuro): AMPSEAL 23/35, decisão separada.
- Furos M3 (Φ3,2 mm) e recorte conforme `weact_h562_coreboard.md`. A altura com USB e pinos ainda deve ser medida na placa real.
- Fiação dos headers: P1/P2 conforme `weact_h562_schematic.md` (cruzar a numeração do pino 1 com o silkscreen físico antes de fabricar).

## Pinos usados e livres

| Grupo | Pinos |
|---|---|
| INJ1–4 | PE0, PE2, PE4, PE6 |
| IGN1–4 | PE9, PE11, PE13, PE15 |
| ETB | PE5 (PWM), PE7, PE8 |
| Relés | PE10 (bomba), PE12 (ventoinha) |
| Entradas | PA0 CKP, PA1 CMP, PB5 flex, ADC (bloco 5) |
| Comunicação | PB8/PB9 CAN, PA9/PA10 UART, PA11/PA12 USB |
| VVT | PB6, PB7 |
| Livres | PE3, PE14 (reserva para leitura do ARM), PB12–PB15 (SPI2), PD8–PD15, PB10 (EWG diferido) |

## Esquemático e conferência

O esquemático KiCad 10 (5 folhas) está em `hardware/interface_vgt6/`, gerado por `python3 hardware/interface_vgt6/gen/build.py`. **O gerador é de uma passada:** depois de editar no KiCad, não rode de novo sobre os mesmos arquivos. Os símbolos do MAX9924 e do TPS2553 (ausentes da biblioteca oficial) estão em `hardware/interface_vgt6/OpenEMS.kicad_sym`. Conferência: `kicad-cli sch erc hardware/interface_vgt6/interface_vgt6.kicad_sch` e `python3 tools/interface_netcheck.py` (pinos de P1/P2 contra `interface_vgt6_nets.md`, `PGND`/`SGND` separadas, unidades dos 74AHCT125). Nenhum dos dois confere valores, polaridade de peça nem pinagem de footprint.

## Ordem de validação do hardware

1. Sem MCU encaixado: tensões de 5 V, 5 V de sensores e 3,3 V, inversão de polaridade, fusíveis.
2. Com a WeAct, ARM aberto: todas as saídas em LOW (multímetro e scope), inclusive durante reset e reprogramação por DFU/SWD.
3. ARM fechado, cargas falsas (resistor + LED): forma de onda, tempo e jitter dos 8 canais; ETB sem motor, depois com motor solto e o limite de corrente da ponte medido.
4. Sensores: ADC com valores conhecidos, conferir escalas e curvas de CLT/IAT.
5. Só então injetor e bobina reais, depois do gate de bancada do `docs/ROADMAP.md`.

## Em aberto

- Corrente de travamento do motor do corpo de borboleta e curva real dos pedais (decidem DRV8871 ou DRV8873 e o divisor de APP).
- Carga de cada saída auxiliar (relés, VVT) para dimensionar fusível e MOSFET.
- Necessidade de load dump completo (decide o buck de 60 V).
- Se o ARM deve passar a ser lido ou comandado pelo firmware (mudança de código e de documento).
- Preço real da BOM e estoque dos itens de menor risco (o estoque consultado está em `interface_vgt6_estoque.md`).
