# Placa de interface VGT6 — especificação do hardware adicional

> **Status:** vigente · validado 2026-11-22 (contra `src/hal/out_pins.h`, `src/hal/etb_driver.cpp`, `src/hal/etb_driver.h`, `src/hal/sd.cpp`, `src/drv/sensors.h` e `docs/hw/pinout.md`)

Especificação (sem arquivo de EDA) de uma carrier que recebe a WeAct H562 VGT6 e coloca estágio de potência
e condicionamento atrás dos GPIOs. Substitui, para o escopo atual, os planos históricos de placa
(`interface_board_v1.md`, `decisoes_placa_historico.md`), que dependiam de CIs fora do projeto.

## Regras que não mudam

- **Fonte dos pinos:** `docs/hw/pinout.md` e `src/hal/out_pins.h`. Este documento não cria pino novo e não exige mudança de firmware.
- **INJ/IGN são active-high; estado seguro = LOW.** No reset a porta E fica em analógico (flutuante), então cada entrada de driver precisa de pull-down externo (`docs/hw/README.md`).
- **Bancada primeiro.** Nenhum injetor ou bobina real até: Rev X do chip confirmada (`REV_ID` em `DBGMCU_IDCODE`, já lido em 2026-11-17), estágio medido com carga resistiva/LED e relatório de bancada fechado (`docs/ROADMAP.md`, Fase 2).
- **Disponibilidade:** nenhum estoque foi consultado ao escrever isto. Os códigos abaixo são candidatos de famílias comuns; confirme estoque (LCSC, Mouser, DigiKey) e a ficha técnica antes de fixar a BOM. Valores de tensão, corrente e encapsulamento dos candidatos vêm da memória do autor e precisam de conferência.

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
| Fusível geral + porta-fusível lâmina | ATO/ATM 20 A | fusíveis por banco: INJ 5 A, IGN 15 A, ETB 7,5 A, aux 15 A |
| TVS de entrada | SMBJ24A ou SMDJ24A | grampo de surto; o fusível protege na inversão |
| Proteção de inversão | P-MOSFET 40 V ≥ 30 A (ex.: AOD4189) + Zener de gate 15 V | Schottky só se a queda for aceitável |
| Buck 12 V → 5 V | AP63205 (3,8–32 V, 2 A, 5 V fixo) | **não cobre load dump completo.** Em veículo, trocar por buck de 60 V (família LMR36015/TPS54360) |
| 5 V de sensores | 5 V do buck → ferrite → chave limitadora TPS2553 | curto no chicote não derruba o 5 V lógico; LED de status |
| 3,3 V | LDO da própria WeAct | conferir no PDF/placa física se o pino 5 V da WeAct tem diodo contra backfeed do USB; se não tiver, Schottky em série (MBR0520) |
| VREF+ | ligado a VDDA (opção (a) de `weact_h562_schematic.md`) | ferrite + 100 nF + 1 µF |

Aterramento: PGND (injetores, bobinas, ETB, relés) separado de SGND (sensores, MCU), unidos em um ponto junto à entrada. Retorno de bobina e de ETB nunca passa pelo plano de sensores.

### 2. Saídas INJ1–4 e IGN1–4 (PE0/2/4/6 e PE9/11/13/15)

Cadeia por canal: GPIO → buffer 5 V → R série → gate → carga.

- **Buffer:** 2× 74AHCT125 (quad, VIH ≤ 2 V a 5 V de VCC lê 3,3 V). Saída 5 V alimenta direto o gate do FET e a entrada do ignitor.
- **Seguro no reset:** entrada de cada buffer com 10 kΩ para GND, saída com 10 kΩ para GND, `OE#` com pull-up de 10 kΩ para 5 V. Buffer desabilitado = saída em alta impedância = LOW pelos pull-downs.
- **ARM:** chave/jumper puxa `OE#` para GND. LED vermelho acende com `OE#` baixo. Sem ARM, nada sai, mesmo com firmware em falha. `PE3` e `PE14` ficam livres para uma leitura futura de `OE#`; este documento não exige isso.
- **Injetor (INJ1–4):** MOSFET N 60–100 V, logic-level, DPAK (candidatos: IRLR2905 55 V como mínimo; IRLR3110Z 100 V com mais folga). R série de gate 22–33 Ω, pull-down de gate 10 kΩ, TVS de grampo dreno–gate (ativo) ou dreno–GND com tensão de grampo ≤ 80% de VDSS. Retorno do injetor ao fusível de 5 A.
- **Ignição (IGN1–4):** só o buffer, 33 Ω série, 10 kΩ para GND na saída, TVS/diodo contra surto no conector. A potência fica no ignitor da bobina, alimentada pelo fusível de 15 A.
- **Latência:** 74AHCT125 ~5–8 ns + FET ~100–300 ns, atraso fixo. Medir na bancada com `tools/ign_jitter.py` (T7d) nos pinos do conector, não só nos pinos do MCU.

### 3. ETB (PE5 PWM, PE7 IN1 abrir, PE8 IN2 fechar)

O firmware entrega PWM + dois sinais de direção (`etb_dir_set`, `etb_pwm_set_duty_x10`). Pontes de 2 pinos (IN1/IN2) pedem combinar os três:

- **Ponte H:** DRV8871 (45 V, 3,6 A de pico, SO-8, limite de corrente por resistor, IN1/IN2). Se o corpo de borboleta travado passar de 3,6 A, subir para DRV8873 (10 A, mais pinos, com IPROPI e nFAULT).
- **Combinação:** 74AHCT08 (quad AND): `IN1_ponte = PWM & DIR_abrir`, `IN2_ponte = PWM & DIR_fechar`. Com PWM em LOW as duas entradas ficam em 0 e a ponte vai a *coast*; é o estado seguro, e a mola do corpo de borboleta leva a lâmina à posição de emergência.
- **Seguro no reset:** 10 kΩ para GND em PE5, PE7 e PE8 e em cada entrada da ponte; `OE#` do ARM também desabilita o AND (use buffer 74AHCT125 antes do 74AHCT08).
- **Realimentação:** TPS1/TPS2 (PA2/PC5) pelo condicionamento do bloco 5. Os limites do firmware são `ETB_TPS_ADC_MIN/MAX` = 200/3900 e `ETB_TPS_NORMAL_MIN/MAX` = 400/3700 (`src/hal/etb_driver.h`).
- ⚠️ O comentário em `etb_driver.cpp` cita BTS7960 (limite 25 kHz). A escolha acima não depende dele; se a ponte for outra, o limite de PWM é o dela. O firmware fica em 10 kHz.

### 4. Auxiliares

| Função | Pino | Circuito |
|---|---|---|
| Bomba de combustível | PE10 | MOSFET N logic-level (AO3400A ou 2N7002 se a bobina < 150 mA) + diodo de roda livre (1N4148W ou SS14) + relé 12 V; 10 kΩ de gate para GND |
| Ventoinha | PE12 | idem, relé de 30 A |
| VVT admissão/escape | PB6 / PB7 (TIM4) | MOSFET como o do injetor + diodo Schottky de roda livre (PWM, não só TVS) |
| Pedido de A/C | PB11 | entrada 12 V → divisor 47 k / 22 k → 10 nF → clamp; o firmware já liga pull-down interno e debounce de 50 ms |
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
| KNOCK | PA5 | **DNP.** Piezo → capacitor de acoplamento → polarização em VDDA/2 → amplificador rail-to-rail (MCP6002 ou LMV358) → passa-baixa. `EMS_KNOCK_HW_PRESENT` fica 0 até o estágio ser montado e testado | — |

⚠️ Divisor de 2/3 e a curva do sensor real mudam a escala que o firmware assume para FUEL/OIL e para as curvas de CLT/IAT. Caracterize o sensor e grave a escala pela página `0x0D` (`tools/ntc_gen.py`); o `docs/ROADMAP.md` (Fase 3) já descreve o caminho.

### 6. CKP (VR ou Hall) e CMP (Hall) — PA0, PA1

**CKP aceita os dois tipos de sensor**, escolhidos por jumper de 3 vias (`JP_CKP`: 1-2 = VR, 2-3 = Hall). Só um caminho alimenta o PA0 por vez.

- **Caminho VR:** **MAX9924** (single, limiar adaptativo por pico + zero-crossing) com o par trançado do sensor, resistores série e clamp conforme a ficha, e 100 nF + 1 µF junto ao CI em 5 V de sensores. Um CKP VR não pode ligar direto ao MCU: a amplitude vai de centenas de mV a dezenas de volts.
- **Caminho Hall:** pull-up de 4,7 kΩ para 5 V de sensores (jumper `JP_CKP_PU` para retirar se a saída for push-pull), R série de 1 kΩ, ~100 pF, Schmitt 74LVC1G17 alimentado em 3,3 V (entrada tolerante a 5 V).
- **Saída do jumper → PA0:** 1 kΩ série. O caminho não selecionado fica desligado: o MAX9924 sem sinal e o Schmitt com o pull-up retirado não devem forçar o pino; conferir na ficha o nível de saída em repouso do MAX9924. Mantém o pull-down interno do PA0.
- **CMP (PA1): Hall**, mesmo estágio do caminho Hall do CKP (pull-up, R série, Schmitt 74LVC1G17), sem condicionador VR. Entrada com TVS/ESD no conector.
- **Saída do MAX9924:** conferir na ficha se é push-pull ou dreno aberto. Em dreno aberto, pull-up de 4,7 kΩ para 3V3; em push-pull a 5 V, passar pelo Schmitt (3,3 V) para o PA0 não ver 5 V.
- **Filtro do timer:** o filtro de entrada (≈256 ns) continua valendo como segunda camada.
- ⚠️ **Polaridade e ângulo:** o MAX9924 comuta no zero-crossing e a borda que o TIM5 captura depende do tipo de sensor e da ligação; o Hall tem a polaridade do próprio sensor. Meça no scope qual borda cai no centro do dente em cada modo e confira `src/drv/ckp.cpp` e o offset angular. O mesmo firmware vale para os dois, mas a borda pode precisar de inversão por jumper (XOR com `JP_POL`, 74LVC1G86) se a polaridade diferir.
- **Disponibilidade:** o MAX9924 é de um fornecedor só (Analog Devices); estoque melhor em Mouser/DigiKey que em LCSC. Não consultei estoque; os detalhes de modo e saída vêm da memória e precisam de conferência na ficha.
- **Atraso:** o estágio adiciona atraso fixo; entra no erro angular medido (T7d).

### 7. CAN (PB8 RX, PB9 TX)

TJA1051T/3 (alimentado em 5 V, VIO em 3,3 V; candidato de alta disponibilidade), choke de modo comum, TVS de CAN (PESD2CAN ou similar), terminação de 120 Ω com jumper, pino S (standby) com 10 kΩ para GND. Conector para o WBO2 (CAN-H/CAN-L, 12 V, GND). Bit rate de 500 kbps; o RX do WBO2 e o TX 0x400/0x401/0x402 ainda não foram validados em hardware (`docs/ROADMAP.md`).

### 8. SD (SDMMC1 na VGT6)

Slot microSD push-push com detecção de cartão. Pinos: D0..D3 = PC8..PC11, CK = PC12, CMD = PD2 (`src/hal/sd.cpp`). Pull-up de 47 kΩ em D0..D3 e CMD, 100 nF + 10 µF perto do slot, 22–33 Ω série no CK. O clock de dados vai até 25 MHz no modo padrão; comprimento de trilha curto. Só existe na VGT6 (na RGT6 esses pinos são INJ/IGN).

### 9. USB e SWD

USB-C da WeAct fica como está. Quando houver potência na mesma bancada, isole o laço do PC com **módulo** isolador USB de baixo custo (ex.: base ADuM3160) em vez de integrar isolação na carrier. SWD em header de 6 vias (3V3, SWDIO PA13, SWCLK PA14, GND, NRST).

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
| Entradas | PA0 CKP, PA1 CMP, PB5 flex, PB11 A/C, ADC (bloco 5) |
| Comunicação | PB8/PB9 CAN, PA9/PA10 UART, PA11/PA12 USB, SD (bloco 8) |
| VVT | PB6, PB7 |
| Livres | PE3, PE14 (reserva para leitura do ARM), PB12–PB15 (SPI2), PD8–PD15, PB10 (EWG diferido) |

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
- Estoque e preço reais da BOM; nada foi consultado.
