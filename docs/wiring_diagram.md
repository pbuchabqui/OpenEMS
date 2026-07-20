# OpenEMS — Esquemático Elétrico

> **Âmbito deste ficheiro:** alimentação, condicionamento de sinal, atuadores externos,
> **conector** e terra. É o que não cabe no mapa de pinos.
>
> **NÃO é fonte de pinout, de registadores nem de capacidade de saída.** Essas vivem em:
> | O quê | Onde |
> |---|---|
> | Mapa de pinos (RGT6 vs VGT6) | `docs/hw/pinout.md` |
> | INJ/IGN, enables, BSRR | `src/hal/out_pins.h` |
> | Registadores do TLE8888 | `src/hal/tle8888_regs.h` (verificado vs Rev 1.2) |
> | Arquitetura da placa, blocos, BOM | `docs/hw/interface_board_v1.md` |
>
> ⚠️ **Porque esta separação é levada a sério:** a versão anterior deste ficheiro
> continha um mapa de pinos ASCII (INJ em TIM2, IGN em TIM8) e uma descrição dos
> registadores/capacidades do TLE8888 — **ambos inventados**. Essa narrativa
> propagou-se para `src/hal/tle8888.cpp`, que ficou escrito contra um mapa de
> registadores inexistente e teve de ser reescrito de raiz. Autoridade duplicada não
> se mantém sincronizada à mão. **Não repor aqui detalhe de pino ou de registador.**

Alvo da placa de interface v1: **STM32H562VGT6 (LQFP100)** + **TLE8888-2QK** como hub de
potência. INJ/IGN por *direct drive* (GPIO → IN1–IN8 do CI), sem MOSFETs discretos.

---

## Alimentação

```
                    ┌─────────────────────────────────────────────────┐
                    │              DISTRIBUIÇÃO DE POTÊNCIA           │
                    │                                                 │
  Bateria 12V ──►──┤ P-MOSFET (proteção de polaridade invertida)     │
                    │    │                                            │
                    │  Fusível 30A                                    │
                    │    │                                            │
                    │  Relé principal (driver integrado no TLE8888)   │
                    │    │                                            │
                    │    ├── rail VBAT ──────────────────────────────│
                    │    │    ├── TLE8888 VBAT (100µF + 100nF)       │
                    │    │    ├── Relé da bomba (fusível 15A)        │
                    │    │    ├── WBO2 (fusível 5A)                  │
                    │    │    ├── Sensor flex fuel 12V               │
                    │    │    ├── Solenóides VVT                     │
                    │    │    ├── SMBJ24CA TVS no rail VBAT          │
                    │    │    └── divisor 0–18V ──► PC3 (VBATT ADC)  │
                    │    │                                            │
                    │    ├── 5V dos sensores: TRACKERS do TLE8888    │
                    │    │    (DVT5Vx, ±10 mV — feitos para sensor   │
                    │    │     ratiométrico; substituem o buck aqui) │
                    │    │    ├── MAP, TPS, APP1/2                   │
                    │    │    ├── P. combustível, P. óleo            │
                    │    │    ├── pull-ups NTC (CLT, IAT)            │
                    │    │    ├── ETB TPS1/TPS2                      │
                    │    │    └── pull-up do flex fuel               │
                    │    │                                            │
                    │    └── LDO 3.3V para o MCU                     │
                    │         ├── STM32 VDD (100nF por pino VDD)     │
                    │         ├── STM32 VDDA (ferrite + 1µF + 100nF) │
                    │         └── VREF+ (pino separado — só LQFP100) │
                    │                                                 │
                    └─────────────────────────────────────────────────┘
```

⚠️ **Peças do desenho legado explicitamente rejeitadas** (critério de imunidade a ruído):

| Rejeitado | Porquê | Usar |
|---|---|---|
| **LM2596-HV** | 150 kHz, ripple ~150 mV — dos switchers mais ruidosos | TPS54302 ou switcher ≥500 kHz de baixo ripple |
| **AMS1117-3.3** | PSRR fraco em alta frequência — não rejeita o ripple que chega | LDO low-noise / high-PSRR |

As duas trocas **andam juntas**: um LDO bom alimentado com 150 kHz de ripple não salva
os sensores ratiométricos. Ver bloco 1 de `interface_board_v1.md`.

⚠️ **VREF+ ainda em aberto.** É pino separado do VDDA no LQFP100 (não existe no LQFP64) e
a escolha — (a) VDDA 3,3 V filtrado, (b) VREFBUF interno 1,8/2,048/2,5 V, (c) derivado do
5 V dos sensores para cancelamento ratiométrico verdadeiro — **redimensiona todos os
divisores** e a constante de `vbatt_raw_to_mv()`. Fechar antes do esquemático.

---

## Condicionamento de sinal

```
Entrada analógica genérica (MAP, TPS, APP1/2, P.combustível, P.óleo):

  Sensor (0.5-4.5V) ──[R1 10k]──┬──[R_filt 1k]──┬──► STM32 ADC (máx 3.3V)
                                 │               │
                               [R2 15k]      [C 100nF]
                                 │               │
                                GND             GND
                                 │
                             [TVS 3.3V]
                                 │
                                GND

NTC (CLT, IAT):

  5V ──[R_pull 2.49kΩ]──┬── NTC para GND
                         │
                    [R1 10k]──┬──[R_filt 1k]──┬──► STM32 ADC
                              │               │
                            [R2 15k]      [C 100nF]
                              │               │
                             GND             GND

VBATT (interno à placa — não gasta pino de conector):

  rail VBAT ──[R1]──┬──[RC generoso]──► PC3 / ADC2_INP13
                     │
                   [R2]        divisor 0–18V → 0–3.3V
                     │         (VBATT é lenta: filtrar forte é grátis e
                    GND         rejeita transientes de bobina)
```

| Sensor | Entrada | Divisor | Faixa ADC | Filtro | Proteção |
|--------|---------|---------|-----------|--------|----------|
| MAP | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| TPS | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| APP1/APP2 | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| ETB TPS1/TPS2 | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| **P. combustível** | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| **P. óleo** | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| CLT (NTC) | 0–5V (pull-up) | 10k/15k | 0–3.0V | RC 1kΩ+100nF | TVS 3.3V |
| IAT (NTC) | 0–5V (pull-up) | 10k/15k | 0–3.0V | RC 1kΩ+100nF | TVS 3.3V |
| **VBATT** | 0–18V | ver acima | 0–3.3V | RC generoso | TVS |
| Flex fuel | 0–12V quadrada | 10k/3.3k | 0–3.0V | — | TVS 3.3V |
| ~~Knock~~ | *diferido v2* | — | — | envelope, **não** portadora | — |

⚠️ O RC de 1k+100nF dá fc ≈ 1,6 kHz — confortável para grandezas lentas, mas **verificar
contra a banda desejada de MAP** (tem conteúdo rápido por pulsação de coletor; filtrar
demais atrasa a resposta transitória de carga).

⚠️ **Knock não usa a rede padrão.** O ADC amostra 1×/dente (abaixo de Nyquist para 6–8 kHz),
por isso o front-end tem de entregar **envelope** — bandpass → retificação → integrador,
polarizado em meio-rail. Diferido para a v2; só footprint (TPIC8101) na v1.

---

## CKP / CMP — sync

```
┌─────────────────────────────────────────────────────────────┐
│  CKP (relutor VR, roda 60-2) — interface VR do TLE8888      │
│                                                             │
│  Sensor VR ──► par trançado e blindado ──► VRIN1 / VRIN2   │
│    • clamp de entrada integrado (50 mA) → SEM rede externa  │
│      de resistor série + clamp                              │
│    • zero-crossing com armamento por deteção de pico        │
│    • modo auto adaptativo; diagnóstico de sensor por SPI    │
│  VROUT (PUSH-PULL) ──► PA0 (TIM5_CH1, input capture)       │
│    • sem pull-up externo (não é open-drain)                 │
│    • pull-down interno de PA0 MANTIDO (anti falso-sync)     │
│  Blindagem aterrada SÓ no lado da ECU                       │
│                                                             │
│  Bancada: TP-DIG + jumper 0Ω entre VROUT e PA0 — removido,  │
│  liberta o nó para o estimulador ESP32 injetar digital.     │
│  TP-VR no par diferencial para fonte analógica VR real.     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  CMP (Hall) — caminho próprio (o CI tem UM canal VR)        │
│                                                             │
│  Hall open-collector ──[pull-up 10k → 5V]──┬── divisor ──► │
│                                             │      PA1      │
│                                          RC leve + clamp    │
│                                                             │
│  ⚠️ ABERTO — decidir antes do esquemático:                  │
│     PA1 tem pull-down interno, que luta contra o pull-up    │
│     externo → trocar para pull-up. MAS Hall open-collector  │
│     idle HIGH/pulso LOW põe o início do dente na borda de   │
│     DESCIDA, e TIM5 captura só SUBIDA (CC2E sem CC2P).      │
│     Trocar só o resistor cronometraria o FIM do pulso.      │
│     Ambas as polaridades passam no gate temporal da ISR →   │
│     o erro sairia como deslocamento angular SILENCIOSO.     │
│     O CKP tem a mesma pergunta (CC1E também é só subida).   │
│     Decidir as duas juntas. Ver interface_board_v1.md.      │
└─────────────────────────────────────────────────────────────┘
```

---

## Atuadores externos

```
┌─────────────────────────────────────────────────────────────┐
│  INJEÇÃO / IGNIÇÃO — TLE8888, direct drive                  │
│                                                             │
│  INJ1–4: GPIO ──► IN1–IN4 ──► OUT1–OUT4                    │
│    low-side 2,2 A → injetores de ALTA IMPEDÂNCIA           │
│    (saturado, sem peak-and-hold)                            │
│    clamp, OC, sobretemperatura e diagnóstico integrados     │
│                                                             │
│  IGN1–4: GPIO ──► IN5–IN8 ──► IGN1–IGN4                    │
│    driver de gate push-pull 20 mA → SMART COILS            │
│    (bobinas com ignitor integrado, entrada lógica)          │
│                                                             │
│  INJEN / IGNEN: enables de HARDWARE dos dois grupos.        │
│    LOW = desabilitado; sobem só se o TLE8888 confirmou      │
│    comunicação E configuração. Corte independente do SPI    │
│    e do escalonador.                                        │
│                                                             │
│  Atribuição IN→OUT é FIXA no silício. Pinos: out_pins.h     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  ETB — ponte-H EXTERNA: BTS7960 @ 10 kHz (decidido)         │
│                                                             │
│  As meias-pontes do TLE8888 são de só 0,6 A — falta uma     │
│  ordem de grandeza para o motor da borboleta.               │
│                                                             │
│  PWM (TIM15_CH1) ──► BTS7960 PWM    [VGT6: PE5]            │
│  DIR abrir  (GPIO) ──► IN1          [VGT6: PE7]            │
│  DIR fechar (GPIO) ──► IN2          [VGT6: PE8]            │
│    (ambos a 0 = travagem)                                   │
│  ETB TPS1/TPS2 ──► ADC (realimentação de posição)          │
│  12V ──► ponte VCC       GND ──► ponte GND                 │
│                                                             │
│  PWM baixado de 20 kHz para 10 kHz: o BTS7960 vai até       │
│  25 kHz, e a 20 kHz sobravam só 20% de margem com perdas    │
│  de comutação altas (FETs internos, sem como aliviar).      │
│  Custo aceite: chiado audível. A interface de 3 pinos       │
│  (PWM+IN1+IN2) mapeia 1:1 no firmware actual.               │
│                                                             │
│  ⚠️ Sem botão de slew (FETs integrados). Se o ETB acoplar   │
│     ruído no par CKP, os remédios são layout, blindagem e   │
│     filtro. Vigiar no teste de ruído sob carga.             │
│                                                             │
│  ⚠️ GATE DE SEGURANÇA — o ETB é a ÚNICA autoridade sobre a  │
│     borboleta (o IACV foi removido) e o autocal+PID nunca   │
│     correram em hardware. Travado aberto = motor em         │
│     disparada. Exigir: mola default-closed verificada,      │
│     corte de energia duro ao alcance, batente mecânico, e   │
│     validação em bancada ANTES de montar no motor.          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  VVT — 2 solenóides, TLE8888 OUT5/OUT6                      │
│                                                             │
│  PWM 15 Hz (TIM4_CH1/CH2) ──► IN11/IN12 ──► OUT5/OUT6      │
│    low-side 4,5 A com CLAMP ATIVO 50–60 V                  │
│    → saem da BOM os drivers de solenóide e os diodos de     │
│      roda-livre                                             │
│                                                             │
│  ⚠️ Os DOIS PIDs consomem o MESMO pos_deg_x10, derivado do  │
│     ÚNICO CMP. O came instrumentado fica em malha fechada;  │
│     o outro persegue a posição do came errado. Controlo     │
│     dual real pede 2º sensor de came + firmware.            │
│     v1: montar os dois, comissionar só o instrumentado,     │
│     reservar via no conector para o 2º sensor.              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  RELÉS — bomba e ventoinha, saídas do TLE8888               │
│                                                             │
│  Saídas de relé (0,6 A — folgado para bobina de ~200 mA)    │
│  + driver de relé principal integrado.                      │
│  → saem da BOM os drivers discretos e os diodos.            │
│                                                             │
│  Bomba: prime 2 s no key-on, mantém com RPM > 0, corta 2 s  │
│  após RPM = 0.  Ventoinha: histerese 95/90 °C.              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  CAN — transceiver integrado no TLE8888                     │
│                                                             │
│  FDCAN1_TX ──► TXD        FDCAN1_RX ◄── RXD                │
│  CANH/CANL ──► barramento, terminação 120Ω jumpeável        │
│  Sem transceiver externo (TJA1051 sai da BOM).              │
│  Serve WBO2 (RX 0x180) e telemetria (0x400/0x401/0x402).    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  WBO2 (Bosch CJ125 / AEM 30-0300) — só por CAN             │
│  12V (fusível 5A) · CANH/CANL · sonda LSU 4.9              │
│  Não há ADC de O2.                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  FLEX FUEL (GM / Continental)                               │
│  12V ──► sensor                                             │
│  Sinal ──[R1 10k]──┬──► PB5 (EXTI)                         │
│                 [R2 3.3k]                                    │
│                    GND                                       │
│  Pull-up 10kΩ → 5V se a saída for coletor aberto            │
│  50 Hz = 0% etanol, 150 Hz = 100%                           │
│  Duty 10–90% = -40 a +125 °C de temperatura do combustível  │
│  Onda quadrada lenta — filtrar com folga.                   │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  USB — COM ISOLADOR GALVÂNICO (conector interno)            │
│                                                             │
│  Ligar o laptop com o motor a rodar cria laço de terra      │
│  entre a massa do veículo e a do portátil — matador         │
│  clássico de ECU e injetor de ruído durante a calibração.   │
│  Isolador JUNTO AO CONECTOR, não junto ao MCU.              │
│  Requer alimentação isolada do lado do veículo.             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  ~~EWG / boost~~ — DIFERIDO para a v2 (só footprint)        │
│  Turbo-específico; o segundo pino DIR ainda é TODO(VGT6).   │
│  É o que libertou PC3 para o VBATT.                         │
│  O canal de realimentação de posição deixou de existir:     │
│  ewg_driver_read_position_raw() devolve 0.                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Conector do chicote do motor

Expresso no que **sai da placa** (saídas do TLE8888 + entradas de sensor + sync), não em
pinos do MCU. USB e SPI são internos e não aparecem aqui.

| Grupo | Vias | Sinais |
|-------|------|--------|
| Potência | 5 | VBAT+, PGND ×2, bobina do relé principal, sense de key-on |
| Injeção | 5 | INJ1–4 low-side (OUT1–OUT4), +12V dos injetores |
| Ignição | 6 | IGN1–4 trigger (gate lógico), +12V das bobinas, PGND das bobinas |
| Sync | 6 | CKP+ (VRIN1), CKP− (VRIN2), blindagem CKP, CMP sinal, CMP +5V, CMP GND |
| Analógicos | 12 | MAP, TPS, CLT, IAT, APP1, APP2, P.combustível, P.óleo, 5V_A, 5V_B, SGND ×2 |
| ETB | 6 | Motor+, Motor−, TPS1, TPS2, 5V, SGND |
| VVT | 3 | VVT escape (LS), VVT admissão (LS), +12V dos solenóides |
| Relés | 2 | Bomba (bobina, LS), Ventoinha (bobina, LS) |
| CAN | 3 | CANH, CANL, blindagem |
| Flex fuel | 3 | +12V, sinal, GND |
| *Knock (diferido)* | 2 | *sinal piezo, blindagem — via reservada, NÃO cablar na v1* |
| Reserva | 2 | 2º sensor de came (VVT dual), expansão |
| **Total** | **55** | |

⚠️ **55 vias, não ~47.** A estimativa antiga de 47 vinha de uma tabela que contava
`EWG pos` (diferido, e o pino de potenciómetro externo vai com ele), omitia pressão de
combustível e de óleo (ambas **populadas** na v1) e listava VVT por pinos do MCU
(`PB6`/`PB7`) em vez das saídas do CI que realmente vão ao chicote. **Escolher o modelo de
conector contra 55, com margem.**

**Regras:** separar fisicamente as vias de potência (INJ/IGN/relés/VVT) das de sinal; CKP e
CAN em vias adjacentes com dreno de blindagem; bitola por circuito (injetor e bobina puxam
corrente); pares trançados para CKP/CMP/CAN; fusíveis por ramo.

⚠️ **VBATT não gasta via** — é medida do rail interno da placa (divisor → PC3).

---

## Terra

```
                   Parafuso do chassis (ponto estrela único)
                            │
                 ┌──────────┼──────────┐
                 │          │          │
               PGND       SGND     Shield GND
             (potência)  (sinal)  (blindagens)
                 │          │          │
           ┌─────┤    ┌─────┤    ┌─────┤
           │ TLE8888   │ sensores │ CAN
           │ reguladores│ AGND ADC│ cabo CKP
           │ ponte ETB │ pull-ups │
           │ bomba     │          │
           └───────────┘──────────┘

  Regras de PCB (4 camadas: sinal / TERRA CONTÍNUO / alimentação / sinal):
  • Pours separados PGND / SGND / AGND / Shield, unidos num único ponto.
    Correntes de injetor e bobina NUNCA atravessam o retorno de sinal.
  • Par CKP é a rede mais sensível: curto, blindado, na camada superior sobre
    terra ininterrupto, longe de INJ/IGN/ETB/relés, sem via desnecessária e
    sem passar sob o indutor do buck.
  • Loops de comutação (injetor, bobina, solenóides VVT) fisicamente pequenos
    — área de laço é o que irradia.
  • Cabo de knock é a 2ª rede mais sensível: longe das linhas de bobina e dos
    solenóides VVT — escuta justamente a banda que a ignição emite.
  • 100nF em cada pino VDD; bulk 100µF no estágio de potência; VDDA por ferrite.
  • Blindagens de CKP e CAN aterradas só no lado da ECU.
```
