# Hardware — placa de interface OpenEMS v1

Ponto de entrada da documentação de hardware. **Ler esta página antes de qualquer outra.**

**Objetivo do projeto:** sair da bancada para a **primeira partida em motor real** (4 cilindros, roda
60-2, injeção e ignição sequenciais, borboleta eletrónica).

**Desenho KiCad de produção:** projecto **em branco** → `hardware/openems_v1/`  
(MCU **H562 soldado** LQFP100, **4 camadas**, arquitectura v2 — CIs dedicados,
TLE8888 **saiu**, ver [`architecture_v2.md`](architecture_v2.md)).  
WeAct H562 = **só bancada de firmware**, não entra na PCB de produção.

⛔ **O fork do microRusEFI deixou de ser a base de desenho (2026-08-07).** O mRE continua
no repo como **referência** (`hardware/vendor/hw_microRusEfi`) — é a implementação que a
regra §1b obriga a consultar antes de cada bloco. O board antigo `hardware/openems_ecu/`
**não foi apagado**, mas não recebe mais trabalho. Porquê da mudança:
[`microruseefi_as_base.md`](microruseefi_as_base.md). Hellen-One **não** é a base da ECU.

---

## 1. Mapa de autoridade — qual fonte manda sobre o quê

⚠️ **Esta tabela é o antídoto para a causa-raiz dos dois piores incidentes do projeto:** o driver do
TLE8888 foi escrito contra um mapa de registadores **inventado**, herdado da narrativa do
`wiring_diagram.md`; e uma tabela stale mandava remover o pull-down do `PA0`, o que teria desfeito o fix
de falso-sync. **Ambos vieram de autoridade duplicada que se dessincronizou.**

| Assunto | Fonte de verdade | ⚠️ NÃO usar |
|---|---|---|
| Mapa de pinos RGT6 vs VGT6 | `pinout.md` | `../wiring_diagram.md` |
| INJ/IGN, enables, BSRR | `src/hal/out_pins.h` | qualquer doc |
| Canais e pinos de ADC | `src/hal/adc.h` | qualquer doc |
| Registadores do TLE8888 | `src/hal/tle8888_regs.h` | qualquer doc |
| Arquitetura, blocos, BOM, **decisões e porquês** | `interface_board_v1.md` | — |
| Alimentação, condicionamento, atuadores, conector, terra | [`../wiring_diagram.md`](../wiring_diagram.md) | — |
| **Números de pino e AF do MCU (v2)** | [`pinout_v2.md`](pinout_v2.md) | inventar/copiar de spec externa |
| **Arquitectura v2 (MC33810/L9960T/TPS65381A/CJ125)** | [`architecture_v2.md`](architecture_v2.md) | — |
| ~~Ligações pino-a-pino (netlist)~~ | `netlist_v1.md` | ⚠️ **pré-v2** — blocos do TLE8888 anulados |
| Contraprova do mapa TLE8888 | `tle8888_crosscheck.md` | — |
| **Pinos de package LQFP-100 do TLE8888** | `tle8888_pinout.md` | inventar números |
| Registadores/periféricos do MCU | `stm32h562_ref.md` | — |
| **Forma / furos / headers do coreboard WeAct VGT6** | `weact_h562_coreboard.md` | assumir board 64 pinos |
| **Esquemático WeAct V1.0 (P1/P2 nets)** | `weact_h562_schematic.md` + `weact_h562_v10_schdoc.pdf` | — |
| **AMPSEAL 35/23 plugues + headers PCB** | `ampseal_connectors.md` + PDFs TE | inventar footprint |
| **KiCad libs Speeduino/rusEFI (review)** | `kicad_vendor_review.md` | inventar footprint se já existe upstream |
| **BOM candidatos** | `bom_v1_candidates.md` | — |
| **Esquemático modular (sheets)** | `schematic/README.md` | um PDF monólito sem revisão |
| **Board de produção (KiCad em branco)** | `hardware/openems_v1/` | `hardware/openems_ecu/` (fork mRE, congelado) |
| ~~Base de desenho a partir do mRE~~ | `microruseefi_as_base.md` | **supersedido** — mRE é referência, não cobre |
| ~~H562 no cobre microRusEFI (bring-up)~~ | ~~`pinout_mre_bringup.md`~~ | **removido** com o `BOARD=mre` |
| ~~Condicionamento VR discreto~~ | `vr_input_conditioning.md` | **supersedido** (MAX9926/9924 saiu) |

### A regra
**O "porquê" vive num sítio só. Nenhum documento repete o racional de outro.**
`netlist_v1.md` diz *o que liga a quê* e aponta para os blocos; não explica escolhas.
Código sempre ganha a documento. Se divergirem, o documento está errado.

---

## 1b. ⭐ Antes de desenhar qualquer bloco — consultar Speeduino e rusEFI

**Regra geral (obrigatória):** antes de elaborar um esquemático **do zero** para cada
módulo (alimentação, VR/CKP, injectores, ignição, ETB, CAN, USB, knock, VVT, relés,
conector, terra, …), **consultar primeiro** as implementações e o hardware de
referência do **Speeduino** e do **rusEFI**. Só depois adaptar ao OpenEMS
(STM32H562 VGT6 + TLE8888-2QK + contratos do firmware).

Isto não é “copiar a placa”; é **não reinventar armadilhas** que a comunidade já
pagou: pinout de CI, ordem de init SPI, polaridade, pull, clamp, direct-drive,
layout de potência vs sync, fusíveis, terra.

| O quê procurar | Speeduino | rusEFI |
|---|---|---|
| Esquemáticos / boards | `reference/`, hardware boards no repo/wiki | `hardware/`, schematics, board packages |
| Drivers de estágio / CI | `speeduino/` (ex. MC33810, injectors) | `firmware/hw_layer/drivers/` (ex. `gpio/tle8888.cpp`) |
| Condicionamento VR/Hall | wiki + boards | wiki + `hw_layer` + forum |
| Knock / WBO2 / CAN | community boards | wiki + drivers |

**Cópias locais neste ambiente (usar se existirem; senão upstream):**
- Speeduino: `~/Downloads/speeduino-202501.6/speeduino` e `~/Arduino/speeduino`
- rusEFI (firmware/hw): `~/RusefiH5/.work/rusefi` (e projectos em `~/RusefiH5/`)
- Upstream: [speeduino/speeduino](https://github.com/speeduino/speeduino),
  [rusefi/rusefi](https://github.com/rusefi/rusefi)

**Checklist por módulo (antes do KiCad / netlist final):**
1. Achar **pelo menos uma** implementação de referência (board ou driver) no Speeduino **ou** rusEFI.
2. Anotar o que **adoptamos**, o que **adaptamos** (H562 / TLE8888 / pinout nosso) e o que **rejeitamos** (e porquê).
3. Preferir contraprova independente (como `tle8888_crosscheck.md`) a inventar mapa/pinout a partir de memória ou de doc stale.
4. Se o módulo for só footprint (ex. knock v2), registar a referência na netlist mesmo assim.

⚠️ **Não substitui datasheet.** Speeduino/rusEFI validam *uso no mundo real*; o datasheet
valida *limites eléctricos*. Os dois em conjunto.

Memória de projecto: `always-check-speeduino-rusefi-ms` (já invocada no plano de knock).

---

## 2. Registo de decisões

Todas fechadas em **2026-07-20**, na branch `feat/interface-board-v1`, salvo indicação em
contrário. Fundamentação em `interface_board_v1.md` salvo indicação em contrário. A coluna
**Estado v2** regista o que a arquitectura de CIs dedicados (2026-08-07,
[`architecture_v2.md`](architecture_v2.md)) fez a cada uma — ⛔ **anulada**, 🔄 **mudou**
(o conceito sobrevive, o detalhe não — ver `pinout_v2.md`), ✅ **sobrevive** sem alteração.
Não repetir aqui o racional da troca — vive só em `architecture_v2.md`.

| Decisão | Porquê, em uma frase | Commit | Estado v2 |
|---|---|---|---|
| **Alvo VGT6 (LQFP100)** | GPIOE inteiro para INJ/IGN, sem os conflitos do RGT6 | — | ✅ sobrevive |
| ~~TLE8888-2QK como hub de potência~~ | Substitui FETs, drivers de bobina, relés, transceiver CAN e reguladores de 5 V; a **-2QK** tem watchdog desativado de fábrica, e errar o watchdog mata o motor no bring-up | `a5f95fb` | ⛔ anulada — CIs dedicados |
| **INJ/IGN por direct drive** | `IN1–IN8` são ativo-alto com pull-down interno → **scheduler congelado fica intacto** | `a5f95fb` | 🔄 mudou — mesmo princípio, agora via MC33810 (entradas paralelas directas, mesmos pinos GPIOE) |
| ~~Driver TLE8888 reescrito~~ | Mapa inventado + frame invertido; agora unlock + InConfig + OE_SET (rusEFI-aligned). **Ainda sem clock em silício** | `a5f95fb` + hub fix | ⛔ anulada — `tle8888.cpp`/`tle8888_regs.h` reformam-se (dívida de firmware, não escrito) |
| ~~INJEN=`PE14` / IGNEN=`PE3`~~ | Corte de injeção e ignição em hardware, independente do SPI *e* do escalonador. `PE14` porque o LQFP100 do H562VGTx não bonda `PE1` | `380a0c5` + fix PE1→PE14 | ⛔ anulada — MC33810 usa `EN`(`PD11`)/`RSTB`(`PD10`)/`FAULTB`(`PD9`), um enable só, não INJ/IGN separados. `PE14` e `PE3` foram reciclados (`MAIN_RELAY` e `TPS65381_ENDRV`) |
| **Bomba/ventoinha** | Em `PB12`/`PB13` matavam o `SPI2_SCK` no boot | `380a0c5` | 🔄 mudou — razão mantém-se (SPI2 continua em `PB12-15` para PMIC/MC33810/L9960T/CJ125), mas o pino mudou: `PE10`/`PE12` agora são VVT1/VVT2; bomba/ventoinha foram para `PE7`/`PE8` |
| **SDMMC guardado no RGT6** | `PC8` é IGN3 no RGT6; ligar o datalog reconfiguraria o pino de uma bobina | `380a0c5` | ✅ sobrevive — não depende do hub |
| ~~CKP pela interface VR do TLE8888~~ | Zero-crossing com armamento por pico, clamp e diagnóstico integrados → **MAX9924 sai da BOM** | — | ⛔ anulada — sem TLE8888 não há interface VR; **CKP passa a Hall** |
| **CMP: Hall** direto ao `PA1` | ~~O CI tem **um** canal VR, gasto no CKP~~ | — | 🔄 mudou — pino e sensor sobrevivem, mas a razão morreu com o TLE (não há mais canal VR a partilhar). Motivo actual: CKP **também** é Hall, decisão irmã em `architecture_v2.md` |
| **VBATT em `PC3`/INP13** | Antes era fixado em 12000 mV, o que subestimava dead-time e encurtava o dwell no cranking | `34b40e3` | ✅ sobrevive |
| ~~EWG diferido para v2~~ | Turbo-específico; é o que liberta `PC3` para o VBATT | `34b40e3`, `8c3d282` | ⛔ anulada — **populado na v2**, no L9960T junto com o ETB. (O "v2" aqui era genérico, revisão futura — coincide, mas não é a arquitectura v2 de CIs dedicados) |
| **Knock diferido** | Bloco analógico mais difícil, não contribui para a primeira partida, e o retard **mascara problema mecânico** | — | ✅ sobrevive — continua diferido, sem relação com a troca de CIs. (Mesmo aviso: "v2" no nome original é genérico, não a arquitectura de CIs dedicados) |
| ~~ETB: BTS7960 @ 10 kHz~~ | A 20 kHz sobravam 20% de margem; o DRV8701 não é drop-in (firmware é 3 pinos, ele é 2) e transferia o layout de potência. Path real: `etb_driver_init` → `etb_pwm_init(10000)` | `11c39f4` + fix path | ⛔ anulada — **L9960T substitui o BTS7960**, ETB e EWG na mesma ponte dupla. A rejeição do DRV8701 por ser de 2 pinos foi explicitamente revertida: o L9960T é igualmente PWM+DIR e foi aceite. `etb_driver.cpp` **reescreve-se** de 3 pinos para 2+SPI (dívida de firmware) |
| **VREF+ = VDDA 3,3 V filtrado**, (c) DNP | Os trims absorvem **deriva** mas não **ruído** → o esforço rende no LDO e no layout, não numa referência exata | `8df4dbc` | ✅ sobrevive |
| **Conector: AMPSEAL `776164-1` (35, sinais) + `770680-1` (23, potência)** | Tamanhos diferentes são **impossíveis de trocar**; potência de bobinas/injetores **não atravessa a ECU** | `fd9faa0` | ✅ sobrevive |
| **Montagem na cabine**, não no compartimento do motor | O coreboard é grau consumidor — nenhuma caixa resolve ciclo térmico; e põe a antepara aterrada entre a ECU e a ignição | `fd9faa0` | ✅ sobrevive |
| **VVT: montar dois, comissionar um** | Os dois PIDs partilham o `pos_deg_x10` do único CMP | `43a2ff0` | ✅ sobrevive — `PE10`/`PE12` no pinout v2 |
| ~~Fingerprint de reset values~~ | `write_verify` é cego ao endereço errado-mas-válido | `43a2ff0`, `529b1ec` | ⛔ anulada — era técnica específica do TLE8888. Os CIs novos têm os seus próprios sinais de diagnóstico (`MC33810_FAULTB`, `TPS65381A` via SPI) mas **não têm equivalente ao fingerprint desenhado ainda** — dívida de firmware, não só de hardware |
| **KiCad, 4 camadas** | Mínimo honesto para os pours PGND/SGND/AGND separados | — | ✅ sobrevive |
| **USB com isolador galvânico** | Laço de terra com o portátil é matador clássico de ECU | — | ✅ sobrevive |
| **Corte da bomba 3 s → 2 s** | Corta mais cedo num acidente sem cortar num calo momentâneo | `380a0c5` | ✅ sobrevive — lógica de firmware, independente do pino físico |

---

## 3. Em aberto

| Item | Estado |
|---|---|
| **Part numbers Tier C finais** | Candidatos em `bom_v1_candidates.md` — fechar stock/preço. |
| **Bobinas** | IGN = gate IGBT 20 mA — escolher bobina/IGBT **antes** de chicote. |
| **Caixa / coating / orçamento** | Aresta conector ≥ **125 mm** (dois AMPSEAL RA); alvo de custo. |

**Já no código (bring-up ainda sem silício):**
- Driver hub: unlock / InConfig / OE_SET / DD pump-fan
- Eco de endereço SPI (gate) + fingerprint consultivo (só OpConfig0/OutConfig3)
- page0 56–76 restaurado no boot (`apply_page0_trims_driveability`)
- **Polaridade CKP/CMP** page0[258] + `tim5_ic_set_capture_polarity` (Speeduino: TrigEdge; pull segue a borda)

**Relé principal — ✅ decisão v1 (após rusEFI):** ECU alimentada por **key-on** (sem power-latch MCU).
Via `MAIN_RLY` no AMPSEAL fica **reservada / DNP** para o driver de main-relay do TLE
(`Cmd0` MRON/MRSE no rusEFI). Não comandar na v1 — evita matar flash a meio de burn.

---

## 4. ⚠️ Afirmado nos documentos mas **NÃO verificado**

A secção mais importante desta página. Tudo aqui está escrito algures como se fosse facto, e **não** foi
confirmado contra fonte primária. Não construir em cima sem confirmar.

| Afirmação | Estado real |
|---|---|
| **Mapa de registadores do TLE8888** | ✅ **Endereços confirmados** por implementação independente (rusEFI) — ver `tle8888_crosscheck.md`. ⚠️ Mas **nenhum valor de reset** foi confirmado, e 5 dos 7 do fingerprint usam endereços que o rusEFI não toca. |
| **Atraso de propagação do VR** | 🚨 **O datasheet NÃO especifica.** Os 50 ns que os docs citavam eram do **MAX9924**, peça que saiu da BOM. Medição obrigatória no passo 3. |
| **VREF+ é pino separado no LQFP100** | ✅ **Sim** no H562 LQFP100 (`stm32h562vg.pdf` pinout — VREF+ ≠ double-bond VDDA). Opção (a): VREF+→VDDA filtrado. |
| **Números de pino do TLE8888** | ✅ **Verificados** DS Rev 1.2 §3 + rusEFI lib — `tle8888_pinout.md`. |
| **Dimensões do coreboard** | ✅ **WeAct V1.0 Board Shape:** 38,62 × 69,10 mm, furos Φ3,2 @ 2,80 mm — `weact_h562_coreboard.md`. Altura USB ainda a medir. |
| **Todo o driver TLE8888** | 🚨 **Nunca clockou silício.** Os 1234 host-tests mockam o SPI. Só o bring-up decide. |

---

## 5. Antes do layout — medições

### Coreboard WeAct (VGT6) — ✅ shape fechado
Ver **`weact_h562_coreboard.md`** (PDF *Board Shape 外形* V1.0):
- Contorno **38,62 × 69,10 mm**
- Furos **Φ 3,2**, offset **2,80 mm**, pitch horizontal **30,48 mm**
- Headers laterais dual-row **2,54 mm**, GPIOE exposto

**Ainda a medir na peça física:** altura total (USB-C + headers + componentes);
confirmar pin 1 de cada conector face ao silkscreen.

### AMPSEAL — plugues + headers PCB ✅
Ver **`ampseal_connectors.md`**:
- Plugs: `776164-1` (35) / `770680-1` (23)
- Headers RA (recomendado cabine): **`1-776180-1`** (35 gold) + **`1-770669-1`** (23 gold)
- Drawings no repo: `TE_776180_*.pdf`, `TE_770669_*.pdf`, `TE_776230_*.pdf` (vertical alt.)
- Aresta mínima ≈ **125 mm**; altura header RA ≈ **18 mm** + plug

---

## 6. Sequência de verificação

Ordem inegociável — **bancada → ETB validado → motor**. Detalhe em `interface_board_v1.md`.
⚠️ **Escrita pré-v2** (2026-07-20); os passos 0 e 3 assumiam o TLE8888/VR e ficaram void
com a arquitectura de CIs dedicados (`architecture_v2.md`, 2026-08-07). A ordem geral
continua válida — bancada antes de ETB, ETB antes de motor.

0. ~~Fingerprint + eco do TLE8888~~ 🚨 **sem substituto desenhado.** O TLE8888 saiu da
   produção; o passo existia porque o `write_verify` do TLE era cego ao endereço
   errado-mas-válido (ver decisão "Fingerprint de reset values" anulada em §2). Os CIs
   novos (MC33810, L9960T, CJ125, TPS65381A) têm os seus próprios sinais de diagnóstico
   (`MC33810_FAULTB`, SPI do TPS65381A) mas **não têm um passo 0 equivalente definido** —
   dívida de firmware/bring-up, não só de hardware. Escrever antes do primeiro power-on
   dos CIs novos.
1. Host tests verdes (`make host-test`, `make host-test-vgt6`).
2. Scope de INJ/IGN e sync CKP/CMP de 200 a 8500 rpm com o estimulador.
3. **Caracterizar o front-end CKP** — 🔄 **reformulado.** CKP deixou de ser VR (era pela
   interface do TLE8888) e passou a **Hall**, mesmo condicionamento do CMP. Já não se mede
   "atraso de propagação do VR" — mede-se o atraso do front-end Hall escolhido, e fecha-se
   a par da polaridade RISING/FALLING pendente (ver `cmp-ckp-capture-edge-polarity` em
   memória) assim que o datasheet do sensor existir.
4. **Ruído sob carga** — o teste que teria apanhado o falso-sync original.
5. Analógicos, VBATT contra multímetro; depois flex e VVT.
6. **Gate do ETB** — autocal e PID com chicote real, **desacoplado do motor**. 🔄
   O driver muda de BTS7960 (3 pinos) para **L9960T** (PWM+DIR+SPI, `etb_driver.cpp` por
   reescrever) — o gate em si não muda de posição na sequência, só o que há para testar.
7. Motor, escalonado: cranking sem faísca → faísca sem combustível → partida.
