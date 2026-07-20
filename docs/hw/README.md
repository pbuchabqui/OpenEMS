# Hardware — placa de interface OpenEMS v1

Ponto de entrada da documentação de hardware. **Ler esta página antes de qualquer outra.**

**Objetivo do projeto:** sair da bancada para a **primeira partida em motor real** (4 cilindros, roda
60-2, injeção e ignição sequenciais, borboleta eletrónica), com uma placa carrier/interface que recebe
o coreboard WeAct STM32H562**VGT6**.

---

## 1. Mapa de autoridade — qual fonte manda sobre o quê

⚠️ **Esta tabela é o antídoto para a causa-raiz dos dois piores incidentes do projeto:** o driver do
TLE8888 foi escrito contra um mapa de registadores **inventado**, herdado da narrativa do
`wiring_diagram.md`; e uma tabela stale mandava remover o pull-down do `PA0`, o que teria desfeito o fix
de falso-sync. **Ambos vieram de autoridade duplicada que se dessincronizou.**

| Assunto | Fonte de verdade | ⚠️ NÃO usar |
|---|---|---|
| Mapa de pinos RGT6 vs VGT6 | `pinout.md` | `wiring_diagram.md` |
| INJ/IGN, enables, BSRR | `src/hal/out_pins.h` | qualquer doc |
| Canais e pinos de ADC | `src/hal/adc.h` | qualquer doc |
| Registadores do TLE8888 | `src/hal/tle8888_regs.h` | qualquer doc |
| Arquitetura, blocos, BOM, **decisões e porquês** | `interface_board_v1.md` | — |
| Alimentação, condicionamento, atuadores, conector, terra | `wiring_diagram.md` | — |
| Ligações pino-a-pino (netlist) | `netlist_v1.md` | — |
| Contraprova do mapa TLE8888 | `tle8888_crosscheck.md` | — |
| Registadores/periféricos do MCU | `stm32h562_ref.md` | — |
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

Todas fechadas em **2026-07-20**, na branch `feat/interface-board-v1`. Fundamentação em
`interface_board_v1.md` salvo indicação em contrário.

| Decisão | Porquê, em uma frase | Commit |
|---|---|---|
| **Alvo VGT6 (LQFP100)** | GPIOE inteiro para INJ/IGN, sem os conflitos do RGT6 | — |
| **TLE8888-2QK como hub de potência** | Substitui FETs, drivers de bobina, relés, transceiver CAN e reguladores de 5 V; a **-2QK** tem watchdog desativado de fábrica, e errar o watchdog mata o motor no bring-up | `a5f95fb` |
| **INJ/IGN por direct drive** | `IN1–IN8` são ativo-alto com pull-down interno → **scheduler congelado fica intacto** | `a5f95fb` |
| **Driver TLE8888 reescrito** | Mapa inventado + frame invertido; agora unlock + InConfig + OE_SET (rusEFI-aligned). **Ainda sem clock em silício** | `a5f95fb` + hub fix |
| **INJEN=`PE1` / IGNEN=`PE3`** | Corte de injeção e ignição em hardware, independente do SPI *e* do escalonador | `380a0c5` |
| **Bomba/ventoinha → `PE10`/`PE12`** | Em `PB12`/`PB13` matavam o `SPI2_SCK` no boot — o TLE8888 nunca seria clockado | `380a0c5` |
| **SDMMC guardado no RGT6** | `PC8` é IGN3 no RGT6; ligar o datalog reconfiguraria o pino de uma bobina | `380a0c5` |
| **CKP pela interface VR do TLE8888** | Zero-crossing com armamento por pico, clamp e diagnóstico integrados → **MAX9924 sai da BOM** | — |
| **CMP: Hall** direto ao `PA1` | O CI tem **um** canal VR, gasto no CKP | — |
| **VBATT em `PC3`/INP13** | Antes era fixado em 12000 mV, o que subestimava dead-time e encurtava o dwell no cranking | `34b40e3` |
| **EWG diferido para v2** | Turbo-específico; é o que liberta `PC3` para o VBATT | `34b40e3`, `8c3d282` |
| **Knock diferido para v2** | Bloco analógico mais difícil, não contribui para a primeira partida, e o retard **mascara problema mecânico** | — |
| **ETB: BTS7960 @ 10 kHz** | A 20 kHz sobravam 20% de margem; o DRV8701 não é drop-in (firmware é 3 pinos, ele é 2) e transferia o layout de potência. Path real: `etb_driver_init` → `etb_pwm_init(10000)` | `11c39f4` + fix path |
| **VREF+ = VDDA 3,3 V filtrado**, (c) DNP | Os trims absorvem **deriva** mas não **ruído** → o esforço rende no LDO e no layout, não numa referência exata | `8df4dbc` |
| **Conector: AMPSEAL `776164-1` (35, sinais) + `770680-1` (23, potência)** | Tamanhos diferentes são **impossíveis de trocar**; potência de bobinas/injetores **não atravessa a ECU** | `fd9faa0` |
| **Montagem na cabine**, não no compartimento do motor | O coreboard é grau consumidor — nenhuma caixa resolve ciclo térmico; e põe a antepara aterrada entre a ECU e a ignição | `fd9faa0` |
| **VVT: montar dois, comissionar um** | Os dois PIDs partilham o `pos_deg_x10` do único CMP | `43a2ff0` |
| **Fingerprint de reset values** | `write_verify` é cego ao endereço errado-mas-válido | `43a2ff0`, `529b1ec` |
| **KiCad, 4 camadas** | Mínimo honesto para os pours PGND/SGND/AGND separados | — |
| **USB com isolador galvânico** | Laço de terra com o portátil é matador clássico de ECU | — |
| **Corte da bomba 3 s → 2 s** | Corta mais cedo num acidente sem cortar num calo momentâneo | `380a0c5` |

---

## 3. Em aberto

| Item | Estado |
|---|---|
| **Polaridade de borda CMP/CKP** | Desenho: byte de calibração em page0 (offset ≥258). Ver `netlist_v1.md`. Firmware **por implementar**; **não** bloqueia o cobre Hall. |
| **Números físicos de pino do TLE8888** | Tirar da tabela de pinout do datasheet. **Bloqueia o layout.** |
| **Dimensões do coreboard e das caixas AMPSEAL** | Medição manual + desenhos da TE. Ver §5. |
| **Part numbers Tier C** | Buck, LDO, isolador USB, DC-DC isolado, bobinas (validar IGN vs smart coil). |
| **Relé principal** | Três narrativas (key-on / driver CI / via J2) — fechar uma. |
| **Caixa, vedação, coating, orçamento** | Precisa de alvo de custo. |

**Já no código (bring-up ainda sem silício):**
- Driver hub: unlock / InConfig / OE_SET / DD pump-fan
- Eco de endereço SPI (gate) + fingerprint consultivo (só OpConfig0/OutConfig3)
- page0 56–76 restaurado no boot (`apply_page0_trims_driveability`)

---

## 4. ⚠️ Afirmado nos documentos mas **NÃO verificado**

A secção mais importante desta página. Tudo aqui está escrito algures como se fosse facto, e **não** foi
confirmado contra fonte primária. Não construir em cima sem confirmar.

| Afirmação | Estado real |
|---|---|
| **Mapa de registadores do TLE8888** | ✅ **Endereços confirmados** por implementação independente (rusEFI) — ver `tle8888_crosscheck.md`. ⚠️ Mas **nenhum valor de reset** foi confirmado, e 5 dos 7 do fingerprint usam endereços que o rusEFI não toca. |
| **Atraso de propagação do VR** | 🚨 **O datasheet NÃO especifica.** Os 50 ns que os docs citavam eram do **MAX9924**, peça que saiu da BOM. Medição obrigatória no passo 3. |
| **VREF+ é pino separado no LQFP100** | ⚠️ Citação **não reconfirmada** — PDFs da ST deram timeout. Não afeta a decisão (a). |
| **Números de pino do TLE8888** (ex. "pino 24/27") | ⚠️ Não verificados. |
| **Dimensões do coreboard** | ⚠️ Não obtidas — a repo pública da WeAct é a de **64 pinos**, não a VGT6. |
| **Todo o driver TLE8888** | 🚨 **Nunca clockou silício.** Os 1234 host-tests mockam o SPI. Só o bring-up decide. |

---

## 5. Antes do layout — medições em falta

**No coreboard físico:** comprimento × largura (±0,5 mm) · passo e posição dos headers (distância entre
filas e do bordo a cada fila) · nº de pinos por fila e origem da numeração · altura com USB montado e
altura sob a placa com barra de pinos · furos de fixação (diâmetro e coordenadas a partir de um canto).

**Desenhos da TE** para `776164-1` e `770680-1`: *customer drawing* do **header PCB** (não do plug), e a
variante (vertical vs cotovelo). Disponíveis nas páginas de produto da TE e em DigiKey/Mouser/Farnell.

---

## 6. Sequência de verificação

Ordem inegociável — **bancada → ETB validado → motor**. Detalhe em `interface_board_v1.md`.

0. **Fingerprint + eco do TLE8888** no primeiro power-on — único momento possível, os valores de reset
   desaparecem na primeira escrita.
1. Host tests verdes (`make host-test`, `make host-test-vgt6`).
2. Scope de INJ/IGN e sync CKP/CMP de 200 a 8500 rpm com o estimulador.
3. **Caracterizar o front-end CKP** — medir o atraso de propagação do VR (o datasheet não o dá).
4. **Ruído sob carga** — o teste que teria apanhado o falso-sync original.
5. Analógicos, VBATT contra multímetro; depois flex e VVT.
6. **Gate do ETB** — autocal e PID com chicote real, **desacoplado do motor**.
7. Motor, escalonado: cranking sem faísca → faísca sem combustível → partida.
