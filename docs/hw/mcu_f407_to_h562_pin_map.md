# STM32F407VGTx → STM32H562VGTx — mapa de migração (LQFP100)

> ⛔ **SUPERSEDIDO (2026-08-07) para efeitos de desenho.** Este documento existia para
> pousar o H562 no cobre F407 do microRusEFI. Esse fork foi abandonado —
> ver [`microruseefi_as_base.md`](microruseefi_as_base.md). A board nova parte de um
> esquemático em branco, onde não há F407 nenhum para mapear.
>
> **Continua útil como referência de peça:** a comparação pino-a-pino 1–100 e as 6
> diferenças de alimentação do H5 estão verificadas contra os symbols oficiais e
> aplicam-se a qualquer desenho com H562VGTx. Para *ligações*, a autoridade é
> [`netlist_v1.md`](netlist_v1.md) — não este ficheiro.

> Fonte: pinos extraídos diretamente dos symbols oficiais KiCad
> (`MCU_ST_STM32F4:STM32F407V_E-G_Tx`, `MCU_ST_STM32H5:STM32H562VGTx`,
> `/usr/share/kicad/symbols/`), comparados pino-a-pino (1–100). O footprint
> **LQFP-100_14x14mm_P0.5mm é idêntico** nos dois — a cópia física do
> `hardware/openems_ecu/*.kicad_pcb` (base microRusEFI, populado com
> STM32F427VGT6) não precisa mudar de footprint, só de ligações em 6 pads.

## Resultado: 94/100 pinos idênticos

Isto é excecionalmente compatível — a maioria dos MCUs "pin-compatíveis" da ST
só garantem isso dentro da mesma família (F4↔F4). Aqui F4→H5 mantém quase tudo.

## Os 6 pinos que mudam

| Pad | F407VGTx | H562VGTx | Ação |
|-----|----------|----------|------|
| 19 | `VDD` | `VSSA` | Ligar a **AGND** (antes ia a VDD/3V3) |
| 20 | `VSSA` | `VREF-` | Ligar a **AGND** (mesma função em espírito — já era GND) |
| 48 | `PB11` | `VCAP` | **GPIO perdido.** Cap cerâmico a GND (regulador interno) |
| 49 | `VCAP_1` | `VSS` | Ligar a **GND** (deixa de ser pino de cap) |
| 73 | `VCAP_2` | `VDDUSB` | **Novo pino de alimentação.** Decoupling + alimentar a partir do `VDD` local (3V3) |
| 98 | `PE1` | `VCAP` | **GPIO perdido** — já sabíamos disto (ver `stm32h562_ref.md` §3.1). Cap cerâmico a GND |

Os restantes "DIFF" que o script assinalou (pinos 72, 76, 77, 89, 90) são só
**relabeling cosmético** SWD/JTAG (`PA13`→`PA13(JTMS/SWDIO)`, `PA14`→`PA14(JTCK/SWCLK)`,
`PA15`→`PA15(JTDI)`, `PB3`→`PB3(JTDO)`, `PB4`→`PB4(NJTRST)`) — mesmo pino, mesma
função, símbolo H5 só documenta a alternate function no nome.

## PB11 (pad 48) — impacto real: nenhum

`docs/hw/pinout.md` já avisa: *"No coreboard WeAct não saem nos headers — não
usar para INJ"*. O firmware (`out_pins.h`, VGT6) nunca usa `PB11` — INJ3/4 são
`PE4`/`PE6`. No board vendor (microRusEFI) o pino aparece como hierarchical
label solto no sheet `stm32.kicad_sch` sem nenhuma ligação a outro sub-sheet
(`grep` confirma zero ocorrências de `"PB11"` fora do próprio sheet MCU) —
já era spare. Perdê-lo ao STM32H5 regulador interno **não tira nenhuma função
usada**.

## PE1 (pad 98) — resolvido no firmware e no KiCad

[[injen-pe1-not-bonded-moved-pe14]] (memória) — `INJEN` migrou de `PE1` para
`PE14` em `src/hal/out_pins.h/.cpp` e nos docs. **`PE14` está livre no vendor
board**: `openems_ecu.kicad_sch` expõe um pino de sheet `"PE14"` vindo do
`stm32.kicad_sch`, mas **sem nenhum label a juntar-se a ele no root** — é um
dead-end não ligado a nada. Confirmado por grep: só 1 ocorrência de `"PE14"` em
todo o `openems_ecu.kicad_sch`. Portanto mover `INJEN` de `PE1`→`PE14` no
esquemático não colide com nada — só passa a **usar** um pino que já estava lá,
livre.

Ação no KiCad (`hardware/openems_ecu/`) — **feita**:
1. ✅ `mcu_h562.kicad_sch` (symbol H562, 6 pads renomeados, HL `PE1`/`PB11` removidos).
2. ✅ Root: `Sheetfile` → `mcu_h562.kicad_sch`; sheet-pins `PE1`/`PB11` removidos; `PE14` mantido.
3. ✅ Labels root `PE1` → `PE14`; `PE14` ligado a `INJ_EN` e `PE3` a `IGN_EN` do TLE.
4. ✅ Power no esquemático **e** no PCB (`openems_ecu.kicad_pcb`):
   - pad19=VSSA→GND, pad20=VREF-→AGND, pad49=VSS→GND, pad73=VDDUSB→VDD
   - pad48/98=VCAP + **C100/C25** 2,2 µF (valor confirmado WeAct H562)
   - Traces F407 (`/PE1`, `/PB11`, VDD no pad19) removidos nos 6 pads; vias GND nos retornos VCAP
   - ⚠️ **DRC visual no Pcbnew** ainda recomendado antes de Gerber (zonas/refill + clearance)

## Suporte elétrico dos 6 pinos alterados (H562, não copiar valores do F407)

| Pino | Tratamento |
|------|-----------|
| `VSSA` (19) | Amarrado a `AGND`/`GND` local, junto ao plano analógico — mesmo tratamento que o F407 já dava a `VSSA` no pad 20 |
| `VREF-` (20) | Amarrado a `AGND`/`GND` — não é referência ativa, é o "terra" do ADC |
| `VREF+` (21) | **Sem mudança de decisão** — [[vref-plus-vdda-filtered]]: VDDA filtrado, cap dedicado DNP |
| `VCAP` ×2 (48, 98) | Cap cerâmico **2,2 µF X7R** a GND cada, o mais próximo possível do pino. **Confirmado** pelo esquemático WeAct H562VGT6 V1.0 (`docs/hw/weact_h562_v10_schdoc.pdf`: C9/C10 = 2.2uF em pads 48/98) |
| `VDDUSB` (73) | Alimentar do `VDD` 3V3 local + decoupling (no board mRE: C26 2,2 µF reutilizado como bulk VDDUSB; 100 nF local opcional) |

## O que NÃO muda

- Todo o mapa `INJ1-4=PE0/2/4/6`, `IGN1-4=PE9/11/13/15`, `Pump=PE10`,
  `Fan=PE12`, `CKP=PA0`, `CMP=PA1`, `ETB PE5/7/8`, ADC (`PA2-5`, `PB0/1`,
  `PC0-5`), SPI2 TLE8888 (`PB12-15`), CAN (`PB8/9`), USB (`PA11/12`), UART
  (`PA9/10`) — pinos e números **idênticos** entre F407 e H562 no LQFP100.
- Footprint do MCU no `.kicad_pcb` (`Package_QFP:LQFP-100_14x14mm_P0.5mm`) —
  zero rework de land pattern.

## Ver também

- [[hw-interface-board-v1-plan]], [[injen-pe1-not-bonded-moved-pe14]]
- `docs/hw/microruseefi_as_base.md` — passo 2 do plano de migração
- `docs/hw/stm32h562_ref.md` §3.1 — pad 98 = VCAP

## Enables TLE no PCB (mRE → OpenEMS)

No layout microRusEFI, `INJ_EN`/`IGN_EN` do TLE8888 iam a `PD11`/`PD10`. OpenEMS usa:

| Sinal TLE | MCU OpenEMS | Net PCB | Nota |
|-----------|-------------|---------|------|
| `INJ_EN` (U2.24) | **PE14** | `/PE14` | alinhado a `out_pins.h` |
| `IGN_EN` (U2.27) | **PE3** | `/PE3` | alinhado a `out_pins.h` |

## MCU ↔ TLE8888 no esquemático (VGT6, netlist verificado)

> 📌 **Esta secção descreve o board CONGELADO** `hardware/openems_ecu/`, no estado em que
> ficou. Não descreve a board de produção. Para *ligações* a autoridade é
> [`netlist_v1.md`](netlist_v1.md); para o estado do board congelado,
> [`hardware/openems_ecu/README.md`](../../hardware/openems_ecu/README.md).

Root sheet `openems_ecu.kicad_sch` alinhado a `out_pins.h` + `netlist_v1.md` / `tle8888_pinout.md`:

### Direct drive + enables (Port E)

| Função | MCU | TLE pin | Símbolo |
|--------|-----|---------|---------|
| INJ1–4 | PE0 / PE2 / PE4 / PE6 | 28–31 | IN1–IN4 |
| IGN1–4 | PE9 / PE11 / PE13 / PE15 | 32–35 | IN5–IN8 |
| PUMP / FAN | PE10 / PE12 | 36 / 37 | IN9 / IN10 |
| INJEN / IGNEN | PE14 / PE3 | 24 / 27 | INJEN / IGNEN |
| IGN outs | — | 96–99 | → Molex J51–J54 |

### SPI2, CAN, CKP, VVT

| Função | MCU | TLE pin | Símbolo |
|--------|-----|---------|---------|
| SPI CS / SCK / MISO / MOSI | PB12 / PB13 / PB14 / PB15 | 3 / 7 / 4 / 5 | CSN / FCLP / SDO / SIP |
| SPI mode | — | 6 → GND, 8 → VDDIO | SIN / FCLN |
| CAN RX / TX | PB8 / PB9 | 43 / 44 | CANRX / CANTX |
| CKP digital | PA0 ← | 21 | VROUT |
| VVT exh / adm | PB6 / PB7 | 38 / 39 | IN11 / IN12 |

### Legado mRE removido destas nets

| Antes (mRE) | Agora |
|-------------|--------|
| PD12–15 → IGN IN5–8 | PE9/11/13/15 |
| PD5 → CSN; PB3/4/5 → SPI | PB12–15 |
| PB6 → CANTX; PB12 → CANRX | PB9 / PB8 |
| PC6 → VROUT | PA0 |
| PE7/8 → IN11/12 | PB6/7; PE7/8 livres (ETB DIR) |
| Flash SPI em PB13–15 | desligado do MCU (SPI2 é só TLE) |
| LIN PD8/9 | desligado (não usado OpenEMS v1) |

### Hierarquia KiCad (sheets)

| Sheet | Estado |
|-------|--------|
| `openems_ecu.kicad_sch` (root) | Nets + labels VGT6; **pinos da sheet TLE** renomeados para GPIO (`PE9`, `PB12`, …) e saídas `IGN1–4` / `INJ1–4` |
| `TLE8888-1QK.kicad_sch` | `hierarchical_label` alinhados aos mesmos nomes (ex-`IGN_IN_1`→`PE9`, ex-`CSN`→`PB12`, ex-`CRNK_IN`→`PA0`) |
| `mcu_h562.kicad_sch` | Já usava nomes GPIO (`PE*`, `PB*`, `PA0`) — sem rename |
| `hi-lo` / `adc` | Pinos pass-through GPIO mantidos (não são o mapa TLE) |

### PCB (`openems_ecu.kicad_pcb`)

| Passo | Estado |
|-------|--------|
| Sync pads do netlist esquemático | Feito (293 pads; 119 nets na **tabela** antes dos footprints) |
| Fix pad C100 (double-net pré-existente mRE) | Feito |
| Remoção de cobre legado | Feito (~567 items) |
| Trilhas manhattan U1↔U2 (21 sinais VGT6) | Feito **F.Cu** 0,25 mm |
| DRC / limpeza | **Pendente no Pcbnew** (L-tracks podem cruzar) |
| Backup limpo pré-re-route | `openems_ecu.kicad_pcb.bak-preroute-vgt6` |

**Nota crash:** a 1ª tentativa de re-route inseriu nets no sítio errado (depois do último pad do ficheiro) e corrompeu o `.kicad_pcb` → SIGSEGV no `LoadBoard`. Re-route seguro restaura o backup e só acrescenta nets na tabela oficial.

Abrir no Pcbnew → DRC → corrigir clearance nas nets `/PE*`, `/PB12–15`, `/PB8/9`, `/PA0`, `/PB6/7`.
