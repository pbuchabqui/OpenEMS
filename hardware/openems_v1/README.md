# OpenEMS ECU v1 — KiCad (produção)

Projecto KiCad **desenhado de raiz**. Nenhum cobre herdado.
Substitui o fork do microRusEFI em `../openems_ecu/` (congelado desde 2026-08-07 —
porquê em [`docs/hw/microruseefi_as_base.md`](../../docs/hw/microruseefi_as_base.md)).

Arquitectura activa: **v2** — TLE8888 saiu; entraram TPS65381A-Q1 / MC33810 / L9960T /
CJ125. Autoridade de pinos: [`docs/hw/pinout_v2.md`](../../docs/hw/pinout_v2.md).
Arquitectura: [`docs/hw/architecture_v2.md`](../../docs/hw/architecture_v2.md).

```bash
kicad hardware/openems_v1/openems_v1.kicad_pro
```

## Fase actual — só esquemático profissional

**Âmbito agora:** sheets KiCad correctas, legíveis, ERC limpo, docs de bloco alinhadas
a `pinout_v2.md`. Definition of done por sheet em
[`docs/hw/schematic/README.md`](../../docs/hw/schematic/README.md).

**Fora de âmbito até o esquemático fechar:**

- Placement, copper, pours, DRC de PCB, gerbers, PnP
- Footprint AMPSEAL 23 (bloqueia **layout** da sheet 09, não o desenho eléctrico do mapa
  de pinos no esquemático)
- Drivers de firmware dos CIs novos

O `.kicad_pcb` existe só como moldura (contorno + stackup de partida). **Não se trabalha
nele nesta fase.**

## Estado (2026-08-08)

| Item | Estado |
|---|---|
| Hierarquia root (ficheiros legacy) | ✅ existe; **renomear** para hierarquia v2 (abaixo) |
| Stackup 4 camadas + contorno provisório | ✅ moldura only |
| Netclasses de partida | ✅ (valores a rever na fase PCB) |
| Sheet `01_power` | ✅ desenhada 2026-08-08, ERC 0 erros (13 avisos esperados) |
| Sheet `11_mcu_h562` | 🔄 desenhada, **limpeza ERC / title block** pendente |
| Sheets `03_cmp`, `07_flex` | ⚠️ rascunho — rever pinos e qualidade |
| Sheet `05_analog` | ⛔ rascunho no **mapa pré-v2** — **refazer**, não remendar |
| Restantes sheets | ❌ vazias |
| Símbolos TPS65381A / MC33810 / L9960T / CJ125 | ❌ em falta (bloqueiam sheets 04*) |
| PCB / copper | ⏸️ diferido |

## Hierarquia alvo (v2)

O root no disco ainda usa nomes legacy (`04_tle8888_hub`, `06_etb`). A tabela abaixo é a
**hierarquia que se vai materializar** no KiCad — zero cobre a perder ao renomear.

| Sheet | Ficheiro alvo | Função | Doc | Estado |
|---|---|---|---|---|
| 01 | `01_power` | Protecção VBAT (P-FET, fusível, TVS, bulk) + star GND | `01_power.md` ✅ | ✅ sheet 2026-08-08 (ERC 0, F1/Q1/R1/R2/D1/C1/C2) |
| 02 | `02_ckp` | CKP Hall → `PA0` (espelho do CMP) | `02_ckp.md` ✅ | ✅ sheet 2026-08-08 (ERC 0) |
| 03 | `03_cmp` | CMP Hall → `PA1` | `03_cmp.md` ✅ | ⚠️ rascunho (alinhado ao CKP) |
| 04a | `04_pmic_tps65381` | TPS65381A-Q1 (rails, SPI, WD, ENDRV) | *a escrever* | ❌ + falta símbolo |
| 04b | `04_drive_mc33810` | MC33810 INJ/IGN | *a escrever* | ❌ + falta símbolo |
| 04c | `04_bridge_l9960t` | L9960T ETB+EWG (substitui `06_etb`) | *a escrever* | ❌ + falta símbolo |
| 04d | `04_wbo2_cj125` | CJ125 + heater | *a escrever* | ❌ + falta símbolo |
| 05 | `05_analog` | ADC conforme **pinout_v2** (13 ch) | `05_analog.md` ✅ | ✅ sheet 2026-08-08, ERC 0, netlist 13/13 |
| 07 | `07_flex` | Flex → **`PB4`** (pinout_v2) | `07_flex.md` ✅ pino | ⚠️ rascunho (pino corrigido) |
| 08 | `08_usb` | USB isolado | `08_usb.md` | ❌ vazio |
| 09 | `09_connectors` | AMPSEAL J1/J2 mapa de pinos | `09_connectors_weact.md` | ❌ vazio |
| 10 | `10_knock_dnp` | Knock DNP | `10_knock_dnp.md` | ❌ vazio |
| 11 | `11_mcu_h562` | MCU H562VGT6 local | `11_mcu_h562.md` ✅ | ✅ netlist U1 OK (2026-08-08); globals sync PA0/PA1/PB4 |
| 12 | `12_can` (recomendado) | FDCAN1 + ESD | *a escrever* | ❌ |

Docs de bloco: [`docs/hw/schematic/`](../../docs/hw/schematic/).

### Sheet 11 (MCU) — o que já está fechado no papel

- Pinos verificados contra o symbol oficial; VCAP 2×2,2 µF (WeAct H562 real).
- HSE: `3225-8.00-10-10-10/A` (CL 10 pF), Cload 12 pF/12 pF C0G.
- Sheet desenhada (U1 + decoupling + cristal + BOOT0 + SWD + Y2 DNP).
- ERC sheet isolado: 1 erro agregado de pinos GPIO por ligar (esperado).

### ✅ Netlist U1 corrigido (2026-08-08)

**Causa:** em `lib_symbols` o símbolo estava como `"STM32H562VGTx"` mas a instância
tinha `lib_id "STM32H562VGTx:STM32H562VGTx"` — o KiCad não resolvia os pinos
(`Net-(U1-Pad??)`, 100 pinos na mesma net).

**Fix:** `lib_id` da instância U1 → `"STM32H562VGTx"` (bate com o nome embutido).

**Validado no netlist de projecto:**
- `MCU.PA0` → U1/23 + R13/C2 (sheet 02_ckp)
- `MCU.PA1` → U1/24 + R4/C1 (sheet 03_cmp)
- `MCU.PB4` → U1/90 + flex (sheet 07)

Globals `MCU.PA0` / `PA1` / `PB4` colocados na sheet 11. GPIO ainda sem sheet destino
aparecem como `unconnected-(U1-…)` no netlist — esperado até as outras sheets.

## Convenções de nets (esquemático)

- Rails: `+3V3`, `+5V_SENS_A`, `+5V_SENS_B`, `VBAT`, `VDDA`, `PGND`, `SGND`, `AGND`
- MCU inter-sheet: global labels `MCU.<PORTn>` (ex. `MCU.PA1`)
- Conector: `J1.*` / `J2.*`
- Locais de sheet: labels locais `N_*`
- **Um** `PWR_FLAG` por rail, na sheet que **produz** o rail (PMIC / `01_power`)
- Root = índice de sheets; **não** hierarchical pins nesta fase

Autoridade de pinos: **só** [`pinout_v2.md`](../../docs/hw/pinout_v2.md).

## Netclasses (partida — fase PCB)

| Classe | Trace | Clearance | Nota |
|---|---|---|---|
| `Default` | 0,25 mm | 0,20 mm | lógica, SPI, GPIO |
| `Power` | 1,00 mm | 0,35 mm | VBAT, rails |
| `Injector` | 0,80 mm | 0,30 mm | ⚠️ MC33810 4,5 A — rever IPC-2152 no layout |
| `CKP_VR` | 0,25 mm | 0,40 mm | ⚠️ nome legacy; CKP=Hall — provavelmente fundir com Default |
| `CAN` | 0,25 mm | 0,25 mm | par diferencial |
| `Analog` | 0,25 mm | 0,30 mm | divisores ADC |

## Libs

| Lib | Uso v2 |
|---|---|
| `MCU_ST_STM32H5` (KiCad stock) + `lib/STM32H562VGTx.kicad_sym` | MCU |
| `pesd1can.kicad_sym` | ESD CAN |
| `rusefi_ref.pretty` | AMPSEAL 35 (+ net-tie) |
| `tle8888qk.kicad_sym` | ⛔ referência só — TLE saiu |
| TPS65381A / MC33810 / L9960T / CJ125 | ❌ **a criar** a partir de datasheet, um CI de cada vez |

Footprint AMPSEAL 23: desenhar na **fase PCB** a partir de
[`docs/hw/TE_770669_header_RA_23.pdf`](../../docs/hw/TE_770669_header_RA_23.pdf).

## Ordem de trabalho (esquemático)

Regra §1b de [`docs/hw/README.md`](../../docs/hw/README.md) em **todo** o bloco:
consultar Speeduino/rusEFI; registar adoptamos / adaptamos / rejeitamos.

1. ~~Doc + cristal + desenho inicial `11_mcu_h562`~~ ✅ (limpeza ERC ainda aberta)
2. ~~Limpar sheet 11~~ ✅ (netlist U1 corrigido 2026-08-08)
3. ~~CKP/CMP Hall~~ ✅ `02_ckp` ERC 0; `03_cmp` alinhado
4. ~~Flex em `PB4`~~ ✅ pino corrigido
5. ~~Refazer `05_analog`~~ ✅ ERC 0, netlist 13/13 (2026-08-08)
6. **Hierarquia root v2** — renomear/substituir `04_tle8888_hub` e `06_etb` pelos 04a–d
7. ~~`01_power`~~ ✅ desenhada 2026-08-08 (ERC 0: F1/Q1/R1/R2/D1/C1/C2, star GND flags). **Falta ainda:** símbolo + sheet PMIC (`04_pmic_tps65381`)
8. **MC33810 → L9960T → CJ125 → CAN → USB → conectores → knock DNP**
9. ERC projecto completo + PDF + BOM schematic-level
10. **Só depois:** fase PCB (footprints em falta, placement, pours, DRC)

## O que **não** fazer nesta fase

- Não copiar cobre de `../openems_ecu/`
- Não montar WeAct na ECU de motor (só bancada de firmware)
- Não tratar netclasses como fechadas
- Não inventar pinos/símbolos de CI de memória — datasheet + §1
- Não abrir layout/copper até o DoD do esquemático
- Não misturar experimentação tscircuit (raiz do repo) com este projecto KiCad
