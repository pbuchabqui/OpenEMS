# OpenEMS ECU v1 — KiCad (produção)

Projecto KiCad **desenhado de raiz**. Nenhum cobre herdado.
Substitui o fork do microRusEFI em `../openems_ecu/` (congelado desde 2026-08-07 —
porquê em [`docs/hw/microruseefi_as_base.md`](../../docs/hw/microruseefi_as_base.md)).

```bash
kicad hardware/openems_v1/openems_v1.kicad_pro
```

## Estado — esqueleto, sem um único componente colocado

O que existe é a moldura. **Nenhum bloco foi desenhado ainda.**

| Item | Estado |
|---|---|
| Hierarquia de sheets (12) | ✅ criada, todas vazias |
| Stackup 4 camadas (`F.Cu`, `In1.Cu`, `In2.Cu`, `B.Cu`) | ✅ |
| Contorno 160 × 110 mm, cantos R3 | ✅ provisório |
| Design rules + 5 netclasses | ✅ valores de partida |
| Libs verificadas registadas | ✅ (2 símbolos, 4 footprints) |
| Componentes, nets, cobre | ❌ **nada** |

## Hierarquia

Espelha [`docs/hw/schematic/`](../../docs/hw/schematic/) — um sheet por bloco,
para revisão barata e paralelizável.

| Sheet | Bloco | Doc |
|---|---|---|
| `sheets/01_power` | Alimentação — protecção, buck, LDO, rails de sensor | `01_power.md` |
| `sheets/02_ckp` | CKP — front-end VR pela interface do TLE8888 | `02_ckp.md` |
| `sheets/03_cmp` | CMP — Hall directo ao `PA1` | `03_cmp.md` |
| `sheets/04_tle8888_hub` | Hub TLE8888-2QK — SPI, INJ, IGN, VVT, relés, CAN | `04_tle8888_hub.md` |
| `sheets/05_analog` | Condicionamento ADC — divisores, clamps, VBATT `PC3` | `05_analog.md` |
| `sheets/06_etb` | ETB — ponte-H BTS7960 externa, PWM 10 kHz | `06_etb.md` |
| `sheets/07_flex` | Flex fuel | `07_flex.md` |
| `sheets/08_usb` | USB com isolador galvânico | `08_usb.md` |
| `sheets/09_connectors` | AMPSEAL J1 (35 sinais) + J2 (23 potência) | `09_connectors_weact.md` |
| `sheets/10_knock_dnp` | Knock — só footprint, DNP na v1 | `10_knock_dnp.md` |
| `sheets/11_mcu_h562` | **MCU H562VGT6** — decoupling, cristal, BOOT0, SWD, VCAP, VREF+ | ⚠️ **não existe** |

⚠️ A sheet 11 não tem doc de bloco. No fork mRE o MCU vinha herdado (`stm32.kicad_sch`);
num projecto em branco tem de ser desenhado, e o `docs/hw/schematic/` nunca teve um bloco
para ele. Escrever `11_mcu_h562.md` **antes** de desenhar a sheet.

## Netclasses (valores de partida, não finais)

| Classe | Trace | Clearance | Para |
|---|---|---|---|
| `Default` | 0,25 mm | 0,20 mm | lógica, SPI, GPIO |
| `Power` | 1,00 mm | 0,35 mm | VBAT, +12 V, rails principais |
| `Injector` | 0,80 mm | 0,30 mm | saídas INJ do TLE (2,2 A) |
| `CKP_VR` | 0,25 mm | 0,40 mm | par VR do CKP — clearance folgada de propósito |
| `CAN` | 0,25 mm | 0,25 mm | par diferencial, gap 0,2 mm |
| `Analog` | 0,25 mm | 0,30 mm | divisores e entradas de ADC |

As larguras vêm de regra de bolso, **não de cálculo de subida de temperatura**.
Refazer com IPC-2152 quando as correntes reais estiverem fechadas
(ver [`bom_v1_candidates.md`](../../docs/hw/bom_v1_candidates.md)).

## Libs

`lib/` tem **só o que esta board usa** — o catálogo rusEFI completo (76 libs de símbolos,
conectores Bosch/Nissan, joysticks) foi deliberadamente deixado de fora.

| Lib | Conteúdo | Proveniência |
|---|---|---|
| `tle8888qk.kicad_sym` | TLE8888 LQFP-100 | rusEFI; pinout verificado DS Rev 1.2 |
| `pesd1can.kicad_sym` | PESD1CAN, ESD do CAN | rusEFI |
| `rusefi_ref.pretty` | `AMPSEAL_35_{RA,STRAIGHT,COMBINED}`, `Net-Tie_2_8.5mil` | rusEFI |

O MCU vem da lib oficial do KiCad (`MCU_ST_STM32H5:STM32H562VGTx`) — é a mesma fonte
com que o mapa pino-a-pino foi verificado.

### ⚠️ Falta o footprint do AMPSEAL 23

A decisão do conector é **35 vias (sinais) + 23 vias (potência)**. O footprint de 35 existe
e está verificado; **o de 23 não existe** em lado nenhum do repo — só há o modelo 3D
(`../openems_ecu/rusefi_lib_external/3d/AMPSEAL_23_STRAIGHT.stp`).

Tem de ser desenhado a partir de [`docs/hw/TE_770669_header_RA_23.pdf`](../../docs/hw/TE_770669_header_RA_23.pdf)
(página *RECOMMENDED P.C. BOARD LAYOUT*). **Bloqueia a sheet 09.**

## Ordem de trabalho

A regra §1b do [`docs/hw/README.md`](../../docs/hw/README.md) é obrigatória e aplica-se
agora a **todos** os blocos, não só aos novos: consultar Speeduino/rusEFI antes de
desenhar, e registar o que se adopta, adapta e rejeita.

1. Escrever `docs/hw/schematic/11_mcu_h562.md`
2. Desenhar `11_mcu_h562` → é o que dá pinos a todos os outros blocos
3. `04_tle8888_hub` → o hub de que quase tudo depende
4. `01_power` → sem rails nada mais fecha
5. Restantes blocos, ERC por sheet à medida
6. Só depois: footprints, placement, pours, DRC

## O que **não** fazer

- Não copiar cobre do `../openems_ecu/` — foi por isso que se recomeçou.
- Não montar WeAct na ECU de motor (grau consumidor + headers). WeAct é **só bancada**.
- Não tratar as netclasses acima como fechadas.
