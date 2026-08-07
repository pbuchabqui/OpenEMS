# OpenEMS ECU v1 — KiCad (produção)

Projecto KiCad **desenhado de raiz**. Nenhum cobre herdado.
Substitui o fork do microRusEFI em `../openems_ecu/` (congelado desde 2026-08-07 —
porquê em [`docs/hw/microruseefi_as_base.md`](../../docs/hw/microruseefi_as_base.md)).

⚠️ **Este ficheiro é do esqueleto inicial (v1 do plano, antes da troca de CIs).** A
arquitectura activa é a **v2** — TLE8888 saiu, entraram TPS65381A-Q1/MC33810/L9960T/CJ125
— ver [`docs/hw/architecture_v2.md`](../../docs/hw/architecture_v2.md) e
[`docs/hw/pinout_v2.md`](../../docs/hw/pinout_v2.md), a autoridade de pinos. A hierarquia
de sheets abaixo é a que foi **criada** (nomes de ficheiro reais); a coluna **Estado v2**
diz o que mudou em cada uma.

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

| Sheet | Bloco | Doc | Estado v2 |
|---|---|---|---|
| `sheets/01_power` | Alimentação — protecção, buck, LDO, rails de sensor | `01_power.md` | 🔄 buck/LDO agora dentro do **TPS65381A-Q1**; doc não revisto |
| `sheets/02_ckp` | ~~CKP — front-end VR pela interface do TLE8888~~ | `02_ckp.md` ⚠️ stale | ⛔ **CKP passa a Hall**, mesmo condicionamento do `03_cmp` |
| `sheets/03_cmp` | CMP — Hall directo ao `PA1` | `03_cmp.md` | ✅ sobrevive |
| `sheets/04_tle8888_hub` | ~~Hub TLE8888-2QK — SPI, INJ, IGN, VVT, relés, CAN~~ | `04_tle8888_hub.md` ⛔ anulado | ⛔ **splita em 4**: MC33810 (INJ/IGN), L9960T (ver sheet 06), CJ125 (wideband), TPS65381A (PMIC) + expansor de I/O SPI. Nenhum doc de bloco novo escrito |
| `sheets/05_analog` | Condicionamento ADC — divisores, clamps, VBATT `PC3` | `05_analog.md` | 🔄 canais novos entram (`LAMBDA_UA/UR`, `APP1/2`, `EWG_POS`) — doc não revisto |
| `sheets/06_etb` | ~~ETB — ponte-H BTS7960 externa, PWM 10 kHz~~ | `06_etb.md` ⚠️ stale | 🔄 **L9960T** substitui o BTS7960, serve ETB **e** EWG |
| `sheets/07_flex` | Flex fuel | `07_flex.md` | ✅ sobrevive |
| `sheets/08_usb` | USB com isolador galvânico | `08_usb.md` | ✅ sobrevive |
| `sheets/09_connectors` | AMPSEAL J1 (35 sinais) + J2 (23 potência) | `09_connectors_weact.md` | ✅ sobrevive — footprint do J2 (23 vias) ainda não existe, ver abaixo |
| `sheets/10_knock_dnp` | Knock — só footprint, DNP na v1 | `10_knock_dnp.md` | ✅ sobrevive, continua diferido |
| `sheets/11_mcu_h562` | **MCU H562VGT6** — decoupling, cristal, BOOT0, SWD, VCAP, VREF+ | ⚠️ **não existe** | — |

⚠️ A sheet 11 não tem doc de bloco. No fork mRE o MCU vinha herdado (`stm32.kicad_sch`);
num projecto em branco tem de ser desenhado, e o `docs/hw/schematic/` nunca teve um bloco
para ele. Escrever `11_mcu_h562.md` **antes** de desenhar a sheet.

⚠️ **A sheet 04 é a maior lacuna estrutural do esqueleto.** Foi criada a pensar num hub
único; a arquitectura v2 não tem hub — tem quatro CIs cada um com o seu SPI CS. A sheet
existe no ficheiro (`sheets/04_tle8888_hub.kicad_sch`, vazia) mas **o nome e a hierarquia
não foram actualizados**: continua por decidir se isto vira 4 sheets novas ou se `04` é
renomeada e as outras 3 se somam à lista. Nenhuma destas sheets foi desenhada, então não
há cobre/ligações a perder ao decidir — é só uma decisão de organização, adiada.

## Netclasses (valores de partida, não finais)

| Classe | Trace | Clearance | Para |
|---|---|---|---|
| `Default` | 0,25 mm | 0,20 mm | lógica, SPI, GPIO |
| `Power` | 1,00 mm | 0,35 mm | VBAT, +12 V, rails principais |
| `Injector` | 0,80 mm | 0,30 mm | ⚠️ dimensionada para **2,2 A** (saída INJ do TLE, que saiu). O MC33810 tem saídas de **4,5 A** — revisitar largura com IPC-2152 antes de rotear |
| `CKP_VR` | 0,25 mm | 0,40 mm | ⚠️ **nome desactualizado** — foi pensada para o par diferencial VR do CKP. Com CKP=Hall (v2), o sinal é single-ended, igual ao CMP — provavelmente não precisa de classe própria. Decidir ao desenhar `02_ckp` |
| `CAN` | 0,25 mm | 0,25 mm | par diferencial, gap 0,2 mm — sobrevive, FDCAN1 continua diferencial |
| `Analog` | 0,25 mm | 0,30 mm | divisores e entradas de ADC |

As larguras vêm de regra de bolso, **não de cálculo de subida de temperatura**.
Refazer com IPC-2152 quando as correntes reais estiverem fechadas
(ver [`bom_v1_candidates.md`](../../docs/hw/bom_v1_candidates.md)) — **e depois da
troca de arquitectura, `Injector` e `CKP_VR` são as duas que mais provavelmente mudam**.

## Libs

`lib/` tem **só o que esta board usa** — o catálogo rusEFI completo (76 libs de símbolos,
conectores Bosch/Nissan, joysticks) foi deliberadamente deixado de fora.

| Lib | Conteúdo | Proveniência | Uso na v2 |
|---|---|---|---|
| `tle8888qk.kicad_sym` | TLE8888 LQFP-100 | rusEFI; pinout verificado DS Rev 1.2 | ⛔ **nenhum** — TLE8888 saiu da arquitectura. Mantido só como referência do que substituir |
| `pesd1can.kicad_sym` | PESD1CAN, ESD do CAN | rusEFI | ✅ CAN continua a existir (FDCAN1) |
| `rusefi_ref.pretty` | `AMPSEAL_35_{RA,STRAIGHT,COMBINED}`, `Net-Tie_2_8.5mil` | rusEFI | ✅ conector sobrevive intacto |

O MCU vem da lib oficial do KiCad (`MCU_ST_STM32H5:STM32H562VGTx`) — é a mesma fonte
com que o mapa pino-a-pino foi verificado.

⚠️ **Faltam os símbolos dos 4 CIs da v2** — TPS65381A-Q1, MC33810, L9960T, CJ125. Nenhum
está na lib do rusEFI (são famílias diferentes das que o mRE usava) e **nenhum foi
verificado ainda**. Não inventar footprint/símbolo a partir de memória — a regra §1 do
`docs/hw/README.md` existe por causa exactamente disto. Procurar lib oficial do
fabricante ou desenhar contra o datasheet, um CI de cada vez, ao chegar a vez do bloco.

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
3. ~~`04_tle8888_hub` → o hub de que quase tudo depende~~ **sem sentido na v2** — não há
   hub único. Decidir primeiro a divisão de sheets (ver aviso acima), *depois*:
   - `TPS65381A-Q1` (PMIC) → é o que dá `+3V3`/`+5V` a tudo o resto, faz as vezes do
     antigo passo "01_power depende disto"
   - `MC33810` (INJ/IGN) → mesmo princípio do antigo hub: entradas directas no GPIOE,
     não altera o scheduler
   - `L9960T` (ETB+EWG), `CJ125` (wideband) → sem dependência entre si, paralelizáveis
4. `01_power` → revisto: buck/LDO já vêm do TPS65381A, este sheet fica mais fino
   (protecção de entrada + o que o PMIC não cobre)
5. Restantes blocos, ERC por sheet à medida
6. Só depois: footprints, placement, pours, DRC

Pré-requisito silencioso destes passos: os símbolos dos 4 CIs (ver "Libs" acima) não
existem. Sem eles, nenhum dos passos 3 chega a ter um componente para colocar.

## O que **não** fazer

- Não copiar cobre do `../openems_ecu/` — foi por isso que se recomeçou.
- Não montar WeAct na ECU de motor (grau consumidor + headers). WeAct é **só bancada**.
- Não tratar as netclasses acima como fechadas.
