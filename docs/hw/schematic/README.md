# Esquemático modular — OpenEMS ECU v2

⚠️ **Arquitectura activa é a v2** (2026-08-07) — TLE8888 saiu; entraram
TPS65381A-Q1 / MC33810 / L9960T / CJ125. Ver [`../architecture_v2.md`](../architecture_v2.md).

## Fase actual — só esquemático profissional

**Faz-se agora:** docs de bloco + sheets KiCad + ERC + PDF/BOM schematic.

**Não se faz agora:** layout, copper, pours, DRC de PCB, gerbers. Footprint AMPSEAL 23
é dívida da **fase PCB**, não bloqueia o mapa eléctrico no esquemático.

Projecto: `hardware/openems_v1/openems_v1.kicad_pro`  
Autoridade de pinos: [`../pinout_v2.md`](../pinout_v2.md)  
Porquê / arquitectura: [`../architecture_v2.md`](../architecture_v2.md)  
Nets pré-v2 (só blocos que sobreviveram): `../netlist_v1.md` — **não** copiar blocos TLE.

## Definition of done (por sheet)

Um sheet só fecha quando cumprir **todos**:

| Critério | Nota |
|---|---|
| Correctness | Pinos/nets batem com `pinout_v2.md` |
| §1b | Doc com Adoptamos / Adaptamos / Rejeitamos (Speeduino/rusEFI ou DS) |
| ERC | 0 erros estruturais no sheet |
| Legibilidade | Fios + junctions; power symbols; globals só inter-sheet |
| Naming | Rails `+3V3`/`+5V_SENS_*`/`VBAT`/`*GND`; `MCU.*`; `J1.*`/`J2.*` |
| Apresentação | `title_block`; valores + PN; DNP marcado |
| Doc sync | Checklist do `.md` + este README actualizados |
| Sem PCB | Não se faz placement/copper |

**DoD do projecto (fase esquemático):** hierarquia v2 desenhada + ERC projecto sem erros
bloqueantes + PDF multi-sheet + BOM schematic. Layout = fase seguinte.

## Convenções

- **Global labels** para nets inter-sheet (`MCU.*`, `J1.*`, rails). Root = índice.
- **Um** `PWR_FLAG` por rail, na sheet que **produz** o rail.
- Não misturar hierarchical pins e globals na mesma net nesta fase.

## Docs de bloco e estado

| Ordem | Sheet alvo | Ficheiro doc | Estado doc | Estado KiCad |
|------:|------------|--------------|------------|--------------|
| 0 | Raiz | este README | ✅ | hierarquia legacy no disco |
| 1 | `01_power` | `01_power.md` | 🔄 stale (ainda fala em TLE/buck) | ❌ vazio |
| 2 | `02_ckp` | `02_ckp.md` | ✅ Hall (espelho CMP) | ✅ sheet 2026-08-08, ERC 0 |
| 3 | `03_cmp` | `03_cmp.md` | ✅ espelho CKP | ⚠️ rascunho (título ok; net MCU bloqueada) |
| 4a | `04_pmic_tps65381` | *a escrever* | ❌ | ❌ + símbolo |
| 4b | `04_drive_mc33810` | *a escrever* | ❌ | ❌ + símbolo |
| 4c | `04_bridge_l9960t` | *a escrever* (substitui `06_etb.md`) | ⚠️ `06_etb.md` stale | ❌ + símbolo |
| 4d | `04_wbo2_cj125` | *a escrever* | ❌ | ❌ + símbolo |
| 5 | `05_analog` | `05_analog.md` | ✅ pinout_v2 13 ch | ✅ sheet 2026-08-08, ERC 0 |
| 7 | `07_flex` | `07_flex.md` | 🔄 pino → **PB4** | ⚠️ rascunho (PB5 errado) |
| 8 | `08_usb` | `08_usb.md` | ✅ esboço | ❌ vazio |
| 9 | `09_connectors` | `09_connectors_weact.md` | ✅ | ❌ vazio |
| 10 | `10_knock_dnp` | `10_knock_dnp.md` | ✅ | ❌ vazio |
| 11 | `11_mcu_h562` | `11_mcu_h562.md` | ✅ | 🔄 limpeza ERC |
| 12 | `12_can` | *a escrever* | ❌ | ❌ |

Docs anulados / stale a não seguir:

- `04_tle8888_hub.md` — ⛔ anulado (TLE saiu)
- `06_etb.md` — ⚠️ stale (BTS7960); conteúdo migra para `04_bridge_l9960t`
- `02_ckp.md` — ⚠️ até ser reescrito como Hall

## Hierarquia KiCad (alvo)

```
openems_v1.kicad_sch                      (root — índice)
├── sheets/01_power.kicad_sch
├── sheets/02_ckp.kicad_sch               ← ✅ Hall, espelho CMP (2026-08-08)
├── sheets/03_cmp.kicad_sch               ← ⚠️ rascunho
├── sheets/04_pmic_tps65381.kicad_sch     ← substitui parte de 01 + hub
├── sheets/04_drive_mc33810.kicad_sch
├── sheets/04_bridge_l9960t.kicad_sch     ← substitui 06_etb
├── sheets/04_wbo2_cj125.kicad_sch
├── sheets/05_analog.kicad_sch            ← pinout_v2 only
├── sheets/07_flex.kicad_sch              ← PB4
├── sheets/08_usb.kicad_sch
├── sheets/09_connectors.kicad_sch
├── sheets/10_knock_dnp.kicad_sch
├── sheets/11_mcu_h562.kicad_sch          ← desenhada; limpeza
└── sheets/12_can.kicad_sch               ← FDCAN1 + ESD
```

No disco, o root **ainda** aponta a `04_tle8888_hub` e `06_etb` (vazios). Renomear ao
chegar à Fase 3–4 do plano de execução — sem perda de trabalho de copper.

⛔ `hardware/openems_ecu/` (fork mRE) **congelado** — ver
[`../microruseefi_as_base.md`](../microruseefi_as_base.md).

## Processo por módulo

1. Ler (ou reescrever) o `.md` do módulo  
2. Consultar Speeduino/rusEFI + datasheet (§1b em `docs/hw/README.md`)  
3. Confirmar **cada pino MCU** em `pinout_v2.md`  
4. Desenhar **só** esse sheet no KiCad  
5. ERC local (`kicad-cli sch erc`)  
6. Marcar checklist no `.md` e actualizar estado aqui  

## Ordem de execução

1. Limpar sheet 11 (padrão de qualidade)  
2. ~~CKP Hall~~ ✅ doc + sheet (2026-08-08); CMP rascunho a rever  
3. ~~Flex `PB4`~~ ✅ doc + sheet  

4. ~~Refazer `05_analog`~~ ✅ pinout_v2 + netlist 13/13 (2026-08-08)  

5. Reorganizar root + PMIC + `01_power`  
6. MC33810 → L9960T → CJ125 → CAN → USB → conectores → knock  
7. ERC projecto + PDF + BOM  
8. **Fase PCB** (só depois do DoD esquemático)  

## O que NÃO fazer ainda

- Layout / copper / DRC  
- Escolher caixa final  
- Popular knock  
- Inventar símbolos dos 4 CIs sem datasheet  
- Tratar rascunhos `03`/`05`/`07` como “fechados”  
