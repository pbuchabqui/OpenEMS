# Hardware — índice

> **Status:** vigente · validado 2026-11-17 (contra `src/hal/out_pins.h`, `docs/ROADMAP.md` e `docs/hw/stm32h562_ref.md`)

## Escopo atual

Firmware e **bancada**: WeAct STM32H562 **VGT6** (LQFP100) + ESP32 como estimulador, **sem atuadores**
(`CLAUDE.md`, `docs/ROADMAP.md`).
Fora do projeto (não faz parte): a placa de produção v1 ou v2 e os CIs TLE8888, MC33810, L9960T, CJ125 e TPS65381A.
INJ/IGN/ETB saem **direto por GPIO**, e o repositório não define nenhum driver de potência a jusante.

⚠️ Antes de ligar qualquer atuador real é preciso projetar e validar esse estágio. No reset os GPIOs **não** são
saídas seguras: no RGT6, `PA15` (JTDI) e `PB4` (NJTRST) resetam com pull-up e `PB3` (JTDO) sem pull;
na VGT6 a porta E reseta em analógico (flutuante). Um pull-down externo em cada entrada de driver de
INJ/IGN/ETB é requisito de hardware (fonte: RM0481, cap. GPIO).

**Chip:** a revisão do silício da placa de bancada foi lida em 2026-11-17 pelo comando `'i'`:
`DBGMCU_IDCODE = 0x10076484` → REV_ID `0x1007` = **Rev X**, DEV_ID `0x484` (ver `stm32h562_ref.md` §5).

## Autoridade — qual fonte manda sobre o quê

**Código ganha de documento. Se divergirem, o documento está errado.** O "porquê" de uma decisão vive num
documento só; nenhum outro o repete.

| Assunto | Fonte de verdade |
|---|---|
| Mapa de pinos RGT6 × VGT6 | `pinout.md` (e `static_assert` em `src/hal/out_pins.h`) |
| INJ/IGN, enables, BSRR | `src/hal/out_pins.h` |
| Canais e pinos de ADC | `src/hal/adc.h` |
| Registradores, memória e periféricos do MCU | `stm32h562_ref.md`, conferido contra o `stm32h562xx.h` oficial da ST e o RM0481 |
| Errata do silício | `../errata_stm32h562.md` (ES0565) |
| Placa de bancada WeAct | `weact_h562_coreboard.md` (forma, furos, headers), `weact_h562_schematic.md` + `weact_h562_v10_schdoc.pdf` |

## Documentos desta pasta

| Documento | Status |
|---|---|
| `pinout.md`, `stm32h562_ref.md`, `weact_h562_coreboard.md`, `weact_h562_schematic.md` | vigente |
| `interface_board_v1.md`, `netlist_v1.md`, `bom_v1_candidates.md`, `pinmap_logical.md`, `vr_input_conditioning.md`, `tle8888_pinout.md`, `tle8888_crosscheck.md`, `microruseefi_as_base.md`, `kicad_vendor_review.md`, `mcu_f407_to_h562_pin_map.md`, `decisoes_placa_historico.md` | histórico |
| `architecture_v2.md`, `pinout_v2.md`, `ampseal_connectors.md`, `schematic/` | fora-de-escopo |

O status de cada arquivo está no cabeçalho dele.

## Ordem de validação

Bancada → ETB validado (desacoplado do motor) → motor. Cada fase só começa quando a anterior estiver
fechada (`docs/ROADMAP.md`; procedimento de bancada em `../bench_test_manual.md`).

## Como manter a documentação sem obsoletos

- Todo documento em `docs/` tem, no topo, `> **Status:** vigente · validado AAAA-MM-DD`, ou `histórico` /
  `fora-de-escopo` com o porquê e onde está a fonte atual.
- `make doc-check` (parte do `make ci-local`) falha se: faltar o status; um `vigente` citar caminho, alvo de
  `make` ou símbolo que não existe; mencionar tecnologia removida como se estivesse em uso; trouxer
  contagem de testes ou hash de `main` no texto; ou reintroduzir um fato já provado errado.
- Achou um erro factual num documento? Corrija e **acrescente o padrão em `tools/doc_facts.txt`** para ele não voltar.
- Mudou comportamento, pino, constante ou comando? Atualize o documento no mesmo commit e a data de `validado`.
