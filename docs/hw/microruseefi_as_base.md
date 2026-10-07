# microRusEFI como base de hardware OpenEMS — ⛔ SUPERSEDIDO

> **Estado (2026-08-07):** o fork do microRusEFI **deixou de ser a base de desenho**
> da ECU OpenEMS. O desenho de produção passa a ser um projecto KiCad **em branco**:
> [`hardware/openems_v1/`](../../hardware/openems_v1/).
>
> O microRusEFI **continua no repo como referência** (`hardware/vendor/hw_microRusEfi`,
> submódulo) — é a implementação de referência que a regra §1b do
> [`README.md`](README.md) obriga a consultar antes de desenhar cada bloco.
> Mudou o papel: era o **cobre**, agora é a **bibliografia**.

Este documento fica por duas razões: registar o porquê da escolha original e o porquê
do abandono, para nenhuma das duas ser re-litigada. **Nada aqui é autoridade.**

---

## Porquê se escolheu o mRE (2026-07-21)

1. **TLE8888-2QK** já estava no esquemático e no layout — alinhado ao firmware OpenEMS.
2. Hierarquia real (TLE, ADC, MCU, H-bridge, hi-lo, pairs) e **PCB já fabricável**.
3. Evitava reinventar cobre fora de um layout já provado.
4. Hellen-One ficava para depois, como adaptadores PnP por carro.

O raciocínio estava certo em princípio. O que falhou foi a prática.

## Porquê se abandonou (2026-08-07)

O fork trouxe consigo o layout de **outra** ECU, e o custo de o dobrar ao pinout
OpenEMS acabou por ser maior que o de desenhar de novo:

| Sintoma | Medido |
|---|---|
| Cobre legado a remover à mão | ~567 items |
| Pad com duplo-net herdado (C100) | corrigido, mas só foi encontrado por acidente |
| Nets fantasma do mRE ainda no board | ex. `/PE1`, que no H562 LQFP100 é **VCAP** |
| Camadas de sinal | **2** — a decisão v1 pede **4** (pours PGND/SGND/AGND separados) |
| Diff de um único re-route | 103 123 linhas — irrevisável |
| Corrupção do `.kicad_pcb` | 1 vez, SIGSEGV no `LoadBoard`, recuperada por backup |
| DRC | nunca chegou a passar |

O ponto de viragem: **um diff de 103 mil linhas não é revisável**, e num projecto cuja
causa-raiz dos dois piores incidentes foi *autoridade que se dessincronizou em silêncio*,
não se pode aceitar um artefacto onde ninguém consegue ver o que mudou.

A cada bloco redesenhado, a fracção do mRE que sobrevivia encolhia — e cada bloco
custava mais a adaptar do que custaria a desenhar. Nesse ponto o fork só carregava risco.

## O que se aproveita do trabalho feito

Nada disto se perde ao mudar de board:

| Artefacto | Onde |
|---|---|
| As 20 decisões fechadas, com refs de commit | [`README.md`](README.md) §2 |
| Netlist pino-a-pino | [`netlist_v1.md`](netlist_v1.md) |
| Esquemático modular, 10 blocos | [`schematic/`](schematic/) |
| Pinout TLE8888 LQFP-100, verificado DS Rev 1.2 | [`tle8888_pinout.md`](tle8888_pinout.md) |
| Contraprova do mapa de registadores (rusEFI) | [`tle8888_crosscheck.md`](tle8888_crosscheck.md) |
| Conector AMPSEAL 35+23 + footprints | [`ampseal_connectors.md`](ampseal_connectors.md) |
| Libs KiCad verificadas (`tle8888qk.lib`, Net-Tie, AMPSEAL) | `hardware/openems_ecu/rusefi_lib/` |
| Firmware inteiro | `src/` — o pinout de produção sempre foi o VGT6 |

## Decisões que sobreviveram intactas

### MCU: STM32H562 **soldado** (LQFP100) — continua fechado

| Opção | Uso |
|-------|-----|
| **H562 LQFP100 na PCB principal** | **Produção / cabine** |
| WeAct H562 | **Só bancada de firmware** — não entra na board de produção |
| Socket + soldado em paralelo | **Rejeitado** (espaço, BOM, confusão) |

Obrigações de design com MCU soldado: SWD acessível, USB, BOOT0, decoupling por pinos
VDD, cristal conforme firmware H562, silkscreen pin 1.

### Conector e ETB — fechados desde então

O que este documento listava como "ainda aberto" foi decidido e vive no
[`README.md`](README.md) §2: AMPSEAL 35+23, e ETB por **BTS7960 @ 10 kHz**
(o TLE9201 do mRE ficou de fora — o firmware é de 3 pinos, ele é de 2).

## Vendor no monorepo (mantém-se — agora como referência)

```bash
git submodule update --init hardware/vendor/hw_microRusEfi
cd hardware/vendor/hw_microRusEfi && git submodule update --init --recursive
```

Caminho: `hardware/vendor/hw_microRusEfi/`

## O board antigo

`hardware/openems_ecu/` **não foi apagado**. Fica como referência e como recuo se o
desenho novo encalhar. Não recebe mais trabalho — ver
[`hardware/openems_ecu/README.md`](../../hardware/openems_ecu/README.md).

## Créditos

Hardware de referência © rusEFI / microRusEFI contributors. O OpenEMS consulta-o com
atribuição; nenhum cobre do mRE segue para a PCB de produção v1.
