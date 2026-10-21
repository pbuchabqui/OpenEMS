# OpenEMS

ECU para motores a combustao interna, focada no STM32H562RGT6 (Cortex-M33, C++17). Prioriza baixa latencia, baixo jitter e previsibilidade temporal para injecao, ignicao, sensores e calibracao.

Regras de codigo, gates de qualidade e errata: ver `CLAUDE.md`.

## Build

```bash
make firmware     # .elf/.hex/.bin em /tmp/openems-build
make host-test    # regressao no PC
make ci-local     # todos os gates antes de merge
make clean
```

## Instalacao no motor

Pelo dashboard proprio (`tools/openems_dash`, aba **Install**), com guia passo a passo; alternativa: TunerStudio com `tools/ts/openems.ini`.

## Documentacao

| Documento | Conteudo |
|---|---|
| `docs/bench_test_manual.md` | Teste de bancada na WeAct VGT6, sem atuadores |
| `docs/hw/README.md` | Hardware e pinout |
| `docs/wiring_diagram.md` | Esquematicos eletricos |
| `docs/ROADMAP.md` | Backlog de produto |
| `docs/fueling_system.md`, `closed_loop_fuel.md`, `idle_system.md`, `ignition_system.md`, `protection_system.md`, `output_contract.md` | Subsistemas |
