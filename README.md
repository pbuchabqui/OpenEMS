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
| `docs/bench_report_template.md` | Modelo do relatório de bancada (preencher durante o ensaio) |
| `docs/hw/README.md` | Hardware e pinout |
| `docs/wiring_diagram.md` | Esquematicos eletricos |
| `docs/ROADMAP.md` | Backlog, progresso e histórico do que foi concluído |
| `docs/datalog.md` | Datalog em cartão SD (VGT6): formato, anel e ferramentas |
| `docs/nn_viabilidade.md` | Estudo: redes neurais no STM32H562 (X-τ, STFT, PID) |
| `docs/guia_instalacao.md` | Guia de instalação no motor (10 passos) |
| `docs/arquitetura.md` | Camadas, regras e contratos entre elas |
| `docs/pipeline_motor.md` | Captura CKP/CMP, combustível, ignição, scheduling, ADC, comunicação |
| `docs/errata_stm32h562.md` | Errata ES0565 e impactos no projeto |
| `docs/qualidade_e_testes.md` | Gates locais, layout dos testes host, segredos |
| `docs/mvp_bancada.md` | Definição de pronto da bancada, frames CAN, ETB, X-τ |
| `docs/estado_e_hardening.md` | Estado atual, hardening feito e pinout por placa |
| `docs/fueling_system.md`, `closed_loop_fuel.md`, `idle_system.md`, `ignition_system.md`, `protection_system.md`, `output_contract.md` | Subsistemas |
