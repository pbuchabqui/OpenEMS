# Qualidade, gates e testes

## Higiene De Codigo

Programa de qualidade do monorepo (concluido 2026-07; HEAD tipico inclui
`fa36aab` e anteriores `713d4a8`…`bce2f4a`). Objectivo: reviews limpos, zero
warnings com `WERROR`, camadas enforcadas e caminho critico de actuadores
isolado sem big-bang rewrite.

### Gates locais (obrigatorios antes de merge)

```bash
make ci-local                   # todos os gates abaixo
# equivalentes manuais:
make secrets-check
make host-test WERROR=1         # referencia: 1341 PASS / 0 FAIL
make precision-test             # motor virtual: 120 PASS / 0 FAIL (centelha, dwell, PW, EOI)
make ini-check                  # TunerStudio ini: referencias, sobreposicao, faixas = firmware
make dash-check DASH_PY=<venv>/bin/python  # dashboard: helpers JS + protocolo vs ECU simulada
make sim-ecu NVM=/tmp/nvm.bin   # ECU simulada num pty para o dashboard (sem placa)
make firmware-rgt6 WERROR=1
make firmware-vgt6 WERROR=1
make lint-includes LINT_PHASE=A LINT_ERROR=1   # ban ENGINE/DRV → app/
make lint-includes LINT_PHASE=B LINT_ERROR=1   # ban ENGINE → hal/regs.h (allowlist)
make format                     # clang-format so em ficheiros dirty (on-touch)
```

| Gate | Target | Notas |
|------|--------|--------|
| Warnings | 0 com `-Wall -Wextra` | Host **e** firmware dual board |
| Secrets tracked | 0 | `**/wifi_credentials.h` gitignored |
| Layering A | 0 includes `app/` em `src/engine`, `src/drv` | `tools/lint_includes.py` |
| Layering B | `hal/regs.h` so em engine com allowlist | `tools/include_allowlist.txt`: `ecu_sched` (TIM5), `auxiliaries` (fan/pump ate Phase C) |
| Host tests | PASS exacto estavel | Suites em `test/test_*.cpp` + `run_all.cpp` |
| Dual board | bins rgt6 + vgt6 | Qualquer mudanca de pin/HAL |

### Layout de testes host

| Ficheiro | Papel |
|----------|--------|
| `test/harness.{h,cpp}` | `CHECK_*`, contadores PASS/FAIL |
| `test/fixtures.{h,cpp}` | setup ADC/ETB, `ckp_fire` / `cam_fire` |
| `test/ui_helpers.{h,cpp}` | envelope TunerStudio para testes de protocolo |
| `test/run_all.cpp` | `main()` — **ordem fixa** das suites |
| `test/suite_registry.h` | declaracoes das funcoes `test_*` |
| `test/test_*.cpp` | suites (etb, torque, ckp, fuel, sched, protocol, …) |
| `test/sim/engine_sim.{h,cpp}` | motor virtual: cinematica θ(t), roda 60-2 real + came, jitter/ruido, modelo de latencia da ISR, log de pinos com timestamp |
| `test/sim/precision_main.cpp` | `make precision-test`: compara o que o motor recebe (angulo de centelha, dwell, combustivel, EOI) com o oraculo cinematico; defeitos conhecidos = XFAIL com a etapa que os corrige |

Host: um unico TIM5 simulado (`hal/host/tim5_host.h`) e partilhado por captura
CKP/CMP, comparador do scheduler e `tim5_count()`; `out_pin_write` expoe um
hook (`out_pins_host::write_hook`) para o simulador registar cada transicao.

Nao reintroduzir monolitio `mvp_bench_tests.cpp`. Novos testes: ficheiro de suite
existente ou novo `test_*.cpp` + registo em `run_all.cpp` e `suite_registry.h`.

### Secrets e vendor

- WiFi tools ESP32: copiar `wifi_credentials.example.h` → `wifi_credentials.h`
  (nunca commitado). Se password ja esteve no historico git, **rodar no AP**.
- `graphify-out/` e bins/fonts pesados do `ardustim_gui` nao sao versionados
  (regenerar graphify / `npm install` localmente). GUI Electron continua
  documentada em `tools/esp32_combined/README.md`.

### Fora de escopo do programa de higiene (opcional)

- `hal/aux_gpio` para fan/pump (hoje `auxiliaries.cpp` ainda usa `regs.h` /
  BSRR — allowlist Phase C).
- Split adicional de monolitios grandes (`ckp.cpp`, `fuel_trim`, etc.).

