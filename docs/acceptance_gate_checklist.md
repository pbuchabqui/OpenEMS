# Checklist de Gate de Integração

Regra: se um módulo não cumprir todos os critérios da linha, não integra.

## 1) Execução host (obrigatório antes de hardware)

Comando:

```bash
bash scripts/run_host_tests.sh
```

Evidência mínima:
- saída com `All host tests passed.`

## 2) Matriz de aceite por módulo

Preencher `Status` com `PASS` ou `FAIL` e anexar evidência.

| Módulo | Critério obrigatório | Como verificar | Evidência esperada | Status |
|---|---|---|---|---|
| `hal/ftm` | Overflow FTM sem comportamento indefinido; PCR corretos para 16 pinos | Host: `test/hal/test_ftm_arithmetic.cpp` + continuidade com multímetro | Log host + foto/registro da continuidade dos 16 pinos |  |
| `drv/ckp` | Sync em <= 2 revoluções; falso gap rejeitado (`tooth_count < 55`); RPM correto | Host: `test/drv/test_ckp.cpp` (simulação de pulsos) | Log host com `failed=0` |  |
| `drv/scheduler` | Eventos em ordem; cancel limpo; evento passado rejeitado | Host: `test/drv/test_scheduler.cpp` + mock GPIO | Log host com `failed=0` |  |
| `drv/sensors` | Unidades de engenharia corretas; fault após 3 amostras fora de range | Host: `test/drv/test_sensors.cpp` | Tabela/print com valores esperados |  |
| `engine/fuel_calc` | PW em ±2% do cálculo manual; AE dispara no threshold | Host: `test/engine/test_fuel.cpp` + cálculo manual (5 condições) | Planilha/manual com 5 casos e erro percentual |  |
| `engine/ign_calc` | `dwell start > spark angle`; clamp `-10°/+55°` aplicado | Host: `test/engine/test_ign.cpp` | Log host com `failed=0` |  |
| `engine/aux` | IACV PID sem oscilação em step input | Host: `test/engine/test_iacv.cpp` + plot Python | Gráfico com resposta temporal e sem oscilação sustentada |  |
| `engine/knock` | Retardo após threshold; recovery correto | Host: `test/engine/test_knock.cpp` | Log host com `failed=0` |  |
| `app/tuner_studio` | Frames `r/w/A` corretos byte a byte vs referência Speeduino | Host: `test/app/test_ts_protocol.cpp` + captura serial real | Diff byte-a-byte contra captura real |  |
| `app/can_stack` | Frames `0x400/0x401` decodificáveis | Host: `test/app/test_can.cpp` + loopback HW com transceiver | Captura CAN no loopback e decode |  |
| `hal/flexnvm` | LTFT persiste após power cycle; sem corrupção após 1000 writes | Host: `test/hal/test_flexnvm.cpp` + teste de loop no hardware | Log de 1000 ciclos + verificação pós power cycle |  |
| `main.cpp` | Sync em < 3 rev no cranking; PIT watchdog kicking | Hardware com osciloscópio no pino de sync/watchdog | Captura de osciloscópio com tempo/revoluções |  |

## 3) Falhas históricas que bloqueiam integração

Se qualquer item abaixo estiver ausente, rejeitar integração:

- Falso gap por ISR atrasada (equivalente ao cenário RusEFI #1488).
- Offset angular não tratado (equivalente ao problema de ~3.9°).
- Overflow silencioso do scheduler.
- Conflito de pinos CAN + FTM (ex.: PTA12/PTA13).

## 4) Registro de decisão

- Data:
- Commit/branch avaliado:
- Responsável:
- Resultado final do gate: `INTEGRA` / `NÃO INTEGRA`
- Pendências abertas:
