# Ignition System Architecture

> **Status:** vigente · validado 2026-11-17 (doc-check + conferência pontual contra o código)

The ignition (spark) system determines when to charge (dwell) and fire (discharge) ignition coils based on engine position, RPM, and load.

## The Spark Pipeline

1.  **Timing Advance Calculation**: Core engine math in `algo/` determines the target advance (e.g., 20° BTDC) based on RPM and Load (MAP or TPS).
2.  **Dwell Calculation**: Determination of coil charging time (in milliseconds) based on battery voltage and RPM.
3.  **Angle Translation**: `engine_cycle/` translates the timing advance into a specific crank angle.
4.  **Event Generation**: Ignition events are created for each cylinder. An event consists of two actions: `START` (start dwelling) and `END` (fire spark).
5.  **Scheduling**: The `EventQueue` handles the execution of these actions at the precise crank angle or time.
6.  **Limp Management**: `LimpManager` can inhibit spark for RPM limiting, protection, or sync requirements.
7.  **Output Driver**: Coils are fired via digital outputs or specialized ignition drivers.

## Ignition Modes
- **Individual Coils (Sequential)**: One output per cylinder, fired every 720°.
- **Wasted Spark**: Outputs shared between companion cylinders, fired every 360°.
- **One Coil (Distributor)**: A single output fired for every cylinder.

## Key Files
- `firmware/controllers/engine_cycle/spark_logic.cpp`: Core scheduling and event generation.
- `firmware/controllers/algo/ignition_state.cpp`: Calculation of base timing and corrections (IAT, CLT, Knock).
- `firmware/controllers/engine_cycle/event_queue.cpp`: The high-precision scheduler.

## Implementation Notes
- **BTDC vs ATDC**: Advance is typically measured in degrees Before Top Dead Center (BTDC). Internal math may use 0-720° cycle coordinates.
- **Dwell Overlap**: rusEFI supports overlapping dwell events (where one coil starts charging before another finishes firing).
- **Wasted Spark on Odd Fire**: Specialized logic in `spark_logic.cpp` handles uneven firing intervals.

Use coilState1 coilState2 etc in logs (see unit_tests/test_results/readme.md for unit tests output) to
assert actual logic state.

## Anti-jerk (OpenEMS, `src/engine/ign_calc.cpp`)

Retardo de avanço no tip-in, somado em `engine_calc.cpp` à correção de driveline (vale o maior dos dois). Dispara quando `tpsdot` passa de `antijerk_tpsdot_threshold_x10` (%/s ×10; padrão 3 %/s) e usa magnitude proporcional a `tpsdot` até `antijerk_retard_deg` (retardo cheio a 100 %/s). O pico do tip-in é mantido durante a janela e o retardo cai em rampa linear até 0.

- **Janela:** byte 70 da página 0 (`antijerk_decay_x10ms`) em unidades de 10 ms (0 a 500 ms; padrão 20 = 200 ms). Antes do 2026-11-19 o byte era "ciclos" de passo de 2 ms, o que dava ~8 ms. A janela tem piso de 2 voltas do virabrequim, porque o avanço só chega ao scheduler uma vez por volta (tabela angular refeita na fronteira da volta). Páginas gravadas com o valor antigo (4) passam a valer 40 ms.
- **Rearme:** só depois que `tpsdot` cai abaixo do limiar (sem re-disparo contínuo com o pedal ainda subindo).
- **Reset:** em partida e pós-partida o estado é zerado.
- Sem efeito na bancada atual (sem combustão); limiar, retardo e janela pedem carro e datalog.
