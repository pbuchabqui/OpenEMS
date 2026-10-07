# Manual de teste de bancada — WeAct H562 VGT6

Este manual testa a **lógica do firmware no silício real**: sincronismo, ângulo
da faísca, largura do pulso de injeção, jitter e watchdog. A placa é a WeAct
STM32H562 VGT6 (`docs/hw/weact_h562_coreboard.md`), e o motor é simulado por um
ESP32 com `tools/esp32_combined`.

**O que este teste NÃO prova:** que a ECU dirige bobinas e injetores, que a
fonte e a proteção funcionam, que os sensores reais leem certo. A placa de ECU
ainda não existe (`hardware/openems_v1` só tem o esquemático, incompleto). Por
isso, **não ligue nada de potência** a esta placa: sem bobina, sem injetor, sem
motor. Os pinos de saída vão direto ao ESP32 ou ao osciloscópio, em 3,3 V.

---

## 1. Material

| Item | Uso |
|---|---|
| WeAct STM32H562 **VGT6** (LQFP100) | ECU sob teste |
| ESP32 DevKit (ESP32-WROOM, com DAC) | gerador CKP/CMP 60-2, MAP/TPS, scope lógico |
| Osciloscópio ou analisador lógico, ≥ 4 canais, ≥ 1 MHz | medir ângulo e largura de pulso (T3, T4) |
| Cabo USB-C (DFU e USB CDC) | gravar e comunicar |
| Adaptador USB-serial 3,3 V (opcional) | UART PA9/PA10, se o USB CDC não enumerar |
| Jumpers fêmea-fêmea, GND comum | ligações |
| PC com `arm-none-eabi-gcc`, `dfu-util`, Python ≥ 3.10 | build, gravação, dashboard |

### 1.1 Revisão do chip (errata ES0565)

Antes de tudo, anote a revisão do silício. O README (§ Errata STM32H562 ES0565)
considera a **Rev X**. Na Rev A, PA1 (CMP) só tem histerese quando PA0 está em
entrada, e a flash tem endurance de 1 kciclo.

1. Leia a marcação do chip (a letra de revisão fica na última linha).
2. Conecte em DFU (BOOT0 pressionado + reset) e abra o STM32CubeProgrammer: ele
   mostra `Revision` (campo `REV_ID` de `DBGMCU_IDCODE`).
3. Registre as duas leituras no relatório do teste. Se for Rev A ou Z, pare e
   avise antes de seguir.

---

## 2. Compilar e gravar

```bash
make firmware-vgt6
# saída: /tmp/openems-build/bin/openems-vgt6.bin
```

Gravação por DFU (bootloader de ROM):

1. Segure **BOOT0**, aperte e solte **NRST**, solte BOOT0.
2. `dfu-util -l` deve listar `0483:df11`.
3. Grave:

   ```bash
   dfu-util -a 0 -s 0x08000000 -D /tmp/openems-build/bin/openems-vgt6.bin
   ```

4. Desligue e ligue a placa com BOOT0 solto (BOOT0 = 0).

A calibração fica em setores próprios da flash e sobrevive à gravação do
firmware. Depois de gravar uma versão nova, use **Restaurar padrões de fábrica**
no dashboard (§ 5) para não testar com dados antigos.

---

## 3. Ligações

Pinos do mapa `BOARD=vgt6` (`src/hal/out_pins.h`, `docs/hw/pinout.md`).
Todas as saídas são GPIO ativas em nível alto, acionadas pelo compare do
TIM5_CH3. Repouso = LOW.

| STM32 (WeAct) | Sinal | ESP32 | Observação |
|---|---|---|---|
| PA0 | CKP (TIM5_CH1) | GPIO 2 → | roda 60-2 gerada por RMT |
| — | — | GPIO 2 → GPIO 34 | jumper no próprio ESP32: loopback do CKP para o scope |
| PA1 | CMP (TIM5_CH2) | GPIO 4 → | 1 pulso a cada 720°, no dente 5 |
| PA3 | MAP (ADC) | GPIO 26 → (DAC2) | **obrigatório**: o modo bancada não simula MAP |
| PA4 | TPS (ADC) | GPIO 25 → (DAC1) | **obrigatório**: o modo bancada não simula TPS |
| PE9 | IGN1 | ← GPIO 32 | scope |
| PE11 | IGN2 | ← GPIO 33 | scope |
| PE13 | IGN3 | — | sem entrada livre no ESP32: use o osciloscópio |
| PE15 | IGN4 | — | sem entrada livre no ESP32: use o osciloscópio |
| PE0 | INJ1 | ← GPIO 27 | scope |
| PE2 | INJ2 | ← GPIO 14 | scope |
| PE4 | INJ3 | ← GPIO 12 | scope |
| PE6 | INJ4 | ← GPIO 13 | scope |
| PA9 / PA10 | USART1 TX / RX | — | opcional: adaptador USB-serial, 115200 8N1 |
| USB-C | USB CDC | — | transporte principal do dashboard |
| GND | GND | GND | **obrigatório**, comum a tudo |

Confira a pinagem do ESP32 no cabeçalho de
`tools/esp32_combined/esp32_combined.ino` antes de ligar.

**O que é esperado nesta placa:**

- O LED de heartbeat do firmware é PB2. O LED da WeAct é PC13, então ele **não
  pisca**. Use o dashboard para saber se o firmware está vivo.
- O USB CDC tem driver completo (`src/hal/stm32h562/usb_cdc.cpp`), mas ainda
  não foi validado neste hardware. Se não enumerar (`/dev/ttyACM*` não aparece),
  use a UART PA9/PA10 e anote o problema.

---

## 4. Gerador ESP32 (`tools/esp32_combined`)

1. Copie `wifi_credentials.example.h` para `wifi_credentials.h` (o arquivo é
   ignorado pelo git) e preencha a rede.
2. Compile e grave com PlatformIO (`tools/esp32_combined/platformio.ini`) ou com
   a Arduino IDE.
3. Abra o monitor serial (115200). Comandos principais:

| Comando | Efeito |
|---|---|
| `RPM <n>` | RPM exato (50 a 9000), com rampa ≤ 2 % a cada 50 ms |
| `+` / `-` | RPM ± 100 |
| `0`…`9` | presets 100/200/300/500/700/1000/1500/2000/3000/5000 RPM |
| `IDLE` / `CRANK` / `CRUISE` / `WOT` / `COAST` | presets de RPM + sensores (IDLE = 700, CRANK = 200) |
| `MAP <kPa>` / `TPS <%>` | tensão nos DACs de PA3 / PA4 |
| `S` / `STATUS` | estado do gerador |
| `l` `e` `p` `w` `t` `s` `r` | modos do scope (live, edges, pulsos, waveform, timing, estatística, reset) |

Forma de onda: cada dente é um pulso **alto** de meio período; o gap é o último
dente com o nível baixo estendido em 2 períodos (razão 3). O CMP sobe no dente
5 da primeira volta, 1 µs depois do CKP, para nunca coincidir com ele.
Portanto, a borda útil dos dois é a **subida**.

---

## 5. Dashboard e configuração

```bash
cd tools/openems_dash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python server.py --port /dev/ttyACM0 --http-port 8000   # ou /dev/ttyUSB0 (UART)
```

Abra `http://localhost:8000`. Na aba **Install**:

1. **Restaurar padrões de fábrica** (com o gerador parado). O tune de fábrica
   (`tools/openems_dash/base_tune.json`) tem offset do gatilho 0, polaridade de
   captura 0 (subida em CKP e CMP) e avanço da lâmpada 10,0°.
2. Passo 2 (CKP/CMP): **deixe as duas caixas de "borda de descida" desmarcadas**.
   O gerador ESP32 é push-pull com dente alto, diferente do Hall open-collector
   do motor real (que usa descida). Polaridade em page0[258]
   (`src/engine/calibration.h:252`).
3. Passo 5 (offset do gatilho): mantenha **0**. Com offset 0, o dente 0 é o PMS
   do cilindro 1, o que torna os ângulos do § 6 fáceis de conferir.
4. Ligue **BENCH** (canto superior). O modo bancada (comando `B`, só RAM, cai no
   reset) simula CLT/IAT/VBATT/óleo/combustível e limpa as falhas desses
   sensores. MAP e TPS continuam reais (PA3/PA4), por isso precisam do ESP32.
   O status bit 15 (`BENCH_MODE`) confirma.

Com o dashboard fechado, o mesmo link serial serve ao script de bancada:

```bash
python3 tools/diag/bench_check.py /dev/ttyACM0 --period 0.5
```

Ele imprime uma linha por leitura (RPM, sincronismo, modo de injeção, confirmações
de CMP, avanço, PW, período do dente, pior loop de 2 ms e os contadores
late/drop/clamp). Termina com `OK` ou `FAIL`, e sai com código 1 se o
`FULL_SYNC` cair depois de alcançado ou se algum contador late/drop/clamp mudar.
O dashboard e o script não podem abrir a porta ao mesmo tempo.

---

## 6. Testes

Registre para cada teste: data, commit do firmware, revisão do chip, resultado
e a captura do osciloscópio quando houver.

### T0 — Boot seguro

1. Osciloscópio em IGN1 e INJ1, gatilho em qualquer borda, modo single.
2. Desligue e ligue a placa 10 vezes, com o gerador **ligado** a 3000 RPM.

**Passa:** nenhuma borda nas saídas durante o boot. As saídas são forçadas a LOW
logo depois do clock (`ecu_sched_outputs_safe_early()`). Sem loop de reset: o
dashboard reconecta e o RPM aparece em poucos segundos.
**Falha:** qualquer pulso no boot, ou a placa reiniciando sozinha. O IWDG tem
~10 s no boot e ~0,8 s em funcionamento (`src/hal/stm32h562/system.cpp:211`,
`src/main_stm32.cpp:385`). Um reset periódico nesses intervalos indica laço
travado.

### T1 — Comunicação

1. Gerador parado. Rode `bench_check.py --count 20`.

**Passa:** 20 linhas, `OK`, sem exceção de CRC ou tamanho. O snapshot tem
**86 bytes** (tabela no § 7).

### T2 — Sincronismo e RPM

1. `RPM 200`, aguarde. Depois suba: 500, 700, 1000, 2000, 3000, 5000, 7000, 8500.
2. Em cada degrau, deixe `bench_check.py` rodando por 30 s.
3. Volte de 8500 até 200 com o comando `-` repetido.

**Passa:**
- O sincronismo vai a `FULL` e lá fica nas subidas e descidas.
- O RPM reportado bate com o do gerador (±1 %), o que confirma o clock de
  62,5 MHz do TIM5.
- O período do dente (`tooth`) vale 1 s ÷ (RPM × 60) — por exemplo,
  1666,7 µs a 600 RPM.
- `late +0 drop +0 clamp +0`, e o script termina com `OK`.

Critérios do decoder (`src/drv/ckp.cpp`): faixas de razão 0,5 e 1,5, e
exatamente 57 dentes entre gaps.

### T3 — Ângulo da faísca (critério principal)

1. Na aba Install, passo 6: **Ligar modo lâmpada** com 10,0°. A faísca fica fixa
   em 10,0° APMS, sem correções nem knock (`src/engine/ign_calc.cpp:76`).
2. Gerador a 1000, 3000 e 6000 RPM, com sincronismo `FULL` e `seq`.
3. Osciloscópio: CH1 = CKP (PA0), CH2 = CMP (PA1), CH3 = IGN1 (PE9),
   CH4 = IGN3 (PE13). Repita com IGN2 (PE11) e IGN4 (PE15).

A faísca é a **borda de descida** da saída IGN (fim do dwell). Com offset 0, a
faísca de cada cilindro cai em `PMS − avanço` (`src/engine/ecu_sched_angle.cpp:113`).
A ordem de ignição é 1-3-4-2, com PMS em 0°/180°/360°/540°
(`src/engine/engine_config.h:51`). Logo:

| Saída | PMS | Faísca a 10° | Onde medir |
|---|---|---|---|
| IGN1 | 0° | 710° | 10° **antes** da subida do dente 0 que fecha a volta com CMP |
| IGN3 | 180° | 170° | 170° **depois** do dente 0 da volta sem CMP |
| IGN4 | 360° | 350° | 10° **antes** da subida do dente 0 que abre a volta com CMP |
| IGN2 | 540° | 530° | 170° **depois** do dente 0 da volta com CMP |

O dente 0 é o primeiro dente depois do gap. A volta que vem depois do pulso de
CMP é a fase A (`src/drv/ckp.cpp:348`), e a fase A começa no ângulo 0. Se
IGN1 e IGN4 aparecerem trocados, anote: é exatamente o tipo de erro que esta
bancada precisa pegar.

Conversão: 1° = T ÷ 6, onde T é o período de um dente. A 3000 RPM, T = 333,3 µs,
então 10° = 555,6 µs e 0,1° = 5,6 µs.

**Passa:** erro ≤ **0,1°** em todos os RPMs, nas quatro saídas.
**Também confira:** o dwell (tempo em HIGH antes da faísca) é estável de ciclo a
ciclo.

Desligue o modo lâmpada ao terminar (ele também cai no reset).

### T4 — Largura do pulso de injeção

**Parte A — pulso de teste (gerador parado, RPM = 0):**
1. Aba **Outputs**: arme o teste (o dashboard manda o keepalive enquanto estiver
   armado). Dispare INJ1 com 1000 µs, 5000 µs e 20000 µs (o máximo é
   30000 µs, `src/engine/output_test.cpp:98`).
2. Meça a largura do pulso em PE0. Repita em INJ2–4.

**Passa:** largura = valor pedido, ±2 µs.

**Parte B — em funcionamento:**
1. Gerador a 3000 RPM, `MAP 60`, sincronismo `FULL` e `seq`.
2. Compare a largura em PE0 com o `pw` do dashboard ou do script (resolução
   0,1 ms).

**Passa:** uma abertura por injetor a cada 720°, uma a cada cilindro, na ordem
1-3-4-2, com largura igual ao `pw` reportado (±0,1 ms).

### T5 — CMP: sequencial e faísca perdida

1. Gerador a 2000 RPM, `FULL` + `seq`, `cmp 2`.
2. Desconecte o fio do CMP (PA1).
3. Depois de ~2 s, reconecte.

**Passa:**
- Sem CMP, o firmware volta para faísca perdida (status bit 11 = 0) e injeção
  semissequencial após 60 voltas no modo bancada. Fora da bancada, são 6
  (`src/drv/ckp.cpp:113`).
- O RPM e o `FULL` continuam.
- Com o CMP de volta, após 2 confirmações (`cmp 2`), o modo volta a `seq`.

### T6 — Perda de CKP

1. Gerador a 3000 RPM, `FULL`.
2. Desconecte o fio do CKP (PA0) com o osciloscópio olhando IGN1 e INJ1.
3. Reconecte.

**Passa:**
- Todas as saídas vão a LOW em poucos ms e ficam em LOW. Nenhuma bobina fica
  presa em dwell.
- O RPM cai para 0.
- Ao reconectar, o firmware passa por `WAIT_GAP` e volta a `FULL` sem reset.

### T7 — Carga e orçamento do loop

1. Gerador a 8500 RPM, dashboard aberto com a aba Telemetry atualizando.
2. Deixe rodar 10 minutos.

**Passa:**
- `loop max` (pior loop de 2 ms) fica bem abaixo de 2000 µs.
- Nenhum contador late/drop/clamp anda.
- Sem reset.

### T8 — Teste de saídas

1. Gerador parado (RPM = 0). O teste de saídas fica bloqueado com o motor
   girando.
2. Na aba Outputs, arme o teste e dispare cada injetor e cada bobina, um de cada vez.

**Passa:**
- Pulso apenas no pino certo (tabela do § 3).
- Dwell da bobina limitado a 10000 µs.
- Sem keepalive por 5 s, o teste aborta e restaura as saídas em LOW
  (`src/engine/output_test.cpp:12`).

### T9 — Reset com o motor girando

1. Gerador a 3000 RPM, `FULL`.
2. Aperte NRST 10 vezes.

**Passa:**
- Nenhum pulso fora de hora nas saídas durante e logo depois do reset.
- O sincronismo volta a cada vez.
- O modo bancada cai (é só RAM), e o dashboard reflete isso pelo status bit 15.

---

## 7. Snapshot de tempo real (86 bytes)

Lido pelo comando `r` da página de tempo real (`OpenEMSLink.read_realtime()`). Os campos de 14 a
65 são `reserved[52]`; o offset do byte é 14 + o índice. Fonte:
`src/app/ui_protocol_pages.cpp` e `tools/openems_dash/protocol.py:116`.

| Byte | Campo | Unidade |
|---|---|---|
| 0–1 | RPM | u16 |
| 2 | MAP | kPa |
| 3 | TPS | % |
| 4 | CLT | °C + 40 |
| 5 | IAT | °C + 40 |
| 6 | lambda | ×1000 ÷ 5 |
| 7 | PW | 0,1 ms |
| 8 | avanço | ° + 40 |
| 9 | VE[0][0] (estático) | — |
| 10 | STFT | % s8 |
| 12–13 | status bits | u16 |
| 14–17 | eventos atrasados (late) | u32 |
| 18 | alvo de lambda | ×1000 ÷ 5 |
| 19 | LTFT | % s8 |
| 20 | glitches de CMP | u8 |
| 21 | confirmações de CMP (0–2) | u8 |
| 22 | reservado | sempre 0 |
| 23 | etanol | % |
| 24–27 | drops do agendador | u32 |
| 28–31 | clamps de calibração | u32 |
| 32 | máscara de rejeição da config | u8 |
| 33 | modo lâmpada ativo | bit 0 |
| 34–35 | avanço enviado às bobinas | 0,1° s16 |
| 36 | reservado | sempre 0 |
| 44 | sync (nibble baixo) / modo de injeção (nibble alto) | 0 WAIT_GAP, 1 HALF, 2 FULL, 3 LOSS / 0 simult., 1 semi, 2 seq |
| 45–46 | redução do controle de tração | 0,1 % |
| 47 | retardo de torque | ° |
| 48 | falhas de sensores | bitmask |
| 49–52 | loop de 2 ms, último | µs u32 |
| 53–56 | loop de 2 ms, máximo | µs u32 |
| 57–62, 64–65 | AN1–AN4 brutos | u16 |
| 63 | **VE ao vivo** | — |
| 66–67 | MAP fundido | kPa ×1 (bar×100) |
| 68–69 | PW líquido | µs |
| 70–73 | bordas CKP | u32 |
| 74–77 | bordas CMP | u32 |
| 78–81 | período do dente | ns u32 |
| 82–83 | idade da última borda CKP | ms |
| 84–85 | idade da última borda CMP | ms |

**Status bits** (`src/app/status_bits.h`):

| Bit | Nome | Bit | Nome |
|---|---|---|---|
| 0 | FULL_SYNC | 8 | SCHED_CLAMP |
| 1 | PHASE_A | 9 | WBO2_FAULT |
| 2 | SENSOR_FAULT | 10 | reservado (sempre 0) |
| 3 | LIMP_MODE | 11 | IGN_SEQUENTIAL (0 = faísca perdida) |
| 4 | ETB_LIMP | 12 | REV_LIMIT |
| 5 | XTAU_LEARN | 13 | LAUNCH_ACTIVE |
| 6 | SCHED_LATE | 14 | TC_ACTIVE |
| 7 | SCHED_DROP | 15 | BENCH_MODE |

---

## 8. Problemas comuns

| Sintoma | Causa provável |
|---|---|
| `/dev/ttyACM*` não aparece | USB CDC não enumerou: use a UART PA9/PA10 (`--port /dev/ttyUSB0`) e anote |
| `dfu-util` não acha a placa | BOOT0 não estava pressionado no reset; cabo só de carga |
| RPM 0 com o gerador ligado | GND não comum; fio em PA0 errado; polaridade "descida" marcada |
| Sincronismo oscila entre `WAIT_GAP` e `FULL` | gerador abaixo de 200 RPM; ruído no fio do CKP (encurte, torça com GND) |
| Fica em faísca perdida com CMP ligado | `cmp` não chega a 2: CMP em PA1 solto, ou polaridade CMP "descida" marcada |
| `SENSOR_FAULT` aceso | modo bancada desligado, ou MAP/TPS (PA3/PA4) sem os DACs do ESP32 |
| Teste de saídas recusado | RPM > 0: desligue o fio do CKP |
| Ângulo errado por um múltiplo de 6° | offset do gatilho ≠ 0; confira na aba Install |
| Ângulo errado por 360° (IGN1 ↔ IGN4) | fase invertida: relate com a captura CKP + CMP + IGN1 |
| `late`/`drop` sobem em RPM alto | relate com RPM e `loop max`: é bug do agendador |

---

## 9. Antes do primeiro motor

Esta bancada é a primeira de várias etapas. Antes de girar um motor:

- [ ] T0–T9 aprovados nesta placa, com as capturas arquivadas.
- [ ] Revisão do chip registrada (§ 1.1).
- [ ] **Placa de ECU projetada, fabricada e testada**: fonte, drivers de
      injetor e bobina, condicionamento de CKP/CMP, proteção. **Ainda não
      existe.**
- [ ] Repetir T0, T3, T4 e T6 na placa de ECU, com carga resistiva no lugar
      das bobinas e dos injetores.
- [ ] Os 10 passos de instalação do README feitos no motor: polaridade
      descendente para Hall open-collector, offset medido, dead time do
      injetor e confirmação com a lâmpada de ponto.
