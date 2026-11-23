# Manual de teste de bancada — WeAct H562 VGT6

> **Status:** vigente · validado 2026-11-22 (doc-check + conferência pontual contra o código)

Este manual testa a **lógica do firmware no silício real**: sincronismo, ângulo
da faísca, largura do pulso de injeção, jitter e watchdog. A placa é a WeAct
STM32H562 VGT6 (`docs/hw/weact_h562_coreboard.md`), e o motor é simulado por um
ESP32 com `tools/esp32_combined`.

**O que este teste NÃO prova:** que a ECU dirige bobinas e injetores, que a
fonte e a proteção funcionam, que os sensores reais leem certo. A placa de ECU
ainda não existe (o projeto KiCad `openems_v1` foi removido do repositório em 2026-10-21, `a1e26fc`). Por
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

### 1.0 Errata ES0565 §2.2.33 — medir a primeira gravação (opcional, 2 min)

O firmware faz uma gravação sacrificial no boot e mede a 1ª e a 2ª. Para ler o resultado:

1. Desligue e ligue a alimentação da placa (power-on de verdade; um reset pelo botão não conta).
2. Pela serial USB (`/dev/ttyACM0`), envie o byte `j` e leia 14 bytes: `ran, first_kind, status, reserved`, `first_us` (u32 LE), `second_us` (u32 LE), `slot` (u16 LE).
3. Esperado: `ran = 1`, `status = 0`. Se a errata se manifesta neste chip, `first_us` fica cerca de 120 µs acima de `second_us`. Registre os dois valores no relatório.
4. Depois de um reset que não é power-on, `ran = 0` é o comportamento correto.

### 1.1 Revisão do chip (errata ES0565)

Antes de tudo, anote a revisão do silício. O `docs/errata_stm32h562.md` (errata ES0565)
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
| PC0 | APP1 (ADC) | GPIO 23 → (PWM + RC) | só para os testes de ETB/taper; APP2 (PC2) sem saída própria |
| PA2 / PC5 | ETB TPS1 / TPS2 | GPIO 16 / 19 → (PWM + RC) | só para os testes de ETB/taper; TPS2 é o complemento do TPS1 |
| PB0 / PB1 | CLT / IAT | GPIO 21 / 22 → (PWM + RC) | o modo bancada já simula CLT/IAT; só ligue sem ele |
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

> **Diagnóstico do ADC:** o comando serial `'m'` devolve o ADC cru dos 8 canais primários (8 x u16) e o `'n'`
> os registradores `GPIOA_MODER`, `ADC1_DIFSEL`, `ADC1_SQR1/2`, `ADC1_SMPR1/2`, `ADC1_CFGR1` e `ADC1_ISR` (8 x u32).
> O `tps_pct` do dashboard é o **ETB** (média de ETB_TPS1/PA2 e ETB_TPS2/PC5), não o TPS legado do PA4: com PA2
> e PC5 flutuando ele fica em ~50% e oscila. Para vê-lo mudar, ligue o ETB do estimulador (PB7→PA2, PB8→PC5, cada um com RC).


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
| `SLEW <n>` | passo máximo da rampa de RPM, % por 50 ms (1 a 30; padrão 2) |
| `PROFILE TAPER` / `PROFILE STOP` | CRANK 200 rpm 3 s, 1700 rpm 1 s, 850 rpm: teste do taper do ETB |
| `S` / `STATUS` | estado do gerador |
| `l` `e` `p` `w` `t` `s` `r` | modos do scope (live, edges, pulsos, waveform, timing, estatística, reset) |

O gerador **não** emula WBO2 (CAN) nem velocidade de roda: a malha de lambda e o TC não
são testáveis só com ele.

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

### T7b — Latência da ISR do CKP e ICACHE (`tools/isr_diag.py`)

Mede quanto a ISR do TIM5 leva da captura do dente até o fim do tratamento (inclui o despacho de INJ/IGN que
roda antes dele na mesma IRQ) e se o ICACHE está acertando. Também separa o custo do gap em rebuild da tabela
angular, laço que arma os eventos do dente 0 e o resto. Comandos seriais de 1 byte: `'k'` lê (41 bytes),
`'l'` zera só os máximos; nenhum mexe em saída ou calibração.

1. Gerador no RPM desejado (800, 3000, 6000, 8500), sync cheio (`syncFull`), motor "em regime" há alguns segundos.
2. `python3 -I tools/isr_diag.py --reset`, espere ≥ 30 s, depois `python3 -I tools/isr_diag.py`. Os máximos de
   latência não saturam, mas os monitores do ICACHE **sim** (hit em 2³²−1, miss em 65535, sem dar a volta, RM0481
   §8.4.12; o de hit enche em ~17 s a 250 MHz). Para a taxa de acerto, rode `--reset` e leia em poucos segundos;
   se o script escrever "n/d (monitor saturou)", ignore a taxa dessa leitura.
3. Anote `dente normal max`, `dente do gap max` e a taxa de acerto do ICACHE. Compare com o período do dente
   (`1e6 / (rpm/60 × 60)` µs ≈ 117 µs a 8500 RPM, 1000 µs a 1000 RPM).
4. Repita com um firmware compilado com `-DEMS_ICACHE=0` (o `isr_diag.py` mostra "DESLIGADO"): a diferença do máximo
   é o ganho do ICACHE. O pior caso é o gap (rebuild da tabela angular dentro da ISR): se ele for uma fração
   relevante do período do dente a RPM alto, é o argumento para tirar o rebuild da ISR.

**Medido em 2026-11-19** (ECU VGT6 Rev X, estimulador `tools/stm32_bench`, sync cheio, injeção sequencial, `late`/`drops`/`clamps` = 0;
**uma** janela de 20 s por ponto e uma configuração por vez: o máximo depende de eventos raros, então os percentuais têm margem):

| rpm | Dente normal máx, ICACHE ligado → desligado | Gap máx, ligado → desligado | Gap em % do período do dente |
|---|---|---|---|
| 800 | 9,8 → 12,7 µs | 18,6 → 31,5 µs | 1,5 → 2,5 |
| 3000 | 9,4 → 13,0 µs | 19,7 → 30,6 µs | 5,9 → 9,2 |
| 6000 | 8,8 → 11,0 µs | 21,5 → 33,6 µs | 12,9 → 20,2 |
| 8500 | 9,8 → 14,5 µs | 21,4 → 34,9 µs | 18,2 → 29,6 |

O ICACHE reduziu o pior caso em ~20 a 41 % nos quatro pontos. Mesmo com ele o gap chega a 18 % do período do dente a 8500 rpm
(~1,1° de virabrequim), porque o rebuild da tabela angular roda dentro da ISR: é o argumento para tirá-lo de lá.
**Onde o tempo do gap é gasto** (mesma bancada, ICACHE ligado, `'k'` de 41 bytes): o rebuild da tabela angular custava
**12,1 µs, constante em todas as rpm** (58 a 66 % do gap; máximo igual ao último valor, ou seja, limitado por código e não por dados);
o laço que arma os eventos do dente 0 vale 0 µs até 3000 rpm e 1,9 µs a partir de 6000 (entradas "now"); o resto
(`process_gap_event`, sensores, entrada na ISR) fica em 4,5 a 6,2 µs. Com o `cycle` do builder como parâmetro de template e a
divisão de `ticks_to_x10` em 32 bits, o rebuild caiu para **9,0 µs** e o gap a 8500 rpm de 20,9 para 17,5 µs (17,8 → 14,9 % do
dente), com tabelas byte a byte iguais (`test_sched_angle_equivalence`). Adiar o rebuild para fora da ISR foi descartado: as
entradas "now" do gap se perderiam em aceleração, e o flag de modo (`g_knock_sequential`) se separaria da tabela em uso.

Isto é o tempo da ISR (captura → fim do tratamento); **não** mede o jitter dos pinos INJ/IGN, que exige analisador lógico.
Os monitores de hit/miss do ICACHE saturam em segundos (miss em 65535), então a taxa de acerto não foi usada; o ganho vem do A/B acima.

Unidade: ticks do TIM5 a 62,5 MHz (16 ns). O comando `'l'` zera os máximos e os dois monitores. Não há limite de aprovação
fixado ainda: o objetivo é registrar os números por RPM. Um burn de calibração desliga o ICACHE durante a gravação
e o religa no fim (`docs/errata_stm32h562.md`); o último byte do `k` (`icache_on`) deve voltar a 1 depois do burn.
Se ficar em 0, o religar falhou: pare e investigue antes de seguir.

### T7c — Tempo mascarado pelas seções críticas (`tools/isr_diag.py --cs`)

Mede quanto tempo cada `CriticalSectionGuard` mantém as interrupções mascaradas (PRIMASK). Enquanto ele roda, um
evento INJ/IGN que vence no TIM5 espera: é atraso de despacho que o T7b não enxerga, porque o T7b parte da captura do dente.
O firmware normal não carrega a instrumentação; compile uma imagem só para a medida:

1. `make firmware-vgt6 EXTRA_CFLAGS=-DEMS_CS_PROF=1`, grave, e guarde o ELF (`/tmp/openems-build/elf/openems-vgt6.elf`).
2. Gerador no RPM desejado (800, 3000, 6000, 8500), sync cheio, com o dashboard conectado (o tráfego do dashboard
   é o que exercita as seções do laço principal), alguns segundos em regime.
3. `python3 -I tools/isr_diag.py --reset`, espere ≥ 30 s, depois `python3 -I tools/isr_diag.py --cs --elf <ELF>`.
4. Anote, por balde, o máximo em µs, o `n` e a função do `pc`. Balde 0 = laço principal; balde 1 = SysTick e demais
   exceções. As seções dentro da ISR do TIM5 e as aninhadas não entram (o guard mais externo já as cobre).
   `critical_enter()`/`critical_exit()` (não RAII) não são medidos.
5. Compare com o dente normal do T7b (~10 µs) e com o gap (~15–21 µs): um máximo na mesma ordem compete com o rebuild do
   gap como causa de atraso de despacho.

O `pc` é o endereço da construção do guard (instrução `mov r, pc` dentro do inline), então o addr2line aponta a linha. Não
há limite de aprovação fixado: o objetivo é registrar os números. Com `EMS_CS_PROF=0` (padrão) o guard é idêntico ao
anterior. Com ele ligado, cada guard custa uns poucos ciclos a mais (duas leituras do DWT e um `mrs ipsr`), então os
máximos do T7b com essa imagem não valem para o firmware normal. Medida em bancada: ainda não feita.

### T7d — Jitter e erro angular nos pinos IGN (analisador lógico)

Mede o que T7b e T7c não medem: o atraso e a dispersão **nos pinos**, entre a borda do CKP que chega ao PA0 e a
borda de descida (faísca) da saída IGN. É a medida que separa as três causas candidatas de erro: colisão com a ISR do
dente (principalmente o rebuild do gap), seção crítica do laço principal (T7c) e o próprio gerador. Bancada sem bobinas
nem injetores (`CLAUDE.md`): as saídas são só GPIO, nada energizado.

**Material.** Duas opções, a mesma análise:

- **Estimulador `tools/stm32_bench` (já na bancada).** O scope dele carimba CKP/CMP com o instante exato do compare do TIM5
  e IGN/INJ por EXTI, tudo na mesma base de 1 MHz (1 µs de resolução: ±0,5 µs = 0,025° a 8500 rpm). Mede IGN1–4 e INJ1–4.
  Ligações da seção 3 (PE9/11/13/15 → PC0..PC3 do gerador). O jitter do EXTI não está medido: faça a calibração abaixo antes.
- **Analisador lógico externo** de 8 canais, ≥ 50 MS/s, captura contínua em disco, limiar 1,65 V, GND curto comum. Canais:
  CH0 = CKP (PA0), CH1–CH4 = IGN1–IGN4 (PE9, PE11, PE13, PE15). Exporte as bordas no mesmo formato de linha do scope
  (`<t µs> µs  CH<n> <NOME> RISE|FALL`) para usar o mesmo script. Use esta opção se a calibração abaixo der jitter > 3 µs.

Pinos do `BOARD=vgt6` (`src/hal/out_pins.h`, seção 3).

**Calibração do scope do estimulador (antes da corrida A).** Desligue o fio da INJ4 da ECU e ligue um jumper de PA0 do gerador
à entrada INJ4 do scope (PB13). A mesma borda chega com o instante exato do compare (CKP) e com o instante do EXTI (INJ4):
a diferença é a latência e o jitter do EXTI. Com USB ativo, a cada rpm da corrida A (800, 3000, 6000, 8500):

    python3 -I tools/ign_jitter.py --port /dev/ttyACM1 --seconds 60 --save /tmp/calib_3000.log --calib INJ4

Critério: (max − min) ≤ 1 µs, o scope resolve 0,1° até 8500 rpm; ≤ 3 µs,
use mas some ≤ 0,15° de incerteza a 8500 rpm; > 3 µs, use o analisador externo. A latência média sai como `--bias-us` para
subtrair nas corridas. Desfaça o jumper depois.

**Firmware.** Imagem normal (sem `EMS_CS_PROF`): o perfil acrescenta ciclos a cada guard e deslocaria a medida. Modo lâmpada
como no T3 (faísca fixa, sem correção nem knock). Dashboard conectado em metade das corridas e desconectado na outra
metade: a diferença mostra o efeito do tráfego do laço principal.

**Captura e análise.** `tools/ign_jitter.py` envia `e` (modo EDGE do scope) e `l` ao fim, lê as bordas, descarta as voltas
com ≠ 58 dentes de CKP (evento perdido pelo buffer de 2048 do scope) e calcula, por saída, média, desvio, p50, p99, p99,9 e
máximo de |erro| em graus e µs, mais o piso do gerador. Exemplo da corrida A a 3000 rpm:

    python3 -I tools/ign_jitter.py --port /dev/ttyACM1 --seconds 300 --save /tmp/ign_3000.log --advance 10 --bias-us <calibração>

O modo EDGE imprime todas as bordas, CKP incluído: a 8500 rpm são ~16 mil por segundo e podem estourar o buffer do scope
(o script avisa e descarta as voltas afetadas). Se o descarte passar de uns 5%, use rpm menor para essa corrida ou o
analisador externo. `--csv arq --label X` acrescenta uma linha por saída, para a varredura da corrida B.

**Grandezas.** `t_ref` = borda de subida do dente 0 mais próxima da faísca (o primeiro dente depois do gap). `T_rev` =
tempo entre dois dentes 0 seguidos; 1° = `T_rev` ÷ 360. Para cada faísca (borda de descida da IGN):

    erro (°) = ((t_faísca − t_ref) − (−avanço × T_rev / 360)) ÷ (T_rev / 360)

Com avanço 0°, a faísca do IGN1 deve coincidir com `t_ref` (PMS do cilindro 1 no dente 0; tabela do T3). Avanço positivo
= antes do PMS, negativo = depois. O desvio constante de ~0,3 µs (filtro de entrada de 256 ns do CKP + quantização,
`regs.h`) é viés conhecido: subtraia-o, ele vale 0,013° a 8500 rpm.

**Piso de ruído do gerador.** Antes de culpar o firmware, meça o próprio CKP: desvio-padrão e máximo de `T_rev` entre dentes 0
vizinhos. Essa dispersão é do gerador e entra em todo erro acima; o erro do firmware é o excesso sobre ela.

**Corrida A — regime, por rpm.** 800, 3000, 6000, 8500 rpm, sync cheio, injeção sequencial, avanço 10° (IGN1 e IGN4
caem dentro do gap angular; IGN2 e IGN3 fora). ≥ 5 min por ponto e por saída (a 3000 rpm cada IGN dispara 25 vezes por segundo, um ciclo de 720° a cada 40 ms:
5 min dão ~7500 amostras, suficientes para p99; p99,9 pede ≥ 30 min). Registre por saída: média, desvio, p50, p99, máximo
de |erro| em graus e em µs.

**Corrida B — varredura do avanço em torno do dente 0 (acha a colisão com o gap).** O rebuild da tabela angular roda na ISR
do dente 0 e dura cerca de 9 µs (15–21 µs o gap inteiro, T7b). Uma faísca que vence nessa janela espera. Para colocá-la
lá de propósito, use IGN1, 6000 e 8500 rpm, e varie o avanço do modo lâmpada de +2,0° a −2,0° em passos de 0,1°
(a 8500 rpm, 1° = 19,6 µs: 0 a −1° põe a faísca de 0 a ~20 µs depois do dente 0). Em cada passo, ≥ 2 min, com `--advance <valor> --csv /tmp/varredura.csv --label B_8500`. Trace o erro
(média e máximo, colunas do CSV) contra o avanço. **Esperado se a causa for o rebuild:** um degrau ou pico localizado entre 0° e ~−1°, com
largura de ~0,9° a 8500 rpm e menor a 6000. **Se for outra causa:** erro espalhado, sem relação com o avanço.

**Corrida C — aceleração.** Rampa de 3000 para 6000 rpm e de volta com o `SLEW` do gerador no valor mais alto que mantém
o sync cheio (confira a taxa real pelo período do CKP, não pelo comando). Avanço 10°. O simulador dá ≤ 0,5° a ±2700 rpm/s
(`docs/pipeline_motor.md`): registre o erro contra o rpm instantâneo.

**Correlação com o T7c.** Repita o ponto de 8500 rpm da corrida A com a imagem de perfil e leia `--cs` no fim. Se o
máximo mascarado do balde 0 ou 1 ficar na ordem do erro de pior caso em µs, a seção crítica explica a cauda.

**Como ler o resultado**

| Observação | Causa provável |
|---|---|
| Pico de erro na corrida B logo depois do dente 0, tamanho ≈ gap de T7b | rebuild do gap na ISR do TIM5 |
| Cauda de erro sem relação com o avanço, pior com o dashboard conectado | seção crítica do laço principal (confira T7c) |
| Erro cresce com a rampa da corrida C, nulo em regime | extrapolação do período do dente |
| Erro igual ao piso do gerador | limite da bancada, não do firmware |
| Faísca fora do lugar por ≈ um dente (6°) ou troca de IGN1/IGN4 | erro de fase/CMP, não de tempo |

**Limites do roteiro.** O script supõe rpm quase constante dentro de uma volta (a corrida C usa T_rev local e a rampa de 2700
rpm/s muda ~2% por volta), PMS por saída da tabela do T3 (`--pms` muda) e CKP idêntico à subida de cada dente. Testado só com
log sintético; o formato real do scope (`main.cpp`, `process_events`) foi lido, não capturado na bancada.

**Registro.** Anote rpm, avanço, dashboard ligado ou não, tamanho da amostra, e os números por saída no
`docs/bench_report_template.md` (linha de latência e jitter das saídas). Não há limite de aprovação fixado: o objetivo é
registrar os números e decidir com eles se o rebuild do gap, a seção crítica ou a arquitetura (TIM2) merecem mudança.
Medida em bancada: ainda não feita.

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
