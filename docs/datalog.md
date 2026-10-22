# Datalog em cartão SD

Registro do estado do motor a 100 Hz num cartão SD, em blocos brutos de 512 B, sem sistema de arquivos. É a fonte de dados para afinar os ganhos do STFT, o X-τ e qualquer modelo aprendido (ver [`nn_viabilidade.md`](nn_viabilidade.md)).

**Estado:** o formato, o anel e a busca do fim do log estão testados no host (`make host-test`). O driver SDMMC (`src/hal/sd.cpp`) compila na VGT6, mas **nunca rodou em hardware**; a validação está em `docs/bench_report_template.md`.

## Placa

Só a VGT6. O SDMMC1 usa PC8, PC9, PC10, PC11 (D0 a D3), PC12 (CK) e PD2 (CMD), todos em AF12. Na RGT6, PC8/PC9 são IGN3/4 e PC10/PC11 são INJ3/4 (`docs/hw/pinout.md`), então lá o código nem entra no binário. Na VGT6 INJ/IGN estão na GPIOE.

O kernel clock é o PLL1Q (62,5 MHz, o mesmo do FDCAN): SDMMC_CK = 62,5 MHz / (2 × CLKDIV), com 395 kHz na identificação (CLKDIV 79) e 15,6 MHz nos dados (CLKDIV 2). Sem o PLL ligado o cartão fica ausente. Só SDHC/SDXC (endereçamento por bloco).

## Formato

A região começa no setor 2048 (1 MiB, fora de tabela de partição) e tem 262 144 blocos (128 MiB). Escrita linear, sem reciclar.

Bloco de 512 B:

| Bytes | Conteúdo |
|---|---|
| 0–3 | magic `"OLG1"` (`0x31474C4F`) |
| 4–7 | sessão (uma por boot) |
| 8–11 | sequência dentro da sessão |
| 12–15 | CRC-32 (zlib) do bloco inteiro, com este campo em zero |
| 16–495 | 15 registros de 32 B |
| 496 | registros válidos no bloco (15; menos no bloco parcial gravado ao desligar o log) |
| 497–511 | reservado, zero |

Registro de 32 B, little-endian:

| Offset | Campo | Tipo |
|---|---|---|
| 0 | `t_ms` | u32 |
| 4 | `rpm_x10` | u16 |
| 6 | `map_bar_x100` | u16 |
| 8 | `tps_x10` | u16 |
| 10 | `clt_x10` | s16 |
| 12 | `iat_x10` | s16 |
| 14 | `lambda_x1000` | u16 |
| 16 | `lambda_target_x1000` | u16 |
| 18 | `pw_us` | u16 |
| 20 | `advance_x10` | s16 |
| 22 | `stft_x10` | s16 |
| 24 | `ltft_x10` | s16 |
| 26 | `status_bits` | u16 |
| 28 | `cut_reasons` (`g_fuel_cut_reasons`) | u16 |
| 30 | `sync_state` | u8 |
| 31 | `flags` (bit 0: lambda válido) | u8 |

Taxas (comando `L`, abaixo): 100 Hz (3,2 KB/s, padrão), 200, 500 ou 1 kHz (32 KB/s). A 100 Hz, 128 MiB duram cerca de 11 h; a 1 kHz, pouco mais de 1 h. O anel de 32 blocos dá ~480 ms de folga a 1 kHz para o cartão atrasar sem descartar.

## Como funciona

- `datalog_push()` (bloco de 10 ms) só copia 32 B. Quando 15 amostras fecham um bloco, ele entra num anel de 16 blocos em RAM.
- `sd_poll()` e `datalog_poll()` rodam no laço principal, fora da medição de `loop2`, e avançam uma operação por chamada. Nada bloqueia e nada roda em ISR.
- Cartão lento: o anel enche, o bloco novo é descartado e a perda aparece como lacuna de sequência (e em `records_dropped`).
- No boot, uma busca binária assíncrona acha o fim do log anterior (os blocos válidos formam um prefixo) e a sessão nova é a última mais um. Um bloco cortado por perda de energia tem CRC inválido e é sobrescrito.
- Três falhas de gravação seguidas param o log (estado FAULT) e levantam `DATALOG_FAULT` (0x0530) como WARNING. O log nunca corta o motor.
- Região cheia: estado FULL, o log para.

## Comando e contadores

Comando `L` do protocolo (1 byte de argumento): bit 0 = ligado, bits 1–2 = taxa (0 a 3 = 100/200/500/1000 Hz), bit 7 = só consultar. Resposta: `[ACK][estado do log][estado do SD]`. O log liga com 100 Hz no boot e a escolha não é gravada em NVM. Ao desligar, o bloco parcial é gravado (o byte 496 diz quantos registros ele tem), então nada se perde.

No dash: `GET /api/datalog` consulta e `POST /api/datalog?enable=true&rate=3` liga e escolhe a taxa (`OpenEMSLink.datalog_control()`).

Contadores no realtime (OCH, offsets absolutos, canais no `openems.ini`):

| Offset | Canal | Conteúdo |
|---|---|---|
| 36 bits 0–3 | `datalogState` | 0 OFF, 1 SEARCH, 2 ACTIVE, 3 FULL, 4 FAULT |
| 36 bits 4–7 | `sdState` | 0 ausente, 1 inicializando, 2 pronto, 3 falha |
| 37 bit 0 | `datalogOn` | ligado |
| 37 bits 1–2 | `datalogRate` | código da taxa |
| 37 bits 3–7 | `datalogErrs` | erros de gravação (satura em 31) |
| 38–41 | `datalogBlocks` | blocos gravados (u32) |
| 42–43 | `datalogDrops` | amostras descartadas (u16, satura) |

## Ferramentas

```
python3 -I tools/datalog/sdlog.py format <dispositivo|imagem>   # zera a região (uma vez por cartão)
python3 -I tools/datalog/sdlog.py decode <dispositivo|imagem> -o log.csv
python3 -I tools/datalog/sdlog.py selftest                      # make datalog-check
```

O cartão precisa ser preparado com `format` antes do primeiro uso: sem isso, lixo antigo com magic e CRC válidos poderia ser tomado por log. `format` escreve em dispositivo bruto, então confira o caminho com `lsblk` antes.

O registro dourado (`GOLDEN_BYTES` no `sdlog.py`) é o mesmo vetor conferido em `test/test_datalog.cpp`, para o decodificador e o firmware não divergirem.

## Código

| Arquivo | Papel |
|---|---|
| `src/app/datalog.{h,cpp}` | formato, anel, busca do fim, máquina de gravação |
| `src/hal/sd.h` | interface de bloco assíncrona |
| `src/hal/sd.cpp` | driver SDMMC1 (só VGT6, só ARM) |
| `src/hal/sd_sim.cpp` | cartão simulado em RAM para o host-test |
| `src/main_stm32.cpp` | init, amostra no período escolhido, poll a cada volta do laço |
| `src/app/ui_protocol*.cpp` | comando `L` e contadores no realtime |
| `tools/datalog/sdlog.py` | preparar e decodificar |
