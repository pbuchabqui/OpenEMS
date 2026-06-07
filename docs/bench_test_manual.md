# OpenEMS — Manual de Teste em Bancada

**Versão firmware:** commit `3db627c` (586 PASS / 0 FAIL host tests)  
**Hardware alvo:** STM32H562RGT6 (LQFP64), 250 MHz, 3.3 V  
**Data:** 2026-06-06

---

## Índice

1. [Ferramentas e materiais necessários](#1-ferramentas-e-materiais-necessários)
2. [Ligações eléctricas de bancada](#2-ligações-eléctricas-de-bancada)
3. [Compilar e programar o firmware](#3-compilar-e-programar-o-firmware)
4. [Etapa 1 — Verificação de arranque](#4-etapa-1--verificação-de-arranque)
5. [Protocolo de comunicação UART](#5-protocolo-de-comunicação-uart)
6. [Configurar os parâmetros do motor](#6-configurar-os-parâmetros-do-motor)
7. [Etapa 2 — Sinal CKP sintético e verificação de sincronismo](#7-etapa-2--sinal-ckp-sintético-e-verificação-de-sincronismo)
8. [Etapa 3 — Medição dos pulsos de saída no osciloscópio](#8-etapa-3--medição-dos-pulsos-de-saída-no-osciloscópio)
9. [Etapa 4 — Sinal CMP e fase](#9-etapa-4--sinal-cmp-e-fase)
10. [Interpretar o snapshot em tempo real](#10-interpretar-o-snapshot-em-tempo-real)
11. [Problemas comuns e diagnóstico](#11-problemas-comuns-e-diagnóstico)
12. [O que medir antes do primeiro arranque do motor](#12-o-que-medir-antes-do-primeiro-arranque-do-motor)
13. [Referência rápida de comandos UART](#13-referência-rápida-de-comandos-uart)

---

## 1. Ferramentas e materiais necessários

| Item | Especificação mínima |
|------|---------------------|
| Programador SWD | ST-LINK v2/v3 ou J-Link |
| Osciloscópio | 2 canais, 50 MHz, modo cursores |
| Gerador de funções | Frequência até 2 kHz, saída 3.3 V ou 5 V ajustável |
| Terminal série | `picocom`, `minicom`, PuTTY, ou Python `pyserial` |
| Multímetro | Para verificar alimentação e continuidade |
| Fonte 12 V / 5 A | Tensão estável; não usar carregadores USB |
| Cabo USB-C (ou UART-USB) | Para comunicação com o firmware |
| Computador com Python 3 | Para os scripts de configuração incluídos neste manual |

> **NUNCA ligar injectors ou bobinas de ignição** durante as etapas 1–4.  
> Os pulsos de saída devem ser observados apenas com o osciloscópio
> (entrada de alta impedância). Cargas reais só após etapa de validação.

---

## 2. Ligações eléctricas de bancada

### Alimentação

```
12 V DC (+) ──► VBAT do PCB (verificar pino no esquemático)
GND          ──► GND do PCB
3.3 V        ──► gerado internamente pelo regulador do PCB
```

Meça 3.3 V nos pinos de alimentação do STM32 antes de ligar o SWD.

### Pinos de sinal (LQFP64)

| Pino STM32 | Função | Destino na bancada |
|-----------|--------|-------------------|
| PA0 | TIM5_CH1 — CKP input capture | Saída do gerador de funções (CKP) |
| PA1 | TIM5_CH2 — CMP input capture | Saída do gerador de funções (CMP) |
| PC6 | TIM8_CH1 — Ignição cil. 0 | Canal 1 do osciloscópio |
| PC7 | TIM8_CH2 — Ignição cil. 1 | Canal 2 do osciloscópio (opcional) |
| PC8 | TIM8_CH3 — Ignição cil. 2 | — |
| PC9 | TIM8_CH4 — Ignição cil. 3 | — |
| PA15 | TIM2_CH1 — Injecção cil. 0 | Canal 1 do osciloscópio (fase 3) |
| PB3  | TIM2_CH2 — Injecção cil. 1 | — |
| PB10 | TIM2_CH3 — Injecção cil. 2 | — |
| PB11 | TIM2_CH4 — Injecção cil. 3 | — |
| PA9  | UART1 TX | RX do adaptador USB-UART |
| PA10 | UART1 RX | TX do adaptador USB-UART |
| PA11 | USB D− | Conector USB (alternativa ao UART) |
| PA12 | USB D+ | Conector USB |
| PA13 | SWDIO | SWD do programador |
| PA14 | SWDCK | SWD do programador |

> O firmware suporta tanto **UART1 @ 115200 baud** como **USB-CDC** com o
> mesmo protocolo. Use o que for mais conveniente. USB-CDC requer que o
> computador detecte o dispositivo e abra a porta COM/ttyACM antes de
> enviar comandos.

---

## 3. Compilar e programar o firmware

```bash
# Host test (sem hardware, apenas verificação)
make host-test                 # deve imprimir: Results: 586 PASS  0 FAIL

# Build STM32
make stm32                     # produz build/openems.elf

# Programar via OpenOCD + ST-LINK
openocd -f interface/stlink.cfg \
        -f target/stm32h5x.cfg \
        -c "program build/openems.elf verify reset exit"
```

Após programar, o LED de status (se existir no PCB) deve piscar ao ritmo do SysTick (1 Hz por convenção, verificar no esquemático).

---

## 4. Etapa 1 — Verificação de arranque

### 4.1 Sequência de init esperada

O firmware faz as seguintes operações no startup, **antes** de entrar no loop principal:

1. `system_stm32_init()` — PLL → 250 MHz, SysTick @ 1 ms, **IWDG @ 100 ms**
2. `misfire_init()`, `tim5_ic_init()` — tabelas CKP, TIM5 input capture
3. `ECU_Hardware_Init()` — TIM2 e TIM8 a 10 MHz, outputs em modo FORCE_INACTIVE
4. `adc_init()` — ADC1/ADC2 com TIM6 trigger
5. `can0_init()`, `uart0_init()`, `usb_cdc_init()` — comunicação
6. Carregamento de NVM (Bank2) — calibrações
7. `nvic_enable_irq(IRQ_TIM5)` — ISR CKP activada por último
8. Loop de espera de FULL_SYNC (timeout 5 s)

> **IWDG** fica activo após passo 1. O loop principal chama `iwdg_kick()`
> pelo menos a cada 20 ms. Se o firmware travar antes de entrar no loop,
> o IWDG reinicia a placa em 100 ms. O bit `RCC_CSR.IWDGRSTF` fica activo
> após um reset por watchdog — verificar com OpenOCD se houver resets
> inesperados:
> ```
> openocd> mdw 0x44020C94
> # bit 26 = IWDGRSTF
> ```

### 4.2 Verificar comunicação UART

Ligar o adaptador USB-UART e abrir o terminal:

```bash
picocom -b 115200 --omap crcrlf /dev/ttyUSB0
```

Enviar byte `Q` (0x51). O firmware responde com a string ASCII:

```
OpenEMS_v1.1
```

Se não há resposta em 2 segundos:
- Verificar TX/RX não trocados (cruzamento TX→RX, RX→TX)
- Verificar baud rate (115200, 8N1)
- Verificar que a placa está alimentada (3.3 V nos pinos VDD do STM32)

### 4.3 Verificar versão e protocolo

| Comando (byte) | Resposta esperada |
|----------------|-------------------|
| `Q` (0x51) | `OpenEMS_v1.1` |
| `S` (0x53) | `OpenEMS_fw_1.1` |
| `F` (0x46) | `001` |
| `C` (0x43) | `0x00 0xAA` (ACK + magic) |

---

## 5. Protocolo de comunicação UART

O protocolo é **binário**, stateless e funciona em cima de UART ou USB-CDC com o mesmo formato.

### 5.1 Estrutura de pacotes

#### Leitura de página (`r`)

```
→ [0x72] [page] [off_lo] [off_hi] [len_lo] [len_hi]
← [len bytes de dados]
```

#### Escrita com gravação em NVM (`w`)

```
→ [0x77] [page] [off_lo] [off_hi] [len_lo] [len_hi] [len bytes de payload]
← [0x00]  (ACK ok)
  [0x01]  (NACK: fora de limites ou flash fail)
```

#### Escrita só em RAM, sem NVM (`x`)

```
→ [0x78] [page] [off_lo] [off_hi] [len_lo] [len_hi] [len bytes de payload]
← [0x00]  (ACK ok — sem escrita em flash)
```

#### Gravar página em NVM (`b`)

```
→ [0x62] [page]
← [0x00] (ok) / [0x01] (fail)
```

#### Snapshot tempo real (`A`)

```
→ [0x41]
← [64 bytes — ver secção 10]
```

### 5.2 Notas importantes

- **`page`** pode ser `0x00`–`0x06` (valor numérico) **ou** `'0'`–`'6'`
  (valor ASCII 0x30–0x36) — o firmware normaliza automaticamente.
- **Offset e length** são little-endian uint16. Para offsets < 256 e
  lengths < 256, o byte alto é 0x00.
- **Página 3** (snapshot tempo real) é **read-only** — escrita retorna 0x01.
- O comando `w` escreve no buffer RAM **e** grava em NVM na mesma operação.
  Use `x` para testar sem persistir; use `b` para gravar um buffer já
  editado com vários `x`.

---

## 6. Configurar os parâmetros do motor

### 6.1 Mapa da Página 0 (engine config)

| Offset | Tamanho | Campo | Unidade | Exemplo (2.0 L / E30) |
|--------|---------|-------|---------|----------------------|
| 0 | 1 B | `ivc_abdc_deg` | graus ABDC | 50 |
| 1 | 1 B | reservado | — | 0x00 |
| 2–3 | u16 LE | `displacement_cc` | cc | 2000 → `0xD0 0x07` |
| 4–5 | u16 LE | `injector_flow_cc_min` | cc/min | 450 → `0xC2 0x01` |
| 6–7 | u16 LE | `stoich_afr_x100` | AFR×100 | 1300 → `0x14 0x05` |
| 8–9 | u16 LE | `map_ref_bar_x100` | bar×100 | 100 → `0x64 0x00` |
| 10–11 | u16 LE | `trigger_tooth0_engine_deg` | graus | **MEDIR** |
| 12–13 | u16 LE | `default_soi_lead_deg` | graus | 62 → `0x3E 0x00` |
| 14–15 | u16 LE | magic | — | **0x43 0x45** (obrigatório) |

> **ATENÇÃO — magic obrigatório:** `engine_config_load()` verifica os bytes
> 14–15 (`0x43 0x45` = `'CE'` em little-endian = 0x4543). Se estiverem
> errados ou a zero, os campos 2–13 são **ignorados** e o firmware mantém
> os defaults de compilação. O comando `w` deve sempre incluir todos os
> 16 bytes.

### 6.2 Valores de stoich_afr_x100 por combustível

| Combustível | AFR estequiométrico | Valor a usar |
|-------------|---------------------|-------------|
| Gasolina 95/98 | 14.7 : 1 | 1470 → `0xBE 0x05` |
| E10 | 14.1 : 1 | 1410 → `0x82 0x05` |
| E30 | 13.0 : 1 | 1300 → `0x14 0x05` *(default)* |
| E85 | 9.8 : 1 | 980 → `0xD4 0x03` |

### 6.3 Medir o trigger offset (kTriggerTooth0EngineDeg)

Este é o parâmetro mais crítico. Um valor errado desloca **todos** os ângulos de ignição e injecção.

**Procedimento:**

1. Colocar dial indicator no pistão do cil. 0. Identificar e marcar o PMS
   (ponto morto superior) na polia do virabrequim.
2. Ligar o osciloscópio ao pino PA0 (CKP).
3. Rodar o motor **muito lentamente** à mão (ou com motor eléctrico a baixa
   tensão, sem combustível/ignição).
4. No osciloscópio, identificar o **dente 0**: é o primeiro pulso de subida
   **imediatamente após o gap** (o gap é uma ausência de 3 períodos normais
   numa roda dentada 60-2).
5. Medir o ângulo do virabrequim desde o **dente 0** até ao **PMS do cil. 0**.
   - Se o dente 0 ocorre **84° antes** do PMS: `trigger_tooth0_engine_deg = (720 - 84) % 720 = 636`
   - Se o dente 0 ocorre **no PMS** (alinhamento perfeito): `trigger_tooth0_engine_deg = 0`
   - Se o dente 0 ocorre **60° depois** do PMS: `trigger_tooth0_engine_deg = (720 - (-60)) % 720 = 60`

**Fórmula geral:**

```
trigger_tooth0_engine_deg = (720 - offset_deg_antes_do_PMS) % 720
```

onde `offset_deg_antes_do_PMS` é positivo se o dente 0 estiver **antes** do PMS e negativo se estiver depois.

### 6.4 Escrever os parâmetros via UART (script Python)

Guardar como `set_engine_config.py` e executar após ligar o firmware:

```python
#!/usr/bin/env python3
"""
set_engine_config.py — Configura os parâmetros de motor via protocolo
                       OpenEMS (UART 115200 ou USB-CDC).
Editar as constantes na secção CONFIGURAÇÃO DO MOTOR antes de correr.
"""
import serial
import struct
import sys
import time

# ─── CONFIGURAÇÃO DO MOTOR ────────────────────────────────────────────────────
PORT              = "/dev/ttyUSB0"   # ou "COM3" no Windows, "/dev/ttyACM0" para USB-CDC
BAUD              = 115200

IVC_ABDC_DEG              = 50       # graus ABDC (não alterar sem recalcular tabela VE)
DISPLACEMENT_CC           = 2000     # cilindrada em cc
INJECTOR_FLOW_CC_MIN      = 450      # caudal do injetor em cc/min
STOICH_AFR_X100           = 1470     # AFR estequiométrico × 100 (1470 = gasolina)
MAP_REF_BAR_X100          = 100      # pressão de referência (100 = 1.00 bar = atmosférica)
TRIGGER_TOOTH0_ENGINE_DEG = 0        # MEDIR NO MOTOR — ver secção 6.3
DEFAULT_SOI_LEAD_DEG      = 62       # avanço de injecção por defeito (SOI lead)
MAGIC                     = 0x4543   # NÃO ALTERAR
# ─────────────────────────────────────────────────────────────────────────────

def build_page0(ivc, displacement, inj_flow, stoich, map_ref, trigger, soi):
    """Constrói os 16 bytes da engine config page 0."""
    buf = bytearray(16)
    buf[0]  = ivc & 0xFF
    buf[1]  = 0x00                              # reservado
    struct.pack_into('<H', buf,  2, displacement)
    struct.pack_into('<H', buf,  4, inj_flow)
    struct.pack_into('<H', buf,  6, stoich)
    struct.pack_into('<H', buf,  8, map_ref)
    struct.pack_into('<H', buf, 10, trigger)
    struct.pack_into('<H', buf, 12, soi)
    struct.pack_into('<H', buf, 14, MAGIC)
    return buf

def write_page(ser, page_id, offset, data):
    """Envia comando 'w' (write + burn NVM). Retorna True se ACK 0x00."""
    length = len(data)
    cmd = bytes([
        0x77,                           # 'w'
        page_id,
        offset & 0xFF, (offset >> 8) & 0xFF,
        length & 0xFF, (length >> 8) & 0xFF,
    ]) + bytes(data)
    ser.write(cmd)
    time.sleep(0.1)                     # flash write pode demorar ~50 ms
    resp = ser.read(1)
    return len(resp) == 1 and resp[0] == 0x00

def read_page(ser, page_id, offset, length):
    """Envia comando 'r'. Retorna os bytes lidos."""
    cmd = bytes([
        0x72,                           # 'r'
        page_id,
        offset & 0xFF, (offset >> 8) & 0xFF,
        length & 0xFF, (length >> 8) & 0xFF,
    ])
    ser.write(cmd)
    time.sleep(0.05)
    return ser.read(length)

def check_comms(ser):
    """Testa comunicação com handshake 'C'."""
    ser.write(bytes([0x43]))            # 'C'
    time.sleep(0.05)
    resp = ser.read(2)
    return len(resp) == 2 and resp[0] == 0x00 and resp[1] == 0xAA

def main():
    print(f"Ligando a {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
        time.sleep(0.5)                 # aguardar USB-CDC enumerar
        ser.reset_input_buffer()

        # 1) Teste de comunicação
        if not check_comms(ser):
            print("ERRO: sem resposta ao handshake 'C'. Verifique ligações.")
            sys.exit(1)
        print("✓ Comunicação OK")

        # 2) Ler configuração actual
        current = read_page(ser, 0x00, 0, 16)
        if len(current) != 16:
            print("ERRO: não foi possível ler page 0.")
            sys.exit(1)
        print("Configuração actual (page 0):")
        print(f"  displacement  = {struct.unpack_from('<H', current, 2)[0]} cc")
        print(f"  injector_flow = {struct.unpack_from('<H', current, 4)[0]} cc/min")
        print(f"  stoich_afr    = {struct.unpack_from('<H', current, 6)[0] / 100:.2f} : 1")
        print(f"  trigger_deg   = {struct.unpack_from('<H', current, 10)[0]}°")
        print(f"  magic         = 0x{struct.unpack_from('<H', current, 14)[0]:04X}")

        # 3) Construir nova configuração
        new_cfg = build_page0(
            IVC_ABDC_DEG,
            DISPLACEMENT_CC,
            INJECTOR_FLOW_CC_MIN,
            STOICH_AFR_X100,
            MAP_REF_BAR_X100,
            TRIGGER_TOOTH0_ENGINE_DEG,
            DEFAULT_SOI_LEAD_DEG,
        )
        print(f"\nNova configuração a escrever (hex): {new_cfg.hex(' ').upper()}")

        # 4) Escrever e gravar em NVM
        if not write_page(ser, 0x00, 0, new_cfg):
            print("ERRO: NACK na escrita de page 0. Ver secção 11 do manual.")
            sys.exit(1)
        print("✓ Escrita em NVM OK")

        # 5) Verificação de leitura (read-back)
        verify = read_page(ser, 0x00, 0, 16)
        if verify == bytes(new_cfg):
            print("✓ Verificação OK — configuração confirmada em NVM")
        else:
            print("AVISO: os bytes lidos diferem dos escritos!")
            print(f"  Escrito: {bytes(new_cfg).hex(' ').upper()}")
            print(f"  Lido:    {bytes(verify).hex(' ').upper()}")

if __name__ == "__main__":
    main()
```

**Executar:**
```bash
pip install pyserial
python3 set_engine_config.py
```

Saída esperada:
```
Ligando a /dev/ttyUSB0 @ 115200...
✓ Comunicação OK
Configuração actual (page 0):
  displacement  = 2000 cc
  injector_flow = 450 cc/min
  stoich_afr    = 14.70 : 1
  trigger_deg   = 0°
  magic         = 0x4543
Nova configuração a escrever (hex): 32 00 D0 07 C2 01 BE 05 64 00 00 00 3E 00 43 45
✓ Escrita em NVM OK
✓ Verificação OK — configuração confirmada em NVM
```

---

## 7. Etapa 2 — Sinal CKP sintético e verificação de sincronismo

### 7.1 Configurar o gerador de funções (roda dentada 60-2)

O decoder CKP espera pulsos de bordo de subida em PA0, com:
- 58 dentes normais de período `T`
- 1 gap de duração `≥ 2.5 × T` (na prática o firmware detecta `> 2 × T`)
- Total: 58 dentes + 1 gap = equivalente a 60 posições por volta do virabrequim

Para simular **500 RPM** (cranking realista):
```
Frequência total por revolução: 500/60 = 8.33 rev/s
Período de dente normal: 1 / (8.33 × 60) = 2.0 ms → T = 2000 µs
Gap: 3 × T = 6000 µs (ausência de pulso)
Nível: 0–3.3 V (TTL directo no GPIO, sem pull-up externo necessário)
```

**Com gerador DDS (ex: Rigol DG1022):**

Não é possível gerar directamente o padrão 60-2 com funções simples. Usar modo **ARB** (arbitrary waveform) com uma sequência de 60 ciclos onde os últimos 2 são "vazios" (low). Alternativamente, usar um microcontrolador auxiliar (Arduino/RP2040) com o seguinte código:

```python
# Exemplo MicroPython (RP2040) — gera sinal 60-2 em GPIO 0
import machine, time

ckp = machine.Pin(0, machine.Pin.OUT)
RPM = 500
tooth_period_us = int(60_000_000 / (RPM * 60))  # µs por dente

while True:
    for i in range(58):           # 58 dentes normais
        ckp.value(1)
        time.sleep_us(tooth_period_us // 2)
        ckp.value(0)
        time.sleep_us(tooth_period_us // 2)
    # gap: 2 posições em silêncio (2 × tooth_period)
    time.sleep_us(tooth_period_us * 2)
```

### 7.2 Verificar sincronismo via snapshot

Com o sinal CKP activo, enviar snapshot:

```python
# get_snapshot.py
import serial, struct, time

def get_snapshot(port="/dev/ttyUSB0", baud=115200):
    with serial.Serial(port, baud, timeout=1.0) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write(bytes([0x41]))    # 'A'
        data = ser.read(64)
        if len(data) != 64:
            print("Erro: resposta incompleta")
            return
        rpm        = struct.unpack_from('<H', data, 0)[0]
        map_x100   = data[2]
        tps_pct    = data[3]
        clt_degc   = data[4] - 40
        iat_degc   = data[5] - 40
        status     = struct.unpack_from('<H', data, 11)[0]
        sync_full  = bool(status & 0x01)
        phase_a    = bool(status & 0x02)
        sens_fault = bool(status & 0x04)
        late_evt   = bool(status & 0x40)
        late_count = struct.unpack_from('<I', data, 13)[0]
        drop_count = struct.unpack_from('<I', data, 23)[0]

        print(f"RPM          : {rpm}")
        print(f"MAP          : {map_x100 / 100:.2f} bar")
        print(f"TPS          : {tps_pct} %")
        print(f"CLT          : {clt_degc} °C")
        print(f"IAT          : {iat_degc} °C")
        print(f"FULL_SYNC    : {sync_full}")
        print(f"PHASE_A      : {phase_a}")
        print(f"SENSOR_FAULT : {sens_fault}")
        print(f"LATE_EVENTS  : {late_evt}  (count={late_count})")
        print(f"DROP_CYCLES  : {drop_count}")

get_snapshot()
```

**Valores esperados a 500 RPM com sinal CKP sintético:**

| Campo | Esperado |
|-------|----------|
| RPM | 480–520 |
| FULL_SYNC | True |
| PHASE_A | True ou False (sem CMP é aleatório) |
| SENSOR_FAULT | False (sensores ADC sem sinal → default map=1.0 bar) |
| LATE_EVENTS | 0 a 500 RPM |
| DROP_CYCLES | 0 |

> Se FULL_SYNC = False após 3 segundos de sinal CKP:
> - Verificar nível de tensão no PA0 (deve atingir ≥ 2.0 V no bordo de subida)
> - Verificar que o gap tem duração correcta (≥ 2 × período de dente)
> - Verificar no osciloscópio se chegam pulsos ao pino PA0

---

## 8. Etapa 3 — Medição dos pulsos de saída no osciloscópio

### 8.1 O que medir

Com FULL_SYNC activo e a 500 RPM, o scheduler deve gerar:
- **TIM8 (ignição):** pulsos de dwell em PC6–PC9, um por cilindro por ciclo (720° de virabrequim)
- **TIM2 (injecção):** pulsos de injecção em PA15, PB3, PB10, PB11

O tempo de dwell padrão é lido da tabela `dwell_ms_x10_table` (page 5, offset 176). O default a 12 V é 3.0 ms.

### 8.2 Verificar duração do dwell

```
A 500 RPM:
  1 revolução = 120 ms
  1 ciclo (720°) = 240 ms
  Dwell esperado = 3.0 ms
  Período entre sparks do mesmo cilindro = 240 ms
```

No osciloscópio em PC6 (TIM8_CH1, cil. 0):
1. Trigger: bordo de subida
2. Escala: 5 ms/div horizontal, 2 V/div vertical
3. Medir a duração do pulso HIGH (dwell): deve ser **3.0 ± 0.1 ms**
4. Medir o período entre pulsos consecutivos do mesmo canal: deve ser **240 ± 5 ms**

### 8.3 Verificar avanço de ignição

O avanço de ignição é lido da tabela `spark_table` (page 2). Para verificar o ângulo actual:

1. Com o osciloscópio, medir o tempo entre:
   - o fim do gap CKP (bordo de subida do dente 0 em PA0)
   - o bordo de descida do pulso de ignição (fim do dwell → SPARK) em PC6

2. Calcular o ângulo:
   ```
   tempo_medido / período_dente × 6° = ângulo desde dente 0
   ```

3. Comparar com o valor lido da tabela spark: `r page 2 offset correspondente a (RPM, MAP)`.

### 8.4 Verificar largura de pulso de injecção

A largura de pulso de injecção (`inj_pw_ticks`) é calculada pelo `fuel_calc`. Para verificar:

```python
# Calcular req_fuel base para o motor configurado
# (confirmar independentemente antes de ligar injectors)
IVC_ABDC_DEG     = 50
DISPLACEMENT_CC  = 2000
INJECTOR_FLOW    = 450    # cc/min
STOICH_AFR_X100  = 1470   # gasolina

# req_fuel_us = (displacement/cylinders × air_density) /
#               (injector_flow/60 × fuel_density / stoich) × 1e6
cylinders        = 4
air_density      = 1.184  # mg/cc @ 1 bar
fuel_density     = 755    # mg/cc
inj_flow_cc_s    = INJECTOR_FLOW / 60.0
req_fuel_us      = (DISPLACEMENT_CC / cylinders * air_density) / \
                   (inj_flow_cc_s * fuel_density / (STOICH_AFR_X100 / 100.0)) * 1e6
print(f"req_fuel_us = {req_fuel_us:.0f} µs")
# Esperado: ~6500–7500 µs para este motor (gasolina, VE=100%, lambda=1.00)
```

No osciloscópio em PA15 (TIM2_CH1, cil. 0):
- Escala: 5 ms/div, trigger bordo de subida
- Medir duração do pulso HIGH: deve coincidir com o cálculo acima × VE%

---

## 9. Etapa 4 — Sinal CMP e fase

### 9.1 Porquê o CMP é necessário

Sem o sinal CMP (camshaft position), o firmware fica em **HALF_SYNC**: sabe quantos dentes passaram desde o gap, mas não sabe em que meia-volta do ciclo está (compressão ou escape do cil. 0). O scheduler usa injecção simultânea (wasted spark + bank fire) até ter FULL_SYNC.

Para injecção sequencial correcta é obrigatório o FULL_SYNC.

### 9.2 Simular o sinal CMP

O sinal CMP é 1 pulso por 2 rotações do virabrequim (1 por ciclo de 720°), em PA1.

```
A 500 RPM:
  Período CMP = 2 × 120 ms = 240 ms
  Duração do pulso: ≥ 1 ms (bordo de subida detectado)
  Fase: o pulso deve ocorrer numa janela dentro da fase A (ver documentação do sensor real)
```

Com o gerador, gerar uma onda quadrada em PA1 de **frequência = RPM / 120** Hz:
- 500 RPM → 500/120 ≈ 4.17 Hz → período ≈ 240 ms

Confirmar via snapshot que `PHASE_A` estabiliza (não alterna aleatoriamente) após o pulso CMP:

```python
import time
for _ in range(5):
    get_snapshot()
    time.sleep(0.5)
# PHASE_A deve ser consistente (sempre True ou sempre False)
```

---

## 10. Interpretar o snapshot em tempo real

### Estrutura dos 64 bytes (page 3, read-only)

| Byte(s) | Campo | Interpretação |
|---------|-------|---------------|
| 0–1 | `rpm` (u16 LE) | RPM = valor × 1 |
| 2 | `map_bar_x100` (u8) | MAP = valor / 100 bar |
| 3 | `tps_pct` (u8) | TPS = valor % |
| 4 | `clt_p40` (i8) | CLT °C = valor − 40 |
| 5 | `iat_p40` (i8) | IAT °C = valor − 40 |
| 6 | `o2_mv_d4` (u8) | Lambda×1000 = valor × 4 (via CAN) |
| 7 | `pw1_ms_x10` (u8) | PW injecção = valor / 10 ms |
| 8 | `advance_p40` (u8) | Avanço °BTDC = valor − 40 |
| 9 | `ve` (u8) | VE = valor % |
| 10 | `stft_p100` (i8) | STFT = valor % (negativo = enriquece) |
| 11–12 | `status_bits` (u16 LE) | Ver tabela abaixo |
| 13–16 | `late_event_count` (u32 LE) | Eventos com CCR atrasado |
| 23–26 | `cycle_drop_count` (u32 LE) | Ciclos descartados por overflow |
| 31 | `sync_state_raw` (u8) | 0=NONE, 1=HALF, 2=FULL |
| 35–38 | `loop2ms_last_us` (u32 LE) | Duração última iteração 2 ms (µs) |
| 39–42 | `loop2ms_max_us` (u32 LE) | Pico de duração iteração 2 ms (µs) |

### Status bits

| Bit | Máscara | Significado |
|-----|---------|-------------|
| 0 | 0x0001 | FULL_SYNC activo |
| 1 | 0x0002 | Fase A activa |
| 2 | 0x0004 | Falha de sensor (ADC fora de limites) |
| 3 | 0x0008 | Limp mode activo |
| 4 | 0x0010 | ETB limp (borboleta electrónica) |
| 5 | 0x0020 | X-Tau learning activo |
| 6 | 0x0040 | Scheduler late event |
| 7 | 0x0080 | Scheduler cycle drop |
| 8 | 0x0100 | Injection PW clamped (IVC limit) |
| 9 | 0x0200 | WBO2 fault (CAN lambda timeout) |

---

## 11. Problemas comuns e diagnóstico

| Sintoma | Causa provável | Acção |
|---------|---------------|-------|
| Sem resposta a `Q` | TX/RX trocados; baud errado; placa sem 3.3 V | Verificar ligações, medir VDD |
| FULL_SYNC = False após 5 s | Nível de sinal em PA0 insuficiente; gap errado | Medir PA0 no osciloscópio; verificar 60-2 timing |
| RPM errado (ex: metade) | Gap detectado a cada 2 rotações (CMP confundido com CKP) | Confirmar PA0 = CKP, PA1 = CMP |
| Sem pulsos em PC6–PC9 | TIM8 não saiu de FORCE_INACTIVE | Verificar FULL_SYNC; verificar `ECU_Hardware_Init` via SWD |
| Dwell muito curto ou longo | Tabela dwell incorrecta; tensão de bateria diferente | Ler page 5 offset 176 (dwell_ms_x10_table), verificar vbatt_corr |
| NACK (0x01) em escrita page 0 | Flash write falhou; `engine_config_valid` rejeitou dados | Confirmar magic 0x4543; confirmar displacement > 0 |
| `late_event_count` > 0 | CPU sobrecarregada ou ISR a demorar muito | Verificar `loop2ms_max_us`; pode ser normal a baixo RPM |
| `g_flash_write_faults` > 0 | NVM corrompida ou falha de leitura no arranque | Apagar Bank2 via OpenOCD; reflash |

**Apagar Bank2 (calibrações) e repor defaults:**
```
openocd> flash erase_sector 1 1 7
# Apaga sectores 1–7 do Bank2 (calibrações)
# No próximo arranque o firmware usa os defaults de compilação
```

---

## 12. O que medir antes do primeiro arranque do motor

Antes de qualquer tentativa de arranque com combustível, confirmar
**obrigatoriamente**:

### Lista de verificação pré-arranque

- [ ] `trigger_tooth0_engine_deg` medido e escrito via UART (secção 6.3)
- [ ] `displacement_cc` corresponde ao motor real
- [ ] `injector_flow_cc_min` corresponde à ficha técnica dos injectors
- [ ] `stoich_afr_x100` corresponde ao combustível usado
- [ ] `kFiringOrder` em `engine_config.h` corresponde à ordem de ignição do motor (recompilação necessária se diferir de {0,2,3,1} = 1-3-4-2)
- [ ] Pulsos TIM8 verificados no osciloscópio com duração e posição correctas
- [ ] Pulsos TIM2 verificados no osciloscópio com largura correcta para o req_fuel calculado
- [ ] FULL_SYNC estável com CKP real (não sintético) antes de adicionar combustível
- [ ] IWDG confirmado (provocar reset intencional desligando `iwdg_kick` num build de teste)
- [ ] Primeiro arranque: mão no corte de combustível, avanço conservador (5–8°)

### Verificação do trigger offset no motor real

Rodar o motor à mão, com osciloscópio duplo em PA0 (CKP) e na marca de TDC:

```
Passo 1: Identificar dente 0 no osciloscópio (primeiro pulso após gap)
Passo 2: Medir ângulo entre dente 0 e marca TDC cil. 0
Passo 3: Calcular trigger_tooth0_engine_deg = (720 - offset) % 720
Passo 4: Escrever via set_engine_config.py com o valor medido
Passo 5: Re-verificar: a posição do pulso de ignição no osciloscópio
         deve corresponder a advance_deg° antes do TDC
```

---

## 13. Referência rápida de comandos UART

### Enviar comandos manualmente com Python

```python
import serial, time

def cmd(ser, data, read_n=0):
    ser.write(bytes(data))
    time.sleep(0.1)
    if read_n: return ser.read(read_n)
    return b''

with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as s:

    # Handshake
    cmd(s, [0x43])                              # 'C' → espera 0x00 0xAA

    # Ler page 0 completa (16 bytes)
    r = cmd(s, [0x72, 0x00, 0x00, 0x00, 0x10, 0x00], read_n=16)
    print(r.hex(' '))

    # Snapshot
    snap = cmd(s, [0x41], read_n=64)
    import struct
    print("RPM:", struct.unpack_from('<H', snap, 0)[0])
    print("STATUS:", hex(struct.unpack_from('<H', snap, 11)[0]))
```

### Tabela de comandos

| Byte | Comando | Descrição |
|------|---------|-----------|
| `0x51` (`Q`) | Handshake | Responde `OpenEMS_v1.1` |
| `0x53` (`S`) | Versão FW | Responde `OpenEMS_fw_1.1` |
| `0x43` (`C`) | Teste comms | Responde `0x00 0xAA` |
| `0x41` (`A`) | Snapshot RT | 64 bytes de dados em tempo real |
| `0x72` (`r`) | Leitura de página | 5 args: page, off_lo, off_hi, len_lo, len_hi |
| `0x77` (`w`) | Escrita + NVM | 5 args + payload; ACK `0x00`/`0x01` |
| `0x78` (`x`) | Escrita RAM | 5 args + payload; ACK sem NVM |
| `0x62` (`b`) | Burn para NVM | 1 arg: page; ACK `0x00`/`0x01` |
| `0x64` (`d`) | Dirty mask | 1 byte: bits 0-5 = páginas com edições não gravadas |

### Páginas disponíveis

| Page | ID | Tamanho | Conteúdo |
|------|----|---------|---------|
| 0 | `0x00` | 512 B | Engine config (IVC, displacement, injector, AFR, trigger, SOI) |
| 1 | `0x01` | 256 B | Tabela VE 16×16 |
| 2 | `0x02` | 256 B | Tabela spark (avanço) 16×16 |
| 3 | `0x03` | 64 B | Snapshot real-time (read-only) |
| 4 | `0x04` | 512 B | Tabela lambda target |
| 5 | `0x05` | 256 B | Tabelas de correcção 1D (CLT, IAT, warmup, vbatt, dwell, AE...) |
| 6 | `0x06` | 80 B | X-Tau, AE, quick crank |

---

*Manual gerado automaticamente a partir do código-fonte do commit `3db627c`.*  
*Actualizar sempre que houver alterações ao protocolo ou ao mapeamento de páginas.*
