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
| 12–13 | u16 LE | `default_eoi_lead_deg` | graus | 60 → `0x3C 0x00` |
| 14–15 | u16 LE | magic | — | **0x44 0x45** (v2/EOI — obrigatório) |

> **ATENÇÃO — magic obrigatório:** `engine_config_load()` verifica os bytes
> 14–15 (`0x44 0x45` em little-endian = 0x4544, versão v2/EOI). Se estiverem
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
DEFAULT_EOI_LEAD_DEG      = 60       # alvo de FIM de injecção (EOI, ° BTDC)
MAGIC                     = 0x4544   # v2 — semântica EOI. NÃO ALTERAR
# ─────────────────────────────────────────────────────────────────────────────

def build_page0(ivc, displacement, inj_flow, stoich, map_ref, trigger, eoi):
    """Constrói os 16 bytes da engine config page 0."""
    buf = bytearray(16)
    buf[0]  = ivc & 0xFF
    buf[1]  = 0x00                              # reservado
    struct.pack_into('<H', buf,  2, displacement)
    struct.pack_into('<H', buf,  4, inj_flow)
    struct.pack_into('<H', buf,  6, stoich)
    struct.pack_into('<H', buf,  8, map_ref)
    struct.pack_into('<H', buf, 10, trigger)
    struct.pack_into('<H', buf, 12, eoi)
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
            DEFAULT_EOI_LEAD_DEG,
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
  magic         = 0x4544
Nova configuração a escrever (hex): 32 00 D0 07 C2 01 BE 05 64 00 00 00 3E 00 43 45
✓ Escrita em NVM OK
✓ Verificação OK — configuração confirmada em NVM
```

---

## 7. Etapa 2 — Sinal CKP sintético e verificação de sincronismo

### 7.1 Anatomia do sinal CKP (60-2)

O decoder CKP do OpenEMS espera pulsos de bordo de subida em PA0:

```
 Roda 60-2: 58 dentes reais + gap de 2 posições em falta

  D0   D1   D2  ...  D57   GAP(2T)   D0   D1  ...
  ┌─┐  ┌─┐  ┌─┐      ┌─┐            ┌─┐  ┌─┐
──┘ └──┘ └──┘ └─ · · ─┘ └──────────┘ └──┘ └──
  ←T→  ←T→  ←T→       ←T→  ← 2T →   ←T→  ←T→
                             extra

  Período dente normal  : T
  Período do gap        : 3T (de RE(D57) a RE(D0) seguinte)
  Detecção de gap       : período_actual > 2 × período_anterior
```

**Tabela de tempos por RPM:**

| RPM | T (µs) | HIGH (µs) | LOW normal (µs) | LOW gap (µs) | Período rev. (ms) |
|-----|--------|-----------|-----------------|--------------|-------------------|
| 200 | 5000   | 2500      | 2500            | 12500        | 300               |
| 500 | 2000   | 1000      | 1000            | 5000         | 120               |
| 1000| 1000   | 500       | 500             | 2500         | 60                |
| 3000|  333   | 166       | 166             | 832          | 20                |

### 7.2 Gerar o sinal com ESP32 (recomendado)

O código completo está em `tools/esp32_ckp_gen/`. Dois ficheiros:

| Ficheiro | Quando usar |
|----------|-------------|
| `esp32_ckp_gen.ino` | Arduino C++ — jitter < 5 µs, RPM até 5000 |
| `ckp_gen_micropython.py` | MicroPython — mais simples, adequado ≤ 1000 RPM |

#### Ligações ESP32 → STM32H562

```
ESP32 GPIO 2  ──────────────►  PA0 (CKP input, TIM5_CH1)
ESP32 GPIO 4  ──────────────►  PA1 (CMP input, TIM5_CH2)
ESP32 GND     ──────────────►  GND do PCB   ← OBRIGATÓRIO
```

> Os dois microcontroladores têm de ter **GND comum**. Sem isso os bordos
> de subida são referenciados em tensões diferentes e o STM32 não detecta
> os pulsos.

#### Versão Arduino C++ (`esp32_ckp_gen.ino`)

```
1. Instalar Arduino IDE + placa "ESP32 by Espressif" (core ≥ 2.0)
2. Abrir tools/esp32_ckp_gen/esp32_ckp_gen.ino
3. Seleccionar: Board = "ESP32 Dev Module", Upload Speed = 921600
4. Fazer upload
5. Abrir Serial Monitor (115200 baud)
```

Saída esperada após upload:
```
=== OpenEMS CKP Generator (ESP32) ===
Comandos: '+'/'-' RPM±100 | '0'-'9' preset | 's' estado
[CKP] RPM=500  T=2000 µs  HIGH=1000 µs  LOW_gap=5000 µs  rev=120.0 ms
```

Comandos pelo monitor série:

| Tecla | Efeito |
|-------|--------|
| `+` | RPM + 100 |
| `-` | RPM − 100 |
| `3` | Preset 500 RPM |
| `5` | Preset 1000 RPM |
| `8` | Preset 3000 RPM |
| `s` | Imprimir estado (RPM, revoluções, pulsos CMP) |

Presets `'0'`–`'9'`: 100 / 200 / 300 / 500 / 700 / 1000 / 1500 / 2000 / 3000 / 5000 RPM.

O LED integrado pisca a cada 4 revoluções (~33 Hz a 500 RPM) — confirmar
visualmente que o sinal está a ser gerado.

#### Versão MicroPython (`ckp_gen_micropython.py`)

```bash
# Instalar MicroPython no ESP32 (se ainda não estiver)
esptool.py --chip esp32 erase_flash
esptool.py --chip esp32 write_flash -z 0x1000 esp32-20231227-v1.22.0.bin

# Copiar o ficheiro
mpremote cp tools/esp32_ckp_gen/ckp_gen_micropython.py :/ckp_gen.py

# Correr (bloqueante — Ctrl+C para parar)
mpremote run tools/esp32_ckp_gen/ckp_gen_micropython.py
```

Ou no REPL interactivo:
```python
import ckp_gen
ckp_gen.start(rpm=500)    # bloqueia até Ctrl+C
```

> **Nota de precisão:** o MicroPython no ESP32 tem jitter de ±50–200 µs
> devido ao escalonamento FreeRTOS. A 500 RPM (T=2000 µs) isso representa
> ±2.5–10% por dente. O detector CKP tolera esta variação para sincronismo
> mas não use para medir avanço de ignição — use a versão Arduino C++.

### 7.3 Verificar sincronismo via snapshot

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

## 8. Etapa 3 — ESP32 como osciloscópio lógico

O sketch `tools/esp32_scope/esp32_scope.ino` transforma o ESP32 num
analisador lógico de 9 canais com resolução de 1 µs, suficiente para
verificar todos os pulsos de ignição e injeção do OpenEMS.

### 8.1 Ligações ESP32 → STM32H562

```
ESP32            STM32H562        Função
─────────────    ──────────       ─────────────────
GPIO 32     ←─── PC6             TIM8_CH1 Ignição cil.0
GPIO 33     ←─── PC7             TIM8_CH2 Ignição cil.1
GPIO 25     ←─── PC8             TIM8_CH3 Ignição cil.2
GPIO 26     ←─── PC9             TIM8_CH4 Ignição cil.3
GPIO 27     ←─── PA15            TIM2_CH1 Injeção cil.0
GPIO 14     ←─── PB3             TIM2_CH2 Injeção cil.1
GPIO 12     ←─── PB10            TIM2_CH3 Injeção cil.2
GPIO 13     ←─── PB11            TIM2_CH4 Injeção cil.3
GPIO 36     ←─── PA0 (loopback)  CKP gerado pelo próprio ESP32
GND         ───► GND             OBRIGATÓRIO
```

> GPIO 36 (VP) é input-only no ESP32 — não pode ser acidentalmente
> configurado como saída. Ligar ao GPIO 2 do CKP generator com um fio
> curto para monitorizar o próprio sinal CKP gerado.
>
> Se usar **dois ESP32** (um para gerar CKP, outro para o scope), ligar
> o GPIO 2 do gerador ao GPIO 36 do scope, além dos 8 canais de saída
> do STM32.

### 8.2 Instalar e arrancar o scope

```
1. Abrir tools/esp32_scope/esp32_scope.ino no Arduino IDE
2. Verificar os GPIOs em kChan[] (ajustar ao DevKit se necessário)
3. Upload para o ESP32
4. Abrir Serial Monitor @ 115200 baud
```

Ao ligar, o scope imprime:

```
╔══════════════════════════════════════════╗
║   OpenEMS ESP32 Logic Scope              ║
║   Resolução: 1 µs  Latência: ~5 µs      ║
╚══════════════════════════════════════════╝
```

### 8.3 Modos de operação

| Tecla | Modo | Descrição |
|-------|------|-----------|
| `l` | **LIVE** (default) | Tabela de métricas actualizada a cada 1 s |
| `p` | **PULSE** | Uma linha por pulso completo (falling edge) |
| `e` | **EDGE** | Uma linha por bordo (rising e falling) |
| `w` | **WAVE** | Barra de texto dos últimos 300 ms |
| `s` | **STATS** | Mínimo/máximo/média desde o reset |
| `r` | Reset | Zera contadores e estatísticas |

### 8.4 Saída esperada (modo LIVE a 500 RPM)

```
+───────────────────────────────────────────────────────────────+
| OpenEMS Scope @ 12.345 s                                             |
+──+──────+───────+────────+────────+────────+───────+─────────+
|CH| Name |STM32  |PW (ms) |Per(ms) |Freq(Hz)| Count | Status  |
+──+──────+───────+────────+────────+────────+───────+─────────+
| 0|IGN0  |PC6    |   3.021|  240.00|    4.17|    500|  OK     |
| 1|IGN1  |PC7    |   3.019|  240.00|    4.17|    500|  OK     |
| 2|IGN2  |PC8    |   3.022|  240.00|    4.17|    500|  OK     |
| 3|IGN3  |PC9    |   3.020|  240.00|    4.17|    500|  OK     |
| 4|INJ0  |PA15   |   7.250|  240.00|    4.17|    500|  OK     |
| 5|INJ1  |PB3    |   7.248|  240.00|    4.17|    500|  OK     |
| 6|INJ2  |PB10   |   7.252|  240.00|    4.17|    500|  OK     |
| 7|INJ3  |PB11   |   7.249|  240.00|    4.17|    500|  OK     |
| 8|CKP   |PA0    |   1.000|    2.00|  500.00|  29000|  OK     |
+──+──────+───────+────────+────────+────────+───────+─────────+
  RPM estimado (IGN0 period): 500.0
```

**O que verificar na tabela:**

| Campo | IGN (TIM8) | INJ (TIM2) |
|-------|-----------|----------|
| PW (ms) | 1.0–5.0 (dwell) | calc. em 9.4 |
| Per (ms) | 240 a 500 RPM | 240 a 500 RPM |
| Freq (Hz) | 4.17 a 500 RPM | 4.17 a 500 RPM |
| Status | OK | OK |

Se algum canal mostrar **IDLE** mais de 2 s após FULL_SYNC: o pino
correspondente não está a receber pulsos — ver secção 12.

### 8.5 Verificar avanço de ignição com o scope

No modo PULSE, o ESP32 imprime cada pulso com timestamp:

```
CH0 IGN0   PW=  3.021 ms  T=240.000 ms  #501
CH0 IGN0   PW=  3.020 ms  T=240.001 ms  #502
```

Para verificar o avanço relativo ao gap CKP:

1. Activar modo **EDGE** (`e`) para ver todos os bordos com timestamps
2. Identificar o bordo RE do dente 0 do CKP (canal 8, RISE, após o gap)
3. Identificar o bordo FALL seguinte do IGN0 (canal 0, FALL = instante de spark)
4. Calcular:
   ```
   Δt = ts_IGN0_FALL - ts_CKP_RISE_dente0  (em µs)
   ângulo = Δt / T_dente × 6°  (6° por posição de dente)
   avanço = 360° - ângulo  (antes do TDC)
   ```
5. Comparar com o avanço definido na tabela spark (page 2)

### 8.6 Script de leitura no computador (opcional)

```bash
# Instalar dependência
pip install pyserial

# Modo live com gravação CSV
python3 tools/esp32_scope/scope_host.py \
    --port /dev/ttyUSB1 --mode live --csv bench_$(date +%Y%m%d_%H%M).csv

# Modo pulse (uma linha por pulso, mais legível)
python3 tools/esp32_scope/scope_host.py --port /dev/ttyUSB1 --mode pulse

# Analisar CSV gravado anteriormente
python3 tools/esp32_scope/scope_host.py --analyse bench_20260606_1430.csv
```

Saída de `--analyse`:

```
Análise de bench_20260606_1430.csv  (3000 amostras)

Canal    N     Média     Min       Max       σ         Avaliação
────────  ──────  ─────────  ─────────  ─────────  ────────
 IGN0      500   3.021ms  3.018ms  3.025ms  0.001ms  ✓ OK  (dwell 1–5 ms)
 IGN1      500   3.019ms  3.015ms  3.023ms  0.001ms  ✓ OK
 INJ0      500   7.251ms  7.245ms  7.258ms  0.002ms  ✓ OK  (PW 1–30 ms)
```

---

## 9. Etapa 4 — Medição dos pulsos de saída

### 9.1 O que medir

Com FULL_SYNC activo e a 500 RPM, o scheduler deve gerar:
- **TIM8 (ignição):** pulsos de dwell em PC6–PC9, um por cilindro por ciclo (720° de virabrequim)
- **TIM2 (injecção):** pulsos de injecção em PA15, PB3, PB10, PB11

O tempo de dwell padrão é lido da tabela `dwell_ms_x10_table` (page 5, offset 176). O default a 12 V é 3.0 ms.

### 9.2 Verificar duração do dwell

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

### 9.3 Verificar avanço de ignição

O avanço de ignição é lido da tabela `spark_table` (page 2). Para verificar o ângulo actual:

1. Com o osciloscópio, medir o tempo entre:
   - o fim do gap CKP (bordo de subida do dente 0 em PA0)
   - o bordo de descida do pulso de ignição (fim do dwell → SPARK) em PC6

2. Calcular o ângulo:
   ```
   tempo_medido / período_dente × 6° = ângulo desde dente 0
   ```

3. Comparar com o valor lido da tabela spark: `r page 2 offset correspondente a (RPM, MAP)`.

### 9.4 Verificar largura de pulso de injecção

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

## 10. Etapa 5 — Sinal CMP e fase

### 10.1 Porquê o CMP é necessário

Sem o sinal CMP (camshaft position), o firmware fica em **HALF_SYNC**: sabe quantos dentes passaram desde o gap, mas não sabe em que meia-volta do ciclo está (compressão ou escape do cil. 0). O scheduler usa injecção simultânea (wasted spark + bank fire) até ter FULL_SYNC.

Para injecção sequencial correcta é obrigatório o FULL_SYNC.

### 10.2 Simular o sinal CMP

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

## 11. Interpretar o snapshot em tempo real

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

## 12. Problemas comuns e diagnóstico

| Sintoma | Causa provável | Acção |
|---------|---------------|-------|
| Sem resposta a `Q` | TX/RX trocados; baud errado; placa sem 3.3 V | Verificar ligações, medir VDD |
| FULL_SYNC = False após 5 s | Nível de sinal em PA0 insuficiente; gap errado | Medir PA0 no osciloscópio; verificar 60-2 timing |
| RPM errado (ex: metade) | Gap detectado a cada 2 rotações (CMP confundido com CKP) | Confirmar PA0 = CKP, PA1 = CMP |
| Sem pulsos em PC6–PC9 | TIM8 não saiu de FORCE_INACTIVE | Verificar FULL_SYNC; verificar `ECU_Hardware_Init` via SWD |
| Dwell muito curto ou longo | Tabela dwell incorrecta; tensão de bateria diferente | Ler page 5 offset 176 (dwell_ms_x10_table), verificar vbatt_corr |
| NACK (0x01) em escrita page 0 | Flash write falhou; `engine_config_valid` rejeitou dados | Confirmar magic 0x4544 (v2); confirmar displacement > 0 |
| `late_event_count` > 0 | CPU sobrecarregada ou ISR a demorar muito | Verificar `loop2ms_max_us`; pode ser normal a baixo RPM |
| `g_flash_write_faults` > 0 | NVM corrompida ou falha de leitura no arranque | Apagar Bank2 via OpenOCD; reflash |

**Apagar Bank2 (calibrações) e repor defaults:**
```
openocd> flash erase_sector 1 1 7
# Apaga sectores 1–7 do Bank2 (calibrações)
# No próximo arranque o firmware usa os defaults de compilação
```

---

## 13. O que medir antes do primeiro arranque do motor

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

## 14. Referência rápida de comandos UART

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
| 0 | `0x00` | 512 B | Engine config (IVC, displacement, injector, AFR, trigger, EOI) |
| 1 | `0x01` | 256 B | Tabela VE 16×16 |
| 2 | `0x02` | 256 B | Tabela spark (avanço) 16×16 |
| 3 | `0x03` | 64 B | Snapshot real-time (read-only) |
| 4 | `0x04` | 512 B | Tabela lambda target |
| 5 | `0x05` | 256 B | Tabelas de correcção 1D (CLT, IAT, warmup, vbatt, dwell, AE...) |
| 6 | `0x06` | 80 B | X-Tau, AE, quick crank |

---

*Manual gerado automaticamente a partir do código-fonte do commit `3db627c`.*  
*Actualizar sempre que houver alterações ao protocolo ou ao mapeamento de páginas.*
