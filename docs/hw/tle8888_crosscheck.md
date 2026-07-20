# TLE8888 — contraprova independente contra o rusEFI

> **Porque este ficheiro existe.** O nosso `src/hal/tle8888_regs.h` foi escrito a partir de **uma**
> leitura do datasheet Rev 1.2, depois de se descobrir que o driver anterior tinha um mapa de
> registadores **inventado** (ver `interface_board_v1.md`). Uma segunda leitura minha não prova nada —
> repetiria o mesmo erro. O que prova alguma coisa é uma **implementação independente**.
>
> **Data:** 2026-07-20. **Fonte:** `rusefi/rusefi`, `firmware/hw_layer/drivers/gpio/tle8888.cpp`
> (1304 linhas), obtido de `raw.githubusercontent.com`.
>
> ⚠️ **O que isto prova e o que não prova.** Confirma que dois projetos independentes leram os mesmos
> **endereços** e o mesmo **formato de frame** — o que retira grande parte do risco de mapa inventado.
> **Não** substitui o clock-out em bancada: nada aqui prova que o nosso código fala com silício real.

---

## 1. Formato do frame — **coincide**

```c
/* C0 */
#define CMD_READ            (0 << 0)
#define CMD_WRITE           (1 << 0)
/* C7:1 */
#define CMD_REG_ADDR(a)     (((a) & 0x7f) << 1)
/* CD7:0 */
#define CMD_REG_DATA(d)     (((d) & 0xff) << 8)
```

→ `data[15:8]` · `addr[7:1]` · R/W no `bit 0`. **Idêntico** ao nosso `frame()` (`tle8888.cpp`).

Isto é significativo porque o driver antigo tinha o frame **invertido** — era um dos três erros que
obrigaram à reescrita.

---

## 2. Endereços — **27 registadores, zero divergências**

| rusEFI | Endereço | `tle8888_regs.h` | |
|---|---|---|---|
| `CMD_CMD0` | `0x01` | `CMD0` | ✅ |
| `CMD_WWDSERVICECMD` | `0x15` | `WWD_SERVICE_CMD` | ✅ |
| `CMD_FWDRESPCMD` | `0x16` | `FWD_RESP_CMD` | ✅ |
| `CMD_SR_CODE` | `0x1a` | `CMD_SR` | ✅ |
| `CMD_OE_SET` / `CMD_OE_CLR` | `0x1c` | `CMD_OE` | ✅ |
| `CMD_CHIP_UNLOCK` | `0x1e` | `CMD_LOCK` | ✅ |
| `REG_DIAG(n)` | `0x20+n` | `DIAG0` / `DIAG1` | ✅ |
| `CMD_VRSDIAG(n)` | `0x22+n` | `VRS_DIAG0` / `VRS_DIAG1` | ✅ |
| `CMD_COMDIAG` | `0x24` | `COM_DIAG` | ✅ |
| `CMD_OUTDIAG(n)` | `0x25+n` | `OUT_DIAG0..4` | ✅ |
| `CMD_PPOVDIAG` | `0x2a` | `PPOV_DIAG` | ✅ |
| `CMD_BRIDIAG(n)` | `0x2b+n` | `BRI_DIAG0` / `BRI_DIAG1` | ✅ |
| `CMD_IGNDIAG` | `0x2d` | `IGN_DIAG` | ✅ |
| `CMD_WDDIAG` | `0x2e` | `WD_DIAG` | ✅ |
| `REG_OPSTAT(n)` | `0x34+n` | `OP_STAT0` / `OP_STAT1` | ✅ |
| `REG_WWDSTAT` | `0x36` | `WWD_STAT` | ✅ |
| `REG_FWDSTAT(n)` | `0x37+n` | `FWD_STAT0` / `FWD_STAT1` | ✅ |
| `REG_TECSTAT` | `0x39` | `TEC_STAT` | ✅ |
| `CMD_OUTCONFIG(n)` | `0x40+n` | `OUT_CONFIG0..5` | ✅ |
| `CMD_BRICONFIG(n)` | `0x46+n` | `BRI_CONFIG0` / `BRI_CONFIG1` | ✅ |
| `CMD_IGNCONFIG` | `0x48` | `IGN_CONFIG` | ✅ |
| `CMD_VRSCONFIG(n)` | `0x49+n` | `VRS_CONFIG0/1/2` | ✅ |
| `CMD_OPCONFIG0` | `0x4e` | `OP_CONFIG0` | ✅ |
| `CMD_INCONFIG(n)` | `0x53+n` | `IN_CONFIG0..3` | ✅ |
| `CMD_DDCONFIG(n)` | `0x57+n` | `DD_CONFIG0..3` | ✅ |
| `CMD_OECONFIG(n)` | `0x5b+n` | `OE_CONFIG0..3` | ✅ |
| `CMD_CONT(n)` | `0x7b+n` | `CONT0..3` | ✅ |

**Sem uma única divergência.** Inclui os dois endereços de que o direct drive depende — `DDConfig`
(`0x57`) e `OEConfig` (`0x5b`) —, que são o coração da decisão de manter o scheduler intacto.

---

## 3. ⚠️ O que o rusEFI **NÃO** confirma

O rusEFI nunca toca nestes, logo o nosso endereço para eles continua a assentar numa leitura única:

`ComConfig0` (0x4f) · `ComConfig1` (0x50) · `WWDConfig0/1` (0x5f/0x60) · `FWDConfig` (0x61) ·
`TECConfig` (0x62) · `WdConfig0` (0x63) · `WdConfig1` (0x64)

🚨 **5 dos 7 registadores do nosso fingerprint estão nesta lista** (`ComConfig0`, `ComConfig1`,
`WdConfig0`, `WdConfig1`, `FWDConfig`). Em retrospetiva foi a escolha mais fraca possível — ver a
política revista em `interface_board_v1.md`.

⚠️ **E o rusEFI não confirma valor de reset nenhum.** Há um comentário próximo do watchdog:

```c
/* Looks like reset value is 113.6ms? 1.6ms * 0x47 */
#define FWD_PERIOD_MS       (20)
```

Está imediatamente antes de `FWD_PERIOD_MS`, portanto refere-se ao watchdog **funcional**, e **qual**
registador tem o valor `0x47` é ambíguo. **Não tratar como corroboração** do nosso `WdConfig0 = 0x47`.

---

## 4. Mecanismo de validação por eco — melhor que o nosso fingerprint

A resposta SPI do TLE8888 **devolve o endereço do registador** que está a responder. `spi_validate()`:

```c
uint8_t reg = getRegisterFromResponse(rx);
if ((last_reg != REG_INVALID) && (last_reg != reg)) {
    if (reg == REG_OPSTAT(0))       por_cnt++;    // após power-on reset
    else if (reg == REG_FWDSTAT(1)) wdr_cnt++;    // após watchdog reset
    else if (reg == REG_DIAG(0))    comfe_cnt++;  // frame de comunicação inválido (COMFE)
    need_init = true;
    return -1;
}
```

Comentários do próprio rusEFI:
- *"after power on reset: the address and the content of the status register OpStat0 is transmitted
  with the next SPI transmission"*
- *"after watchdog reset: ... FWDStat1 ... with the first SPI transmission after the low to high
  transition of RST"*
- *"after an invalid communication frame: ... Diag0 ... and the bit COMFE in ComDiag is set to 1"*
- *"during power on reset: SPI commands are ignored, SDO is always tristate"*
- *"during watchdog reset: SPI commands are ignored, SDO has the value of the status flag"*

`REG_INVALID = 0x00` é sentinela segura — `0x00` não é registador real (`Cmd0` é `0x01`).

**Porque é melhor:** prova link, formato de frame e round-trip de endereço **continuamente**, e **não
depende da nossa leitura do datasheet**. É também a **única** validação possível para registadores de
comando, que não são relíveis.

**O que o eco NÃO prova:** endereço↔semântica. O CI ecoa o endereço que descodificou, logo um endereço
errado-mas-válido ecoa limpo. Essa continua a ser a função do fingerprint — mas esta contraprova já a
cumpriu offline, com mais fiabilidade do que uma leitura em runtime.

---

## 5. Ordem do `chip_init()` — duas lacunas nossas

```
CMD_SR                   ← soft reset, ~3 ms de espera; depois last_reg = INVALID
CMD_CHIP_UNLOCK          ← PRIMEIRO
CMD_INCONFIG(0..3)
CMD_OUTCONFIG(0..5)
...
CMD_OE_SET               ← ÚLTIMO
```

✅ **Implementado (2026-07-20):** `configure()` emite `CMD_CHIP_UNLOCK` no início, escreve
`InConfig0–3` + DD/OE (INJ/IGN/VVT/bomba/fan), e `CMD_OE_SET` no fim — fire-and-forget nos
comandos, `write_verify` só nos registadores de armazenamento. Ver `src/hal/tle8888.cpp`.

⚠️ **Unlock, OE e SR são registadores de COMANDO, não de armazenamento** — o rusEFI emite-os
fire-and-forget e nunca os relê. Não podem passar pelo nosso `write_verify()`: a releitura falharia num
CI saudável e abortaria a init.

⚠️ **Ainda por implementar:** validação por eco de endereço (esta secção §4) e despromoção do
fingerprint a consultivo.

---

## 6. Diferença deliberada — leitura em 2 transacções

O rusEFI faz *pipeline* e colhe a resposta na operação seguinte (lê `rx[i + 1]`; usa um frame dummy
inicial no dump de registadores). O nosso `tle_read()` reemite o mesmo comando de leitura duas vezes e é
auto-contido.

**Ambos corretos** contra o protocolo "a resposta chega no frame seguinte". O nosso custa um frame extra
por leitura e ganha localidade — escolha adequada a um driver sem thread dedicada. **Não mudar.**

---

## 7. Watchdog — política divergente, deteção adotada

O rusEFI **serve** o window watchdog: `CMD_WWDSERVICECMD = CMD_W(0x15, 0x03)`, com o comentário
*"Window watchdog open WWDOWT window time = 12.8 mS - fixed value for TLE8888QK"*.

**Nós mantemos a política de não servir** — a v1 usa a variante **-2QK**, com watchdog desativado de
fábrica (ver `interface_board_v1.md`). Mas adotamos a **deteção** de watchdog reset via eco
(`FWDStat1`), porque é a evidência direta de que foi montado um **-1QK** por engano, ou de que o -2QK
não vem desativado como assumido — falha que de outro modo se manifestaria como "o motor morreu sem
razão".
