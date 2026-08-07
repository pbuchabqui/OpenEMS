# BOM v1 — candidatos (não fecha compra)

Critério do plano: desempenho / imunidade a ruído, não preço mínimo.  
**Part numbers a validar stock/preço antes de pedido.**

---

## Já fechados por decisão

| Função | PN / tipo | Notas |
|--------|-----------|-------|
| MCU module | WeAct **H562 VGT6** CoreBoard V1.0 | 38,62×69,10 mm |
| Power stage IC | **TLE8888-2QK** LQFP-100 | WD off; pinout `tle8888_pinout.md` |
| ETB H-bridge | **BTS7960** module @ 10 kHz | 3 pinos PWM+IN1+IN2 |
| Plug sinais | **776164-1** | 35-pos black |
| Plug potência | **770680-1** | 23-pos black |
| Header PCB 35 | **1-776180-1** (RA gold) | ou tin `776180-1` |
| Header PCB 23 | **1-770669-1** (RA gold) | ETB / corrente |
| Crimp | **770520-*** / **770854** | ver amp table |
| TVS bat | **SMBJ24CA** | plano |
| Fusível entrada | 30 A | plano |

---

## Candidatos (Tier C)

| Função | Candidato | Porquê / alternativa |
|--------|-----------|----------------------|
| Buck ≥500 kHz | **TPS54302** (ou similar automotive) | Plano rejeita LM2596; ripple baixo |
| LDO 3,3 V low-noise | **TPS7A20** / **ADP7118** / **LT1763** class | Rejeita AMS1117; high PSRR em f_sw do buck |
| USB isolator | **ADuM3160** / **ISOUSB211** | Full-speed CDC; + DC-DC isolado lado host |
| DC-DC isolado USB | **B0505S-1W** class ou **NME0505** | Corrente CDC; barreira sem pour de terra |
| P-MOSFET reverse | automotive logic-level P-FET | + fuse 30 A |
| ETB module | BTS7960B board (mercado) | Confirmar pinout RPWM/LPWM vs PWM+IN1+IN2 |
| Hall CMP | open-collector 5 V | polaridade page0[258] bit1 |
| VR CKP | sensor VR 60-2 | TLE VRIN1/2 directo |
| Injectors | high-Z only | OUT1–4 2,2 A class |
| Coils | **IGBT smart** ou bobina + IGBT | IGN = 20 mA gate drive **não** é 5 V TTL garantido |
| Caixa | Hammond 1455 / die-cast catálogo | Cabine; outline após aresta ≥125 mm |

---

## VREF+ (MCU) — ✅ reconfirmado

STM32H562 **LQFP100**: **VREF+ é pino separado** (ex. pin 21 no pinout DS; ver
`stm32h562vg.pdf` Fig. 8). Opção (a) do plano: ligar a **VDDA filtrado**.  
(Em packages onde VREF+ está double-bonded a VDDA, VREFBUF fica limitado — não é o
caso do LQFP100 alvo.)

---

## Ordem de compra sugerida (bring-up)

1. WeAct VGT6 + cabos header 2,54  
2. TLE8888-2QK + pasta/reflow ou proto adapter se houver  
3. AMPSEAL 35+23 plugs + headers RA gold + crimp kit  
4. BTS7960 module + dummy injectors/coils  
5. Buck + LDO + TVS + bulk  
6. Isolador USB + DC-DC isolado  
