# TLE8888 LQFP-100 — pinout de package (fonte primária)

> **Fontes:** Infineon TLE8888-1QK Data Sheet **Rev. 1.2** (2017-02-10), §3 Pin Configuration  
> (PDF local de trabalho / download Infineon). Contraprova de nomes: símbolo KiCad  
> `rusefi/hardware/rusefi_lib/tle8888qk.lib` (microRusEFI).  
> Variantes **-1QK / -2QK / QK** partilham o mesmo package e pinout; diferem no watchdog.

**Tier A** para layout KiCad. O que muda por board é só a **fiação MCU ↔ estes pinos**.

---

## SPI single-ended (modo OpenEMS) — strapping obrigatório

Do datasheet (comunicação SPI / single-ended):

| Pino | Símbolo | Ligação SPI |
|------|---------|-------------|
| 3 | **CSN** | ← MCU CS (PB12) |
| 4 | **SDO** | → MCU MISO (PB14) |
| 5 | **SIP** | ← MCU MOSI (PB15) — *data in* |
| 7 | **FCLP** | ← MCU SCK (PB13) — *clock* |
| **6** | **SIN** | → **AGND** (select SPI mode) |
| **8** | **FCLN** | → **VDDIO** (select single-ended) |

⚠️ Sem SIN=AGND e FCLN=VDDIO o CI pode ficar em MSC/LVDS e o SPI “não responde”.  
VDDIO (pino 20) = rail 3,3 V do MCU (mesma lógica dos direct-drive INx).

---

## Mapa OpenEMS v1 (MCU → package)

| Função OpenEMS | MCU | TLE pino | Símbolo DS | Notas |
|----------------|-----|----------|------------|--------|
| SPI CS | PB12 | **3** | CSN | |
| SPI MISO | PB14 | **4** | SDO | push-pull em SPI |
| SPI MOSI | PB15 | **5** | SIP | |
| SPI SCK | PB13 | **7** | FCLP | |
| SPI mode | — | **6** | SIN | **→ AGND** |
| SPI mode | — | **8** | FCLN | **→ VDDIO** |
| INJEN | PE1 | **24** | INJEN | enable OUT1–4 |
| IGNEN | PE3 | **27** | IGNEN | enable IGN1–4 |
| INJ1–4 | PE0/2/4/6 | **28–31** | IN1–IN4 | DD fixo → OUT1–4 |
| IGN1–4 | PE9/11/13/15 | **32–35** | IN5–IN8 | DD fixo → IGN1–4 |
| Bomba | PE10 | **36** | IN9 | → OUT14 (InConfig) |
| Ventoinha | PE12 | **37** | IN10 | → OUT15 |
| VVT esc | PB6 | **38** | IN11 | → OUT5 |
| VVT adm | PB7 | **39** | IN12 | → OUT6 |
| CKP dig | PA0 | **21** | VROUT | push-pull digital |
| CKP+ | conector | **52** | VRIN1 | par diferencial |
| CKP− | conector | **51** | VRIN2 | |
| CAN RX | PB8 | **43** | CANRX | MCU ← CI |
| CAN TX | PB9 | **44** | CANTX | MCU → CI |
| CANH/L | conector | **46/47** | CANH/CANL | |
| +5V sens A/B | sensores | **9/10** | T5V1/T5V2 | trackers |
| Main relay | DNP v1 | **55** | MR | key-on na v1 |
| BAT | rail | **54** | BAT | + BATPA/B 87/90 |
| PGND | estrela | **25,50,75** + tab | PGND | |
| AGND | VSSA | **100** | AGND | |
| VDDIO | +3V3 | **20** | VDDIO | lógica I/O |

---

## Tabela completa LQFP-100 (datasheet §3.2)

| Pin | Símbolo | Dir | Função (resumo DS) |
|-----|---------|-----|---------------------|
| 1 | RST | I/O | Reset bidirecional |
| 2 | MON | I/O | Monitor bidirecional |
| 3 | CSN | IN | SPI/MSC chip select |
| 4 | SDO | OUT | SPI/MSC data out |
| 5 | SIP | IN | SPI data in (single-ended) / LVDS+ data |
| 6 | SIN | IN | SPI select (**AGND** em SPI) / LVDS− data |
| 7 | FCLP | IN | SPI clock / LVDS+ clock |
| 8 | FCLN | IN | SPI select (**VDDIO** em SPI) / LVDS− clock |
| 9 | T5V1 | OUT | Tracker 5 V sensores A |
| 10 | T5V2 | OUT | Tracker 5 V sensores B |
| 11 | V5V | OUT | 5 V main ECU |
| 12 | V6V | IN | Source pré-regulador externo |
| 13 | VG | OUT | Gate pré-regulador externo |
| 14–16 | OUT7A/B/C | OUT | LS 4,5 A — **ligar A+B+C juntos** |
| 17 | OUT20 | OUT | LS pequeno |
| 18 | OUT19 | OUT | LS pequeno |
| 19 | n.c. | — | aberto ou GND |
| 20 | VDDIO | SUP | Supply lógica I/O (3,3 V) |
| 21 | VROUT | OUT | Saída digital VR |
| 22 | LINTX | IN | LIN TX |
| 23 | LINRX | OUT | LIN RX |
| 24 | **INJEN** | IN | Enable injectores OUT1–4 |
| 25 | PGND | GND | Power ground |
| 26 | KOFFDO | OUT | Key-off delay |
| 27 | **IGNEN** | IN | Enable ignição IGN1–4 |
| 28–31 | IN1–IN4 | IN | DD injectores |
| 32–35 | IN5–IN8 | IN | DD ignição |
| 36–39 | IN9–IN12 | IN | DD mapeáveis (InConfig) |
| 40 | EOTEN | IN | Engine-off timer enable |
| 41 | V5VSTBY | OUT | 5 V standby |
| 42 | CANWKEN | IN | CAN wake enable |
| 43 | CANRX | OUT | CAN RX → MCU |
| 44 | CANTX | IN | CAN TX ← MCU |
| 45 | V5VCAN | SUP | 5 V CAN |
| 46 | CANH | I/O | Bus |
| 47 | CANL | I/O | Bus |
| 48 | WK | IN | Wake |
| 49 | KEY | IN | Key (+ supply path MR) |
| 50 | PGND | GND | |
| 51 | VRIN2 | IN | VR diferencial − |
| 52 | VRIN1 | IN | VR diferencial + |
| 53 | BATSTBY | SUP | Battery standby |
| 54 | BAT | SUP | Battery main |
| 55 | MR | OUT | Main relay low-side |
| 56–58 | OUT18–16 | OUT | LS relé |
| 59–60 | OUT1A/B | OUT | Injector 1 — **A+B juntos** |
| 61–62 | OUT2A/B | OUT | Injector 2 |
| 63–64 | OUT3A/B | OUT | Injector 3 |
| 65–66 | OUT4A/B | OUT | Injector 4 |
| 67 | OUT15 | OUT | LS (fan OpenEMS) |
| 68 | OUT14 | OUT | LS (pump OpenEMS) |
| 69–74 | DFB8–10 / OUT8–10 | | MOSFET on-board drivers |
| 75 | PGND | GND | |
| 76–81 | DFB11–13 / OUT11–13 | | MOSFET drivers |
| 82 | LINIO | I/O | LIN bus |
| 83–85 | OUT5A/B/C | OUT | LS 4,5 A VVT — **A+B+C juntos** |
| 86 | OUT24 | OUT | Half-bridge |
| 87 | BATPA | SUP | Bat half-bridges/CP — **ligar a BATPB** |
| 88–89 | OUT23–22 | OUT | Half-bridge |
| 90 | BATPB | SUP | com BATPA |
| 91 | OUT21 | OUT | Half-bridge |
| 92–94 | OUT6A/B/C | OUT | LS 4,5 A VVT 2 — **A+B+C juntos** |
| 95 | CP | OUT | Charge pump (cap externa) |
| 96–99 | IGN1–4 | OUT | Push-pull **gate IGBT** (20 mA) |
| 100 | AGND | GND | Signal ground |
| tab | PGND | GND | Exposed pad |

---

## Regras de layout (datasheet)

1. **OUT1A+OUT1B** (e 2/3/4) no mesmo nó, sem parasitagem entre A e B.  
2. **OUT5A+B+C**, **OUT6A+B+C**, **OUT7A+B+C** idem.  
3. **BATPA ↔ BATPB** sem parasitagem.  
4. **PGND pins + cooling tab** num pour de potência.  
5. **IGN1–4**: DS diz *“on- or off-board IGBT”* — **não** assume smart coil 5 V lógica sem medir V_OH / V_IH da bobina.

---

## Correção ao texto stale

Afirmações antigas “pino 24/27 não verificados” estão **obsoletas**: o datasheet e o símbolo rusEFI concordam.  
O que era inventado era o **mapa de registadores SPI**, não estes pinos de package.
