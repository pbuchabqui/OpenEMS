# Condicionamento de entrada VR (CKP/CMP) — MAX9926

> **⚠️ SUPERSEDIDO para o plano v1 — ver `docs/hw/interface_board_v1.md`.**
> O CKP passou a usar a **interface VR integrada do TLE8888-2QK**, que tem a
> mesma arquitectura do MAX9924 modo A2 (comutação no **zero-crossing**, armada
> por detecção de pico) e ainda traz **clamp de entrada integrado** (50 mA,
> 2–3 V) e **diagnóstico do sensor** (short-to-GND, short-to-BAT, open-load e
> leitura ADC da tensão de entrada) — coisas que o MAX9924 não tem. O `VROUT` é
> push-pull, portanto sem pull-up externo.
> O **CMP é Hall** e vai direto ao `PA1`, sem condicionador (o TLE8888 tem um só
> canal VR; o "modo Hall" é modo *daquele* canal, não um segundo canal).
> Este documento fica como referência caso se opte por condicionador externo.

Estado atual: PA0 (TIM5_CH1/CKP) e PA1 (TIM5_CH2/CMP) recebem sinal **digital**
direto (estimulador ESP32 / sensor Hall), com pull-down interno e filtro IC
N8/DTS8 ≈256 ns (`src/hal/stm32h562/timer.cpp:41-64`). Isto NÃO serve para
sensor de relutância variável (VR/indutivo) real: a saída de um VR é uma
senoide diferencial cuja amplitude varia de ~centenas de mV (cranking) a
dezenas de volts (alta rotação) — ligada direto ao pino, ou não cruza VIH ou
destrói o MCU.

## Solução de referência: MAX9926 (Analog Devices / ex-Maxim)

Família MAX9924–MAX9927: interface VR→digital com **entrada diferencial**,
**adaptive peak threshold** e detecção por **zero-crossing** — pulso de saída
limpo mesmo com sinal fraco ou ruído forte. O MAX9926 é a versão **dual**
(CKP + CMP num só chip), QSOP-16, faixa automotiva -40..+125 °C.
(Ideia validada ao analisar o Route-ECU-Firmware/FSAE, que usa o single
MAX9924 para o mesmo fim.)

Recomendação para a fase de veículo real com sensores VR:

- **1× MAX9926U** — canal A = CKP roda 60-2, canal B = CMP.
- **Modo A1** (adaptive peak threshold + zero-crossing, bias externo): o
  threshold segue a amplitude do sinal, imune a ruído de baixa amplitude no
  cranking e a picos em alta RPM.
- Saída push-pull do chip → direto em PA0/PA1. **Manter** o pull-down interno
  e o filtro IC atuais como segunda camada anti-ruído (a defesa anti
  falso-sync com sensor desligado continua válida).
- Entradas do chip: par diferencial do sensor VR (par trançado), resistores
  série + clamp conforme datasheet.
- Se o sensor do veículo for **Hall** (saída digital open-collector), o
  MAX9926 é desnecessário — basta pull-up externo adequado; o pull-down
  interno atual deve então ser revisto (conflita com open-collector).

Datasheet: https://www.analog.com/MAX9926/datasheet

## Firmware

Nenhuma mudança: bordas de subida em PA0/PA1 como hoje. A decisão VR vs Hall
afeta apenas BOM/chicote e, no caso Hall, a configuração de pull do GPIO.
