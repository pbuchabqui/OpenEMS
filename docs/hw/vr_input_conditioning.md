# Condicionamento de entrada VR (CKP/CMP) — MAX9926

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
