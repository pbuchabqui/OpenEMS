# STM32H562 — Referência de Hardware (OpenEMS)

Extrato curado do RM0481 (Rev 4) e CMSIS stm32h562xx.h.
Contém apenas o que é relevante para o projeto OpenEMS.
Fonte autoritativa: RM0481 + stm32h562xx.h (WeAct STM32H5 SDK).

---

## 1. Mapa de Memória — Endereços Base

```
Região                  Base          Notas
─────────────────────────────────────────────────────────────
FLASH Bank1 (code)      0x0800_0000   até 1 MB (512 KB usado)
FLASH Bank2 (NVM)       0x0810_0000   8 KB setores, NVM adaptativa
FLASH controller        0x4002_2000   registradores FLASH_ACR, SR, CR
SRAM1 (128 KB)          0x2000_0000
SRAM2 (80 KB)           0x2002_0000   .fastrun, vector table RAM
SRAM3 (64 KB)           0x2003_4000

APB1 base               0x4000_0000
APB2 base               0x4001_0000
AHB1 base               0x4002_0000
AHB2 base               0x4202_0000
AHB3 base               0x4402_0000

Periférico              Endereço base   Bus
────────────────────────────────────────────
TIM2                    0x4000_0000     APB1
TIM3                    0x4000_0400     APB1
TIM4                    0x4000_0800     APB1
TIM5                    0x4000_0C00     APB1
TIM6                    0x4000_1000     APB1 (ADC trigger)
TIM7                    0x4000_1400     APB1
IWDG                    0x4000_3000     APB1
FDCAN1                  0x4000_A400     APB1
TIM1                    0x4001_2C00     APB2 (ETB PWM)
TIM8                    0x4001_3400     APB2 (ignição OC)
USART1                  0x4001_3800     APB2 (UI UART)
GPDMA1                  0x4002_0000     AHB1
GPDMA1_CH0              0x4002_0050     +0x80 por canal (CH0..CH7)
GPDMA1_CH1              0x4002_00D0
GPIOA                   0x4202_0000     AHB2
GPIOB                   0x4202_0400     AHB2
GPIOC                   0x4202_0800     AHB2
ADC1                    0x4202_8000     AHB2
ADC2                    0x4202_8100     AHB2 (+0x100 do ADC1)
ADC12_CCR               0x4202_8300     Registrador comum ADC1/ADC2
RCC                     0x4402_0C00     AHB3
NVIC                    0xE000_E100     Core (NVIC_ISER0..NVIC_IPR0)
SCB (VTOR)              0xE000_ED08     Vector table offset
```

---

## 2. IRQ Numbers — Tabela Completa STM32H562

Fonte: stm32h562xx.h IRQn_Type + RM0481 Table 146.
Vetor = endereço 0x40 + IRQ×4 na tabela em SRAM.

```
IRQ   Periférico              IRQ   Periférico
────────────────────────────────────────────────────────
  0   WWDG                     66   TIM8_UP
  1   PVD_AVD                  67   TIM8_TRG_COM / DIR / IDX
  2   RTC                      68   TIM8_CC               ← ignição CC
  3   RTC_S                    69   ADC2                  ← usado
  4   TAMP                     70   LPTIM2
  5   RAMCFG                   71   TIM15
  6   FLASH                    72   TIM16
  7   FLASH_S                  73   TIM17
  8   GTZC                     74   USB_DRD_FS
  9   RCC                      75   CRS
 10   RCC_S                    76   UCPD1
 11   EXTI0                    77   FMC
 12   EXTI1                    78   OCTOSPI1
 13   EXTI2                    79   SDMMC1
 14   EXTI3                    80   I2C3_EV
 15   EXTI4                    81   I2C3_ER
 16   EXTI5                    82   SPI4
 17   EXTI6                    83   SPI5
 18   EXTI7                    84   SPI6
 19   EXTI8                    85   USART6
 20   EXTI9                    86   USART10
 21   EXTI10                   87   USART11
 22   EXTI11                   88   SAI1
 23   EXTI12                   89   SAI2
 24   EXTI13                   90   GPDMA2_CH0
 25   EXTI14                   91   GPDMA2_CH1
 26   EXTI15                   92   GPDMA2_CH2
 27   GPDMA1_CH0    ← usado    93   GPDMA2_CH3
 28   GPDMA1_CH1    ← usado    94   GPDMA2_CH4
 29   GPDMA1_CH2                95   GPDMA2_CH5
 30   GPDMA1_CH3                96   GPDMA2_CH6
 31   GPDMA1_CH4                97   GPDMA2_CH7
 32   GPDMA1_CH5                98   UART7
 33   GPDMA1_CH6                99   UART8
 34   GPDMA1_CH7               100   UART9
 35   IWDG                     101   UART12
 36   SAES                     102   SDMMC2
 37   ADC1          ← usado    103   FPU
 38   DAC1                     104   ICACHE
 39   FDCAN1_IT0    ← usado    105   DCACHE1
 40   FDCAN1_IT1               108   DCMI_PSSI
 41   TIM1_BRK                 109   FDCAN2_IT0
 42   TIM1_UP                  110   FDCAN2_IT1
 43   TIM1_TRG_COM             111   CORDIC
 44   TIM1_CC                  112   FMAC
 45   TIM2          ← usado    113   DTS
 46   TIM3                     114   RNG
 47   TIM4                     115   OTFDEC1
 48   TIM5          ← usado    116   AES
 49   TIM6                     117   HASH
 50   TIM7                     118   PKA
 51   I2C1_EV                  119   CEC
 52   I2C1_ER                  120   TIM12
 53   I2C2_EV                  121   TIM13
 54   I2C2_ER                  122   TIM14
 55   SPI1                     123   I3C1_EV
 56   SPI2                     124   I3C1_ER
 57   SPI3                     125   I2C4_EV
 58   USART1        ← usado    126   I2C4_ER
 59   USART2                   127   LPTIM3
 60   USART3                   128   LPTIM4
 61   UART4                    129   LPTIM5
 62   UART5                    130   LPTIM6
 63   LPUART1
 64   LPTIM1        ← NÃO COMP (H562 sem COMP)
 65   TIM8_BRK      ← ignição break
```

**Nota crítica:** STM32H562 **não possui** periférico COMP (comparador analógico).
Não existe COMP1_IRQn neste device. O knock usa ADC1_IN6 (PA5) com threshold software.

---

## 3. Alternate Function — Pinos do Projeto

Fonte: STM32CubeMX .ioc examples + GPIO_AF defines em stm32h5xx_hal_gpio_ex.h.
Registradores: `GPIOx_AFRL` (pinos 0-7, 4 bits/pino) e `GPIOx_AFRH` (pinos 8-15).

```
Pino   AF#   Função           Uso no OpenEMS
──────────────────────────────────────────────────────────────
PA0    AF2   TIM5_CH1         CKP input capture
PA1    AF2   TIM5_CH2         CMP (cam) input capture
PA2    ANALOG ADC1_IN3        MAP sensor
PA3    ANALOG ADC1_IN4        MAF sensor
PA4    ANALOG ADC1_IN5        TPS sensor
PA5    ANALOG ADC1_IN6        KNOCK sensor (ADC threshold SW)
PA6    AF2   TIM3_CH1         AUX PWM 1 (IACV/wastegate)
PA7    AF1   TIM1_CH1N        TIM1 complementar (não usado no projeto)
PA8    AF1   TIM1_CH1         ETB PWM (20 kHz)
PA9    AF7   USART1_TX        UI UART TX — NÃO reconfigurar como TIM1
PA10   AF7   USART1_RX        UI UART RX
PA15   AF1   TIM2_CH1         INJ1 output compare

PB0    ANALOG ADC1_IN7        APP1 (pedal 1)
PB1    ANALOG ADC1_IN8        APP2 (pedal 2)
PB3    AF1   TIM2_CH2         INJ2 output compare
PB6    AF2   TIM4_CH1         AUX PWM 3 (VVT)
PB7    AF2   TIM4_CH2         AUX PWM 4 (VVT)
PB8    AF9   FDCAN1_RX        WBO2 CAN RX
PB9    AF9   FDCAN1_TX        WBO2 CAN TX
PB10   AF1   TIM2_CH3         INJ3 output compare
PB11   AF1   TIM2_CH4         INJ4 output compare
PB14   —     GPIO_Output      ETB DIR (H-bridge direction)
PB15   —     GPIO_Output      ETB EN  (H-bridge enable)

PC0    ANALOG ADC1_IN9        ETB_TPS1
PC1    ANALOG ADC1_IN10       ETB_TPS2
PC2    ANALOG ADC2_IN1 (*)    CLT sensor
PC3    ANALOG ADC2_IN2 (*)    IAT sensor
PC4    ANALOG ADC2_IN13(*)    FUEL_PRESS sensor
PC5    ANALOG ADC2_IN14(*)    OIL_PRESS sensor
PC6    AF3   TIM8_CH1         IGN1 output compare
PC7    AF3   TIM8_CH2         IGN2 output compare
PC8    AF3   TIM8_CH3         IGN3 output compare (⚠ conflito microSD WeAct)
PC9    AF3   TIM8_CH4         IGN4 output compare (⚠ conflito microSD WeAct)

(*) ADC2 IN numbers per RM0481 §25 — PC2=IN3, PC3=IN4,
    PC4=IN13, PC5=IN14 (ver adc.cpp comentários)
```

**AF summary:**
- AF1 = TIM1, TIM2
- AF2 = TIM3, TIM4, TIM5, TIM8
- AF3 = TIM8 (PC6-PC9)
- AF7 = USART1 (PA9/PA10)
- AF9 = FDCAN1 (PB8/PB9)
- ANALOG = sem AF, MODER=11

---

## 4. Offsets de Registradores

### 4.1 TIMx — General Purpose (TIM2/3/4/5) e Advanced (TIM1/TIM8)

```
Offset  Reg       Notas
0x00    CR1       CEN=bit0, ARPE=bit7, OPM=bit3, URS=bit2
0x04    CR2       MMS[4:2] para TIM6 TRGO=010
0x08    SMCR
0x0C    DIER      UIE=bit0, CCxIE=bits1-4
0x10    SR        UIF=bit0, CC1IF..CC4IF=bits1-4
0x14    EGR       UG=bit0 (força update)
0x18    CCMR1     CC1S[1:0], OC1M[3:0]+bit16, OC1PE=bit3
0x1C    CCMR2     CC3S[1:0], OC3M, OC3PE / CC4
0x20    CCER      CC1E=bit0, CC1NE=bit2, CC1P=bit1, CC1NP=bit3
0x24    CNT       contador (32-bit TIM2/5; 16-bit TIM1/3/4/8)
0x28    PSC       prescaler
0x2C    ARR       auto-reload
0x30    RCR       repetition counter (TIM1/TIM8 apenas)
0x34    CCR1      capture/compare 1
0x38    CCR2
0x3C    CCR3
0x40    CCR4
0x44    BDTR      dead-time (TIM1/TIM8): MOE=bit15, DTG[7:0]
0x48    DCR
0x4C    DMAR
```

**OC mode bits (CCMR OC1M[3:0] + bit16):**
- `0110` = PWM mode 1 (alto enquanto CNT < CCR)
- `0111` = PWM mode 2
- `0001` = Active on match (used for INJ/IGN OC events)

**CCER CC1NE:** habilita saída complementar CH1N — só TIM1/TIM8. TIM1_CH1N disponível em PA7 (AF1) ou PB13 (AF1), **não em PA9**.

### 4.2 ADC1/ADC2

```
Offset  Reg       Notas
0x00    ISR       ADRDY=bit0, EOC=bit2, EOS=bit3, ADCAL=bit11
0x04    IER       interrupção enables
0x08    CR        ADEN=bit0, ADDIS=bit1, ADSTART=bit2, ADSTP=bit4,
                  ADCAL=bit31, ADCALDIF=bit30, ADVREGEN=bit28, DEEPPWD=bit29
0x0C    CFGR1     RES[3:2], DMAEN=bit0, DMACFG=bit1,
                  EXTSEL[4:0]=bits5-9, EXTEN[1:0]=bits10-11
0x14    SMPR1     SMP0..SMP9 (3 bits cada, canais IN0-IN9)
0x18    SMPR2     SMP10..SMP18 (canais IN10-IN18; IN13=bits9-11, IN14=bits12-14)
0x30    SQR1      L[3:0]=bits0-3 (nconv-1), SQ1[4:0]=bits6-10, SQ2..SQ4
0x34    SQR2      SQ5..SQ8
0x38    SQR3      SQ9..SQ12
0x40    DR        data register (leitura 16-bit)
0x300   CCR       (ADC12 comum) CKMODE[1:0]=bits16-17
```

**EXTSEL para TIM6_TRGO:** `0b01101` = 13 (campo CFGR1[9:5]).
**EXTEN rising edge:** `0b01` (campo CFGR1[11:10]).
**CKMODE HCLK/4:** `0b11` (campo ADC12_CCR[17:16]).
**SMP 47.5 ciclos:** `0b011` (3 bits por canal).

### 4.3 GPDMA1 — Canal (base_canal = 0x4002_0050 + 0x80 × ch)

```
Offset  Reg       Notas
0x10    CSR       IDLEF=bit0, TCF=bit8, DTEF=bit11, USEF=bit18
0x14    CCR       EN=bit0, RESET=bit1, PRIO[1:0]=bits22-23,
                  TCIE=bit8, DTEIE=bit11, USEIE=bit18
0x40    CTR1      DDW[1:0], SDW[1:0], DINC=bit19, SINC=bit3
0x44    CTR2      REQSEL[7:0]=bits0-7, TCEM[1:0]=bits30-31
0x48    CBR1      BNDT[15:0]=bytes a transferir
0x4C    CSAR      endereço fonte
0x50    CDAR      endereço destino
0x5C    CLLR      linked-list next (0 = fim)
0x04    CFCR      clear flags (escrever 0xFFFFFFFF para limpar tudo)
```

**REQSEL ADC1:** `0x00` | `ADC1 req = 0` — ver RM0481 Table 136 para ID exato.
**REQSEL ADC2:** verificar na tabela.
**TCEM block:** `0b11` = TC ao fim do bloco.

### 4.4 RCC — Enables de Clock

```
Registrador     Offset    Bits relevantes para o projeto
RCC_AHB1ENR     0x88      bit0=GPDMA1EN
RCC_AHB2ENR1    0x8C      bit0=GPIOAEN, bit1=GPIOBEN, bit2=GPIOCEN,
                           bit10=ADC12EN
RCC_APB1LENR    0xB4      bit0=TIM2EN, bit1=TIM3EN, bit2=TIM4EN,
                           bit3=TIM5EN, bit4=TIM6EN, bit5=TIM7EN,
                           bit9=FDCAN1EN, bit11=USART2EN, bit17=UART4EN,
                           bit18=UART5EN, bit23=IWDGEN
RCC_APB2ENR     0xC4      bit11=TIM1EN, bit13=TIM8EN, bit14=USART1EN
```

### 4.5 FLASH Bank2 — Controle NVM

```
Registrador     Endereço        Notas
FLASH_CR2       0x4002_2124     PG=bit1, SER=bit2, SNB[3:0]=bits6-9, STRT=bit16
FLASH_SR2       0x4002_211C     BSY=bit0, WBNE=bit1, DBNE=bit3, errors=bits17-21
FLASH_CCR2      0x4002_2120     clear status (escrever 1 nos bits de erro)
FLASH_KEYR2     0x4002_2118     unlock: escrever 0x45670123 depois 0xCDEF89AB
FLASH_OPTKEYR   0x4002_210C     unlock opções: 0x08192A3B depois 0x4C5D6E7F
```

**Errata ES0565:** Primeira operação erase/program após reset pode congelar CPU por ~120 µs. Executar a primeira escrita antes de habilitar CKP ISR, ou usar código em SRAM. Endurance: Bank2 ~10k ciclos em Rev X.

---

## 5. Notas de Periféricos Ausentes no STM32H562

| Periférico | Presente? | Alternativa usada no projeto |
|---|---|---|
| COMP1/COMP2 (comparador analógico) | **NÃO** | ADC1_IN6 (PA5) + threshold SW para knock |
| OPAMP | NÃO | — |
| DAC2 | NÃO | — |
| Ethernet | NÃO | — |
| DCMI | NÃO | — |
| SDMMC2 | NÃO | — |

**Confirmar no chip físico:** ler `DBGMCU_IDCODE` (0xE004_4000) campo `REV_ID[31:16]`.
- Rev A = 0x1000, Rev Z = 0x1001, Rev X = 0x2001, Rev W = 0x2003.
- Rev X recomendada para produção (errata ES0565 Rev 8).

---

## 6. Clock Tree (250 MHz configuração do projeto)

```
HSE (8 MHz WeAct board)
  └─ PLL1 (M=1, N=125, P=4) → SYSCLK = 250 MHz
       ├─ AHB  /1 → HCLK   = 250 MHz
       ├─ APB1 /2 → PCLK1  = 125 MHz → TIM2/3/4/5/6/7 × 2 = 250 MHz
       ├─ APB2 /2 → PCLK2  = 125 MHz → TIM1/TIM8 × 2 = 250 MHz
       └─ ADC CKMODE = HCLK/4 → ADC clock = 62.5 MHz
```

TIM5 input capture @ 62.5 MHz → 1 tick = 16 ns (usado no CKP decoder).
TIM2/TIM8 scheduler @ 10 MHz (PSC=24 para 250 MHz/25 = 10 MHz, 100 ns/tick).
