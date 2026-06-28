"""
ckp_gen_micropython.py — Gerador CKP (60-2) + CMP para bancada OpenEMS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Plataforma : ESP32 com MicroPython (firmware >= 1.21)
Precisão   : ±50–200 µs (adequado para ≤ 1000 RPM)
             Para > 1000 RPM usar a versão Arduino C++ (esp32_ckp_gen.ino)

Ligações ao STM32H562:
  Pin CKP_PIN → PA0  (TIM5_CH1, input capture CKP)
  Pin CMP_PIN → PA1  (TIM5_CH2, input capture CMP)
  GND         → GND  (referência comum OBRIGATÓRIA)

Uso:
  # Copiar para o ESP32 via Thonny ou ampy
  # Iniciar no REPL:
  import ckp_gen_micropython as g
  g.start(rpm=500)            # arrancar a 500 RPM
  g.start(rpm=1000)           # mudar RPM (para geração actual primeiro)
  g.stop()                    # parar

  # Ou correr directamente:
  # mpremote run ckp_gen_micropython.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import time
from machine import Pin

# ── Configuração ──────────────────────────────────────────────────────────────
CKP_PIN_NUM  = 2     # GPIO ligado a PA0 do STM32
CMP_PIN_NUM  = 4     # GPIO ligado a PA1 do STM32
RPM_DEFAULT  = 500
CMP_TOOTH    = 5     # Dente (0-indexed) onde o pulso CMP ocorre
REAL_TEETH   = 58    # Dentes reais na roda 60-2
# ─────────────────────────────────────────────────────────────────────────────

_running = False


def _tooth_period_us(rpm: int) -> int:
    """Período de um dente em µs: 60 000 000 / (rpm × 60)."""
    return 60_000_000 // (rpm * 60)


def start(rpm: int = RPM_DEFAULT, cmp_tooth: int = CMP_TOOTH) -> None:
    """
    Inicia a geração do sinal CKP + CMP em modo bloqueante.

    Parâmetros
    ----------
    rpm       : RPM a simular (recomendado ≤ 1000 para MicroPython)
    cmp_tooth : dente (0..57) onde o pulso CMP ocorre na revolução 0
    """
    global _running
    _running = True

    ckp = Pin(CKP_PIN_NUM, Pin.OUT, value=0)
    cmp = Pin(CMP_PIN_NUM, Pin.OUT, value=0)

    T_us   = _tooth_period_us(rpm)
    h_us   = T_us // 2           # duração HIGH de um dente
    l_us   = T_us // 2           # duração LOW normal
    gap_us = T_us * 2            # LOW extra no gap (total LOW gap = h_us + gap_us)

    print(f"CKP: {rpm} RPM  T={T_us} µs  HIGH={h_us} µs  "
          f"LOW_normal={l_us} µs  LOW_gap={h_us + gap_us} µs")

    revolution = 0   # alterna 0/1 a cada rotação do virabrequim
    tooth      = 0   # 0..57

    try:
        while _running:
            # ── Rising edge ───────────────────────────────────────────
            ckp.value(1)

            # CMP: gerar no dente cmp_tooth da revolução 0
            if revolution == 0 and tooth == cmp_tooth:
                cmp.value(1)
                time.sleep_us(h_us)
                cmp.value(0)
            else:
                time.sleep_us(h_us)

            # ── Falling edge ──────────────────────────────────────────
            ckp.value(0)
            tooth += 1

            if tooth >= REAL_TEETH:
                # Fim da revolução: gap de 2 posições extra
                # LOW total = h_us + gap_us = 5T/2
                # → tempo RE(57) → RE(0_next) = T/2 + 5T/2 = 3T  ✓
                time.sleep_us(h_us + gap_us)
                tooth = 0
                revolution ^= 1
            else:
                time.sleep_us(l_us)

    except KeyboardInterrupt:
        pass
    finally:
        ckp.value(0)
        cmp.value(0)
        _running = False
        print("CKP parado.")


def stop() -> None:
    """Sinaliza paragem (eficaz no próximo ciclo)."""
    global _running
    _running = False


# ── Executar directamente se chamado como script ──────────────────────────────
if __name__ == "__main__":
    import sys
    rpm = int(sys.argv[1]) if len(sys.argv) > 1 else RPM_DEFAULT
    start(rpm=rpm)
