/*
 * esp32_uart_bridge.ino — ponte USB↔UART transparente (nanoESP32-C6)
 * ════════════════════════════════════════════════════════════════════
 * Usa o C6 como conversor USB-serial para flashar/controlar um ESP32
 * cujo USB (CP2102) morreu, via UART0 dele.
 *
 * Ligações (C6 → ESP32 alvo):
 *   GPIO5 (TX1) → RX0 do alvo
 *   GPIO4 (RX1) ← TX0 do alvo
 *   GND         — GND
 *   5V          → VIN/5V do alvo (alimenta o alvo, USB dele está morto)
 *
 * Flash do alvo (bootloader manual):
 *   1. Segurar BOOT (IO0) do alvo, pulsar EN/RST, soltar BOOT
 *   2. esptool --port /dev/ttyACM_do_C6 --baud 115200 \
 *        --before no_reset --after no_reset write_flash ...
 *
 * NOTA: GPIO4/5 desta placa NÃO funcionam como UART1 (testado — sem
 * saída mesmo com jumper); GPIO6/7 funcionam. GPIO16/17 = UART0 bloqueado.
 * Quirks nanoESP32-C6 (wuxx): USB CDC nativo (HWCDC) — compilar com
 * CDCOnBoot=cdc e usar setTxTimeoutMs(0) senão o TX bloqueia sem host.
 */

#define BRIDGE_BAUD 115200
#define PIN_RX1 6   // ← TX0 do alvo
#define PIN_TX1 7   // → RX0 do alvo

void setup() {
    Serial.begin(BRIDGE_BAUD);
    Serial.setTxTimeoutMs(0);   // HWCDC: não bloquear sem host
    Serial1.begin(BRIDGE_BAUD, SERIAL_8N1, PIN_RX1, PIN_TX1);
}

void loop() {
    // USB → UART (transparente — nada de heartbeat: lixo no RX0 do alvo
    // atrapalharia o sync do esptool e os comandos do estimulador)
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }
    // UART → USB
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
