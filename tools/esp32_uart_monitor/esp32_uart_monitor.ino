// ESP32 UART Monitor — reads STM32 PA9 (TX) on GPIO16 and prints to Serial
// Connect: STM32 PA9 → ESP32 GPIO16, GND → GND
// Open Serial Monitor at 115200 to see debug output

#define RX_PIN 16

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, -1);  // RX only
    Serial.println("\n[UART Monitor ready — listening on GPIO16]");
}

void loop() {
    while (Serial2.available()) {
        char c = Serial2.read();
        Serial.write(c);
    }
}
