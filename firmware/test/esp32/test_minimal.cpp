/**
 * @file test_minimal.cpp
 * @brief Minimal ESP32-S3 test - just verify basic operation
 */

#include <Arduino.h>

void setup() {
    // Wait for USB CDC to be ready
    delay(3000);

    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        delay(100);
    }

    Serial.println();
    Serial.println("======================================");
    Serial.println("ESP32-S3 Minimal Test");
    Serial.println("======================================");
    Serial.println();
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
    Serial.println();
    Serial.println("[PASS] Basic operation working!");
}

void loop() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 2000) {
        lastPrint = millis();
        Serial.printf("Uptime: %lu ms, Free Heap: %d\n", millis(), ESP.getFreeHeap());
    }
}
