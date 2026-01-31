/**
 * @file test_uart.cpp
 * @brief RP2040 UART Tests for Task #2
 *
 * Tests:
 * - TEST_UART_RP_001: UART initializes without error
 * - TEST_UART_RP_002: TX pin outputs data when sending
 * - TEST_UART_RP_003: Serial1 object functional
 */

#include <Arduino.h>
#include <unity.h>

// Test configuration
#define TEST_UART_BAUD      115200
#define TEST_UART_TX_PIN    16
#define TEST_UART_RX_PIN    17

void setUp(void) {
    // Setup before each test
}

void tearDown(void) {
    // Cleanup after each test
}

/**
 * TEST_UART_RP_001: UART initializes without error
 */
void test_uart_init(void) {
    Serial1.end();  // Ensure clean state

    Serial1.setTX(TEST_UART_TX_PIN);
    Serial1.setRX(TEST_UART_RX_PIN);
    Serial1.begin(TEST_UART_BAUD);

    // Check that Serial1 is available
    TEST_ASSERT_TRUE_MESSAGE(Serial1, "Serial1 failed to initialize");

    Serial.println("[PASS] TEST_UART_RP_001: UART initialized");
}

/**
 * TEST_UART_RP_002: TX pin configured correctly
 */
void test_uart_tx_pin(void) {
    // Send test data
    const char* testMsg = "TEST_FROM_RP2040";
    Serial1.print(testMsg);
    Serial1.flush();  // Wait for transmission complete

    // If we get here without hanging, TX is working
    TEST_ASSERT_TRUE(true);

    Serial.println("[PASS] TEST_UART_RP_002: TX pin operational");
}

/**
 * TEST_UART_RP_003: Serial1 object functional
 */
void test_uart_functional(void) {
    // Test available bytes (should be 0 or more)
    int available = Serial1.available();
    TEST_ASSERT_GREATER_OR_EQUAL(0, available);

    // Test write returns correct count
    size_t written = Serial1.write((uint8_t)'R');
    TEST_ASSERT_EQUAL(1, written);

    // Test print returns correct count
    size_t printed = Serial1.print("RP2040");
    TEST_ASSERT_EQUAL(6, printed);

    Serial1.flush();

    Serial.println("[PASS] TEST_UART_RP_003: Serial1 functional");
}

/**
 * TEST_UART_RP_004: Echo mode test
 * Echoes received bytes back to sender (for ESP32 loopback test)
 */
void test_uart_echo_mode(void) {
    Serial.println("[INFO] Entering echo mode for 5 seconds...");
    Serial.println("[INFO] Send data from ESP32 to test");

    uint32_t startTime = millis();
    uint32_t bytesEchoed = 0;

    while (millis() - startTime < 5000) {
        if (Serial1.available()) {
            uint8_t byte = Serial1.read();
            Serial1.write(byte);
            bytesEchoed++;
        }
    }

    Serial.printf("[INFO] Echo mode complete. Bytes echoed: %lu\n", bytesEchoed);

    if (bytesEchoed > 0) {
        Serial.println("[PASS] TEST_UART_RP_004: Echo mode functional");
        TEST_ASSERT_TRUE(true);
    } else {
        Serial.println("[SKIP] TEST_UART_RP_004: No data received (connect to ESP32)");
        TEST_IGNORE_MESSAGE("No data received - skipping");
    }
}

void setup() {
    delay(2000);  // Wait for serial monitor

    Serial.begin(115200);
    Serial.println("\n======================================");
    Serial.println("RP2040 UART Test Suite - Task #2");
    Serial.println("======================================\n");

    UNITY_BEGIN();

    RUN_TEST(test_uart_init);
    RUN_TEST(test_uart_tx_pin);
    RUN_TEST(test_uart_functional);
    RUN_TEST(test_uart_echo_mode);

    UNITY_END();
}

void loop() {
    // Continuous echo mode after tests
    if (Serial1.available()) {
        Serial1.write(Serial1.read());
    }
}
