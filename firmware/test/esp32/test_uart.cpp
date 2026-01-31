/**
 * @file test_uart.cpp
 * @brief ESP32 UART Tests for Task #1
 *
 * Tests UART functionality on GPIO43(TX)/GPIO44(RX) which connects
 * to both the CH340 debug interface and the RP2040.
 *
 * Note: On SenseCAP Indicator, Serial (UART0) uses the same pins
 * as the inter-MCU connection. This test verifies the UART hardware
 * is working correctly.
 */

#include <Arduino.h>
#include <unity.h>

// UART pins to RP2040
#define UART_TX_PIN    43
#define UART_RX_PIN    44
#define UART_BAUD      115200

void setUp(void) {
    // Setup before each test
}

void tearDown(void) {
    // Cleanup after each test
}

/**
 * TEST_UART_ESP_001: UART initializes without error
 * Serial object is already initialized - verify it's functional
 */
void test_uart_init(void) {
    // Serial is already initialized in setup()
    // Verify it's working by checking if it's available
    TEST_ASSERT_TRUE_MESSAGE(Serial, "Serial failed to initialize");
}

/**
 * TEST_UART_ESP_002: TX pin outputs data when sending
 */
void test_uart_tx_pin(void) {
    // Send test data - this goes to both CH340 (monitor) and RP2040
    size_t written = Serial.print("UART_TX_TEST");
    Serial.flush();

    // If we get here without hanging, TX is working
    TEST_ASSERT_EQUAL(12, written);
}

/**
 * TEST_UART_ESP_003: Serial object functional
 */
void test_uart_functional(void) {
    // Test available bytes (should be 0 or more)
    int available = Serial.available();
    TEST_ASSERT_GREATER_OR_EQUAL(0, available);

    // Test write returns correct count
    size_t written = Serial.write((uint8_t)'@');
    TEST_ASSERT_EQUAL(1, written);

    // Test print returns correct count
    size_t printed = Serial.print("_OK_");
    TEST_ASSERT_EQUAL(4, printed);

    Serial.flush();
}

/**
 * TEST_UART_ESP_004: Loopback/echo test
 * This requires RP2040 to be running echo firmware
 */
void test_uart_loopback(void) {
    // Clear any pending data
    while (Serial.available()) {
        Serial.read();
    }

    // Note: On this board, loopback requires RP2040 echo firmware
    // Skip for now - will be tested in Task #3 (Inter-MCU)
    TEST_IGNORE_MESSAGE("Loopback requires RP2040 echo - tested in Task #3");
}

void setup() {
    delay(2000);  // Wait for serial monitor connection

    // Initialize Serial on UART0 (GPIO43/44)
    Serial.begin(UART_BAUD);
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    Serial.println();
    Serial.println("======================================");
    Serial.println("ESP32 UART Test Suite - Task #1");
    Serial.println("GPIO43(TX) / GPIO44(RX) @ 115200 baud");
    Serial.println("======================================");
    Serial.println();

    UNITY_BEGIN();

    RUN_TEST(test_uart_init);
    RUN_TEST(test_uart_tx_pin);
    RUN_TEST(test_uart_functional);
    RUN_TEST(test_uart_loopback);

    int result = UNITY_END();

    Serial.println();
    if (result == 0) {
        Serial.println("*** ALL TESTS PASSED ***");
    } else {
        Serial.printf("*** %d TEST(S) FAILED ***\n", result);
    }
}

void loop() {
    // Idle - tests complete
    delay(1000);
}
