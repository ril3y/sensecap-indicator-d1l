/**
 * @file test_sensors.cpp
 * @brief RP2040 Sensor Tests for Tasks #13-17
 *
 * Tests sensor power control, AHT20, SGP40, SCD41, and buzzer
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>
#include <Adafruit_AHTX0.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include "sensecap_pins.h"

// Sensor objects
Adafruit_AHTX0 aht;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;

// Test state
static bool sensorsInitialized = false;

void setUp(void) {
    // Setup before each test
}

void tearDown(void) {
    // Cleanup after each test
}

// ============================================================================
// Task #13: Sensor Power Control Tests
// ============================================================================

/**
 * TEST_PWR_001: GP18 HIGH enables sensor power
 */
void test_power_enable(void) {
    pinMode(PIN_SENSOR_POWER, OUTPUT);
    digitalWrite(PIN_SENSOR_POWER, HIGH);
    delay(100);  // Let power stabilize

    // Verify by checking GPIO state
    int state = digitalRead(PIN_SENSOR_POWER);
    TEST_ASSERT_EQUAL_MESSAGE(HIGH, state, "GP18 not HIGH");

    Serial.println("[PASS] TEST_PWR_001: Sensor power enabled");
}

/**
 * TEST_PWR_002: GP18 LOW disables sensor power
 */
void test_power_disable(void) {
    pinMode(PIN_SENSOR_POWER, OUTPUT);
    digitalWrite(PIN_SENSOR_POWER, LOW);
    delay(100);

    int state = digitalRead(PIN_SENSOR_POWER);
    TEST_ASSERT_EQUAL_MESSAGE(LOW, state, "GP18 not LOW");

    // Re-enable for subsequent tests
    digitalWrite(PIN_SENSOR_POWER, HIGH);
    delay(100);

    Serial.println("[PASS] TEST_PWR_002: Sensor power disable/enable cycle");
}

/**
 * TEST_PWR_003: I2C scan with power ON
 */
void test_i2c_scan_power_on(void) {
    digitalWrite(PIN_SENSOR_POWER, HIGH);
    delay(200);

    Wire.begin();

    int devicesFound = 0;
    Serial.println("\n[INFO] I2C Scan (Power ON):");

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == 0x38) Serial.print(" (AHT20)");
            if (addr == 0x59) Serial.print(" (SGP40)");
            if (addr == 0x62) Serial.print(" (SCD41)");
            Serial.println();
            devicesFound++;
        }
    }

    Serial.printf("[INFO] Total devices found: %d\n", devicesFound);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(1, devicesFound, "No I2C devices found");

    Serial.println("[PASS] TEST_PWR_003: I2C devices detected");
}

// ============================================================================
// Task #14: AHT20 Temperature/Humidity Tests
// ============================================================================

/**
 * TEST_AHT20_001: Detect AHT20 at I2C 0x38
 */
void test_aht20_detect(void) {
    Wire.beginTransmission(0x38);
    uint8_t error = Wire.endTransmission();

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "AHT20 not found at 0x38");
    Serial.println("[PASS] TEST_AHT20_001: AHT20 detected");
}

/**
 * TEST_AHT20_002: Initialize AHT20
 */
void test_aht20_init(void) {
    bool success = aht.begin();
    TEST_ASSERT_TRUE_MESSAGE(success, "AHT20 init failed");

    Serial.println("[PASS] TEST_AHT20_002: AHT20 initialized");
}

/**
 * TEST_AHT20_003: Read temperature in valid range
 */
void test_aht20_temperature(void) {
    sensors_event_t humidity, temp;
    bool success = aht.getEvent(&humidity, &temp);

    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read AHT20");

    float temperature = temp.temperature;
    Serial.printf("[INFO] Temperature: %.2f °C\n", temperature);

    // Valid range: -40 to +85°C, reasonable room temp: 10-40°C
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(-40.0, temperature, "Temperature too low");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(85.0, temperature, "Temperature too high");

    Serial.println("[PASS] TEST_AHT20_003: Temperature in valid range");
}

/**
 * TEST_AHT20_004: Read humidity in valid range
 */
void test_aht20_humidity(void) {
    sensors_event_t humidity, temp;
    bool success = aht.getEvent(&humidity, &temp);

    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read AHT20");

    float rh = humidity.relative_humidity;
    Serial.printf("[INFO] Humidity: %.2f %%\n", rh);

    // Valid range: 0-100%
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0.0, rh, "Humidity too low");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(100.0, rh, "Humidity too high");

    Serial.println("[PASS] TEST_AHT20_004: Humidity in valid range");
}

// ============================================================================
// Task #15: SGP40 TVOC Tests
// ============================================================================

/**
 * TEST_SGP40_001: Detect SGP40 at I2C 0x59
 */
void test_sgp40_detect(void) {
    Wire.beginTransmission(0x59);
    uint8_t error = Wire.endTransmission();

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "SGP40 not found at 0x59");
    Serial.println("[PASS] TEST_SGP40_001: SGP40 detected");
}

/**
 * TEST_SGP40_002: Initialize SGP40 and read serial
 */
void test_sgp40_init(void) {
    sgp40.begin(Wire);

    uint16_t serialNumber[3];
    uint16_t error = sgp40.getSerialNumber(serialNumber, 3);

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "Failed to read SGP40 serial");

    Serial.printf("[INFO] SGP40 Serial: 0x%04X%04X%04X\n",
                  serialNumber[0], serialNumber[1], serialNumber[2]);

    Serial.println("[PASS] TEST_SGP40_002: SGP40 initialized");
}

/**
 * TEST_SGP40_003: Read raw VOC signal
 */
void test_sgp40_raw_signal(void) {
    uint16_t srawVoc;
    uint16_t defaultRh = 0x8000;  // 50% RH
    uint16_t defaultT = 0x6666;   // 25°C

    uint16_t error = sgp40.measureRawSignal(defaultRh, defaultT, srawVoc);

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "Failed to read SGP40 raw signal");

    Serial.printf("[INFO] SGP40 Raw VOC: %u\n", srawVoc);

    // Raw signal typically 0-65535, but valid range is usually 0-50000
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0, srawVoc, "Raw VOC invalid");

    Serial.println("[PASS] TEST_SGP40_003: SGP40 raw signal valid");
}

// ============================================================================
// Task #16: SCD41 CO2 Tests
// ============================================================================

/**
 * TEST_SCD41_001: Detect SCD41 at I2C 0x62
 */
void test_scd41_detect(void) {
    Wire.beginTransmission(0x62);
    uint8_t error = Wire.endTransmission();

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "SCD41 not found at 0x62");
    Serial.println("[PASS] TEST_SCD41_001: SCD41 detected");
}

/**
 * TEST_SCD41_002: Initialize SCD41
 */
void test_scd41_init(void) {
    scd4x.begin(Wire, 0x62);

    uint16_t error = scd4x.stopPeriodicMeasurement();
    // Ignore error - might not be measuring

    error = scd4x.startPeriodicMeasurement();
    TEST_ASSERT_EQUAL_MESSAGE(0, error, "SCD41 start measurement failed");

    Serial.println("[INFO] SCD41 measurement started (5s interval)");
    Serial.println("[PASS] TEST_SCD41_002: SCD41 initialized");
}

/**
 * TEST_SCD41_003: Read CO2 (requires 5s measurement cycle)
 */
void test_scd41_co2(void) {
    Serial.println("[INFO] Waiting for SCD41 measurement (5+ seconds)...");
    delay(5500);  // Wait for at least one measurement

    uint16_t co2;
    float temperature, humidity;
    uint16_t error = scd4x.readMeasurement(co2, temperature, humidity);

    if (error || co2 == 0) {
        Serial.println("[WARN] First read failed, waiting for next measurement...");
        delay(5500);
        error = scd4x.readMeasurement(co2, temperature, humidity);
    }

    TEST_ASSERT_EQUAL_MESSAGE(0, error, "SCD41 read failed");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, co2, "SCD41 CO2 is zero (invalid)");

    Serial.printf("[INFO] SCD41 CO2: %u ppm, Temp: %.1f°C, Hum: %.1f%%\n",
                  co2, temperature, humidity);

    // Valid CO2 range: 400-5000 ppm (outdoor to high indoor)
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(350, co2, "CO2 too low");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(5000, co2, "CO2 too high");

    Serial.println("[PASS] TEST_SCD41_003: SCD41 CO2 reading valid");
}

// ============================================================================
// Task #17: Buzzer Tests
// ============================================================================

/**
 * TEST_BUZZER_001: Generate 1kHz tone
 */
void test_buzzer_tone(void) {
    pinMode(PIN_BUZZER, OUTPUT);

    Serial.println("[INFO] Playing 1kHz tone for 500ms...");
    analogWrite(PIN_BUZZER, 127);
    delay(500);
    analogWrite(PIN_BUZZER, 0);

    Serial.println("[PASS] TEST_BUZZER_001: Tone generated (verify audibly)");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_BUZZER_002: Beep pattern
 */
void test_buzzer_pattern(void) {
    Serial.println("[INFO] Playing beep pattern...");

    for (int i = 0; i < 3; i++) {
        analogWrite(PIN_BUZZER, 127);
        delay(100);
        analogWrite(PIN_BUZZER, 0);
        delay(100);
    }

    Serial.println("[PASS] TEST_BUZZER_002: Pattern played (verify audibly)");
    TEST_ASSERT_TRUE(true);
}

// ============================================================================
// Main
// ============================================================================

void setup() {
    delay(2000);

    Serial.begin(115200);
    Serial.println("\n======================================");
    Serial.println("RP2040 Sensor Test Suite");
    Serial.println("Tasks #13, #14, #15, #16, #17");
    Serial.println("======================================\n");

    // Initialize I2C
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    UNITY_BEGIN();

    // Power Control Tests (Task #13)
    Serial.println("\n--- Task #13: Sensor Power Control ---");
    RUN_TEST(test_power_enable);
    RUN_TEST(test_power_disable);
    RUN_TEST(test_i2c_scan_power_on);

    // AHT20 Tests (Task #14)
    Serial.println("\n--- Task #14: AHT20 Sensor ---");
    RUN_TEST(test_aht20_detect);
    RUN_TEST(test_aht20_init);
    RUN_TEST(test_aht20_temperature);
    RUN_TEST(test_aht20_humidity);

    // SGP40 Tests (Task #15)
    Serial.println("\n--- Task #15: SGP40 Sensor ---");
    RUN_TEST(test_sgp40_detect);
    RUN_TEST(test_sgp40_init);
    RUN_TEST(test_sgp40_raw_signal);

    // SCD41 Tests (Task #16)
    Serial.println("\n--- Task #16: SCD41 Sensor ---");
    RUN_TEST(test_scd41_detect);
    RUN_TEST(test_scd41_init);
    RUN_TEST(test_scd41_co2);

    // Buzzer Tests (Task #17)
    Serial.println("\n--- Task #17: Buzzer ---");
    RUN_TEST(test_buzzer_tone);
    RUN_TEST(test_buzzer_pattern);

    UNITY_END();
}

void loop() {
    delay(1000);
}
