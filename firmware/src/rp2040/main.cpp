/**
 * @file main.cpp
 * @brief SenseCAP Indicator RP2040 Sensor Hub Firmware
 *
 * Handles:
 * - Environmental sensors (AHT20, SGP40, SCD41)
 * - SD card logging
 * - Buzzer control
 * - Grove ADC inputs
 * - Communication with ESP32-S3
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_AHTX0.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include <VOCGasIndexAlgorithm.h>

#include "sensecap_pins.h"
#include "sensecap_protocol.h"
#include "sensecap_comm.h"

// ============================================================================
// Configuration
// ============================================================================

#define SENSOR_POLL_INTERVAL_MS     5000    // Default 5 second polling
#define HEARTBEAT_INTERVAL_MS       10000   // Heartbeat every 10 seconds
#define DEBUG_SERIAL                1       // Enable debug output on USB Serial

// ============================================================================
// Global Objects
// ============================================================================

Adafruit_AHTX0 aht;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm vocAlgorithm;

// State
static bool sensorsInitialized = false;
static bool sdCardInitialized = false;
static bool sensorPowerOn = false;
static uint32_t pollInterval = SENSOR_POLL_INTERVAL_MS;
static uint32_t lastPollTime = 0;
static uint32_t lastHeartbeatTime = 0;
static uint8_t statusFlags = 0;

// Sensor data
static SensorData currentData;

// ============================================================================
// Forward Declarations
// ============================================================================

void initSensorPower();
void initI2C();
void initSensors();
void initSDCard();
void initBuzzer();
void readAllSensors();
void handlePacket(const uint8_t *buffer, size_t size);
void playBeep(uint16_t duration_ms);
void playBeepPattern(uint8_t pattern);
void logToSD(const SensorData &data);

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Initialize USB Serial for debug
#if DEBUG_SERIAL
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("SenseCAP Indicator RP2040 Sensor Hub");
    Serial.printf("Firmware v%d.%d.%d\n", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    Serial.println("========================================\n");
#endif

    // Initialize UART to ESP32
    Serial1.setTX(PIN_UART_TX);
    Serial1.setRX(PIN_UART_RX);
    sensecapComm.begin(Serial1, UART_BAUD_RATE);
    sensecapComm.onPacketReceived(handlePacket);

#if DEBUG_SERIAL
    Serial.println("[UART] ESP32 communication initialized");
#endif

    // Initialize hardware
    initSensorPower();
    initI2C();
    initSensors();
    initSDCard();
    initBuzzer();

    // Startup beep
    playBeep(100);

    // Send version info to ESP32
    sensecapComm.sendVersion();

#if DEBUG_SERIAL
    Serial.println("\n[INIT] Setup complete, entering main loop");
#endif
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    uint32_t now = millis();

    // Process incoming packets from ESP32
    sensecapComm.update();

    // Poll sensors at interval
    if (sensorPowerOn && (now - lastPollTime >= pollInterval)) {
        lastPollTime = now;
        readAllSensors();

        // Send data to ESP32
        sensecapComm.sendAllSensors(currentData);

        // Log to SD card
        if (sdCardInitialized) {
            logToSD(currentData);
        }
    }

    // Send heartbeat
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = now;
        sensecapComm.sendHeartbeat(now, statusFlags);
    }
}

// ============================================================================
// Initialization Functions
// ============================================================================

void initSensorPower() {
    pinMode(PIN_SENSOR_POWER, OUTPUT);
    digitalWrite(PIN_SENSOR_POWER, SENSOR_POWER_ON);
    sensorPowerOn = true;
    delay(100);  // Let sensors stabilize

#if DEBUG_SERIAL
    Serial.println("[PWR] Sensor power enabled");
#endif
}

void initI2C() {
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    statusFlags |= STATUS_FLAG_I2C_OK;

#if DEBUG_SERIAL
    Serial.printf("[I2C] Initialized on SDA=%d, SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
#endif
}

void initSensors() {
    uint16_t error;
    char errorMessage[256];

    // AHT20 - Temperature & Humidity
    if (aht.begin()) {
#if DEBUG_SERIAL
        Serial.println("[AHT20] Initialized OK");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[AHT20] ERROR - Not found!");
#endif
        sensecapComm.sendError(ERR_SENSOR_AHT20, "AHT20 not found");
    }

    // SGP40 - TVOC
    sgp40.begin(Wire);
    uint16_t serialNumber[3];
    error = sgp40.getSerialNumber(serialNumber, 3);
    if (!error) {
#if DEBUG_SERIAL
        Serial.printf("[SGP40] Initialized OK, Serial: 0x%04X%04X%04X\n",
                      serialNumber[0], serialNumber[1], serialNumber[2]);
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[SGP40] ERROR - Not found!");
#endif
        sensecapComm.sendError(ERR_SENSOR_SGP40, "SGP40 not found");
    }

    // SCD41 - CO2
    scd4x.begin(Wire, I2C_ADDR_SCD41);
    error = scd4x.stopPeriodicMeasurement();
    error = scd4x.startPeriodicMeasurement();
    if (!error) {
#if DEBUG_SERIAL
        Serial.println("[SCD41] Initialized OK, periodic measurement started");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[SCD41] ERROR - Init failed!");
#endif
        sensecapComm.sendError(ERR_SENSOR_SCD41, "SCD41 init failed");
    }

    sensorsInitialized = true;
    statusFlags |= STATUS_FLAG_SENSORS_OK;
    statusFlags |= STATUS_FLAG_POWER_ON;
}

void initSDCard() {
    SPI1.setSCK(PIN_SD_SCK);
    SPI1.setTX(PIN_SD_MOSI);
    SPI1.setRX(PIN_SD_MISO);

    if (SD.begin(PIN_SD_CS, SD_SPI_SPEED, SPI1)) {
        sdCardInitialized = true;
        statusFlags |= STATUS_FLAG_SD_CARD_OK;
#if DEBUG_SERIAL
        Serial.println("[SD] Card initialized OK");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[SD] Card not found or failed");
#endif
    }
}

void initBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

#if DEBUG_SERIAL
    Serial.printf("[BUZZER] Initialized on GPIO%d\n", PIN_BUZZER);
#endif
}

// ============================================================================
// Sensor Reading
// ============================================================================

void readAllSensors() {
    uint16_t error;
    currentData.timestamp = millis();

    // AHT20 - Temperature & Humidity
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
        currentData.temperature = temp_event.temperature;
        currentData.humidity = humidity_event.relative_humidity;
    }

    // SGP40 - TVOC with compensation
    uint16_t srawVoc;
    uint16_t compRh = (uint16_t)(currentData.humidity * 65535.0 / 100.0);
    uint16_t compT = (uint16_t)((currentData.temperature + 45.0) * 65535.0 / 175.0);

    error = sgp40.measureRawSignal(compRh, compT, srawVoc);
    if (!error) {
        currentData.tvoc_index = vocAlgorithm.process(srawVoc);
    }

    // SCD41 - CO2
    uint16_t co2;
    float scdTemp, scdHum;
    error = scd4x.readMeasurement(co2, scdTemp, scdHum);
    if (!error && co2 > 0) {
        currentData.co2 = co2;
    }

    // Grove ADC
    currentData.adc0 = analogRead(PIN_GROVE_ADC0);
    currentData.adc1 = analogRead(PIN_GROVE_ADC1);

#if DEBUG_SERIAL
    Serial.printf("[SENSORS] T=%.1fC H=%.1f%% CO2=%dppm TVOC=%d ADC0=%d ADC1=%d\n",
                  currentData.temperature,
                  currentData.humidity,
                  currentData.co2,
                  currentData.tvoc_index,
                  currentData.adc0,
                  currentData.adc1);
#endif
}

// ============================================================================
// Packet Handler
// ============================================================================

void handlePacket(const uint8_t *buffer, size_t size) {
    if (size < 1) return;

    uint8_t packetType = buffer[0];

#if DEBUG_SERIAL
    Serial.printf("[RX] Packet type=0x%02X size=%d\n", packetType, size);
#endif

    switch (packetType) {
        case PKT_TYPE_CMD_BEEP:
            if (size >= 3) {
                uint16_t duration = buffer[1] | (buffer[2] << 8);
                playBeep(duration);
            } else {
                playBeep(100);  // Default 100ms
            }
            sensecapComm.sendAck("beep");
            break;

        case PKT_TYPE_CMD_BEEP_PATTERN:
            if (size >= 2) {
                playBeepPattern(buffer[1]);
            }
            sensecapComm.sendAck("pattern");
            break;

        case PKT_TYPE_CMD_COLLECT_INTERVAL:
            if (size >= 5) {
                pollInterval = buffer[1] | (buffer[2] << 8) |
                               (buffer[3] << 16) | (buffer[4] << 24);
#if DEBUG_SERIAL
                Serial.printf("[CMD] Poll interval set to %d ms\n", pollInterval);
#endif
            }
            sensecapComm.sendAck("interval");
            break;

        case PKT_TYPE_CMD_SHUTDOWN:
            digitalWrite(PIN_SENSOR_POWER, SENSOR_POWER_OFF);
            sensorPowerOn = false;
            statusFlags &= ~STATUS_FLAG_POWER_ON;
#if DEBUG_SERIAL
            Serial.println("[CMD] Sensors powered OFF");
#endif
            sensecapComm.sendAck("shutdown");
            break;

        case PKT_TYPE_CMD_POWER_ON:
            digitalWrite(PIN_SENSOR_POWER, SENSOR_POWER_ON);
            sensorPowerOn = true;
            statusFlags |= STATUS_FLAG_POWER_ON;
            delay(100);
#if DEBUG_SERIAL
            Serial.println("[CMD] Sensors powered ON");
#endif
            sensecapComm.sendAck("power_on");
            break;

        case PKT_TYPE_CMD_READ_SENSORS:
            readAllSensors();
            sensecapComm.sendAllSensors(currentData);
            break;

        case PKT_TYPE_VERSION:
            sensecapComm.sendVersion();
            break;

        default:
#if DEBUG_SERIAL
            Serial.printf("[CMD] Unknown packet type: 0x%02X\n", packetType);
#endif
            sensecapComm.sendError(ERR_INVALID_COMMAND, "Unknown command");
            break;
    }
}

// ============================================================================
// Buzzer Functions
// ============================================================================

void playBeep(uint16_t duration_ms) {
    analogWrite(PIN_BUZZER, 127);  // 50% duty cycle
    delay(duration_ms);
    analogWrite(PIN_BUZZER, 0);

#if DEBUG_SERIAL
    Serial.printf("[BUZZER] Beep %d ms\n", duration_ms);
#endif
}

void playBeepPattern(uint8_t pattern) {
    switch (pattern) {
        case BEEP_PATTERN_SINGLE:
            playBeep(100);
            break;

        case BEEP_PATTERN_DOUBLE:
            playBeep(100);
            delay(100);
            playBeep(100);
            break;

        case BEEP_PATTERN_TRIPLE:
            for (int i = 0; i < 3; i++) {
                playBeep(80);
                delay(80);
            }
            break;

        case BEEP_PATTERN_LONG:
            playBeep(500);
            break;

        case BEEP_PATTERN_SUCCESS:
            for (int freq = 100; freq <= 255; freq += 30) {
                analogWrite(PIN_BUZZER, freq);
                delay(30);
            }
            analogWrite(PIN_BUZZER, 0);
            break;

        case BEEP_PATTERN_ERROR:
            for (int freq = 255; freq >= 100; freq -= 30) {
                analogWrite(PIN_BUZZER, freq);
                delay(30);
            }
            analogWrite(PIN_BUZZER, 0);
            break;

        case BEEP_PATTERN_ALARM:
            for (int i = 0; i < 5; i++) {
                playBeep(200);
                delay(200);
            }
            break;

        default:
            playBeep(100);
            break;
    }
}

// ============================================================================
// SD Card Logging
// ============================================================================

void logToSD(const SensorData &data) {
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    if (dataFile) {
        dataFile.printf("%lu,%.2f,%.2f,%d,%d,%d,%d\n",
                        data.timestamp,
                        data.temperature,
                        data.humidity,
                        data.co2,
                        data.tvoc_index,
                        data.adc0,
                        data.adc1);
        dataFile.close();

#if DEBUG_SERIAL
        Serial.println("[SD] Data logged");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[SD] Error opening file");
#endif
    }
}
