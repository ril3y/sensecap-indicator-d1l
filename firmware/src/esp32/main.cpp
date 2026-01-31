/**
 * @file main.cpp
 * @brief SenseCAP Indicator ESP32-S3 Main Controller Firmware
 *
 * Handles:
 * - 4" RGB LCD Display (480x480)
 * - Capacitive touch panel
 * - LoRa radio (SX1262) via IO expander
 * - WiFi/BLE connectivity
 * - Communication with RP2040 sensor hub
 * - User interface
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "sensecap_pins.h"
#include "sensecap_protocol.h"
#include "sensecap_comm.h"

// ============================================================================
// Configuration
// ============================================================================

#define DEBUG_SERIAL                1       // Enable debug output
#define HEARTBEAT_REQUEST_MS        15000   // Request heartbeat every 15s

// ============================================================================
// TCA9535 IO Expander Driver (Simple Implementation)
// ============================================================================

class TCA9535 {
public:
    bool begin(uint8_t addr = I2C_ADDR_IO_EXPANDER) {
        _addr = addr;
        Wire.beginTransmission(_addr);
        if (Wire.endTransmission() != 0) {
            // Try alternate address
            _addr = I2C_ADDR_IO_EXPANDER_ALT;
            Wire.beginTransmission(_addr);
            if (Wire.endTransmission() != 0) {
                return false;
            }
        }
        // Set all outputs low initially
        writeRegister(0x02, 0x00);  // Output port 0
        writeRegister(0x03, 0x00);  // Output port 1
        return true;
    }

    void pinMode(uint8_t pin, uint8_t mode) {
        uint8_t reg = (pin < 8) ? 0x06 : 0x07;  // Config register
        uint8_t bit = pin % 8;
        uint8_t val = readRegister(reg);

        if (mode == OUTPUT) {
            val &= ~(1 << bit);  // 0 = output
        } else {
            val |= (1 << bit);   // 1 = input
        }
        writeRegister(reg, val);
    }

    void digitalWrite(uint8_t pin, uint8_t level) {
        uint8_t reg = (pin < 8) ? 0x02 : 0x03;  // Output register
        uint8_t bit = pin % 8;
        uint8_t val = readRegister(reg);

        if (level) {
            val |= (1 << bit);
        } else {
            val &= ~(1 << bit);
        }
        writeRegister(reg, val);
    }

    uint8_t digitalRead(uint8_t pin) {
        uint8_t reg = (pin < 8) ? 0x00 : 0x01;  // Input register
        uint8_t bit = pin % 8;
        uint8_t val = readRegister(reg);
        return (val >> bit) & 0x01;
    }

    uint16_t readInputs() {
        uint8_t low = readRegister(0x00);
        uint8_t high = readRegister(0x01);
        return (high << 8) | low;
    }

private:
    uint8_t _addr;

    void writeRegister(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readRegister(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_addr, (uint8_t)1);
        return Wire.read();
    }
};

// ============================================================================
// Global Objects
// ============================================================================

TCA9535 ioExpander;

// Current sensor data from RP2040
static SensorData sensorData;
static bool sensorDataValid = false;
static uint32_t lastSensorUpdate = 0;
static uint32_t lastHeartbeatRequest = 0;

// RP2040 status
static uint32_t rp2040Uptime = 0;
static uint8_t rp2040StatusFlags = 0;

// ============================================================================
// Forward Declarations
// ============================================================================

void initI2C();
void initIOExpander();
void initUART();
void initButton();
void initDisplay();
void initLoRa();
void handlePacket(const uint8_t *buffer, size_t size);
void handleButton();

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Initialize USB Serial for debug
#if DEBUG_SERIAL
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("SenseCAP Indicator ESP32-S3 Main Controller");
    Serial.printf("Firmware v%d.%d.%d\n", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    Serial.println("========================================\n");
#endif

    // Initialize hardware
    initI2C();
    initIOExpander();
    initUART();
    initButton();
    initDisplay();

    // Release RP2040 from reset (if held)
    ioExpander.pinMode(IOEXP_RP2040_RST, OUTPUT);
    ioExpander.digitalWrite(IOEXP_RP2040_RST, HIGH);

    // Initialize LoRa (D1L model)
#ifdef SENSECAP_D1L
    initLoRa();
#endif

    // Send version request to RP2040
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

    // Process incoming packets from RP2040
    sensecapComm.update();

    // Handle button press
    handleButton();

    // Request heartbeat periodically
    if (now - lastHeartbeatRequest >= HEARTBEAT_REQUEST_MS) {
        lastHeartbeatRequest = now;
        // Could send a ping/heartbeat request here
    }

    // Update display with sensor data
    // TODO: Implement LVGL UI updates

    // Small delay to prevent tight loop
    delay(10);
}

// ============================================================================
// Initialization Functions
// ============================================================================

void initI2C() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);  // 400kHz

#if DEBUG_SERIAL
    Serial.printf("[I2C] Initialized on SDA=%d, SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
#endif
}

void initIOExpander() {
    if (ioExpander.begin()) {
#if DEBUG_SERIAL
        Serial.println("[IO_EXP] TCA9535 initialized");
#endif

        // Configure LoRa pins
        ioExpander.pinMode(IOEXP_LORA_NSS, OUTPUT);
        ioExpander.pinMode(IOEXP_LORA_RST, OUTPUT);
        ioExpander.pinMode(IOEXP_LORA_BUSY, INPUT);
        ioExpander.pinMode(IOEXP_LORA_DIO1, INPUT);
        ioExpander.pinMode(IOEXP_TCXO_VER, INPUT);

        // Set LoRa NSS high (inactive)
        ioExpander.digitalWrite(IOEXP_LORA_NSS, HIGH);
        ioExpander.digitalWrite(IOEXP_LORA_RST, HIGH);

        // Configure LCD pins
        ioExpander.pinMode(IOEXP_LCD_CS, OUTPUT);
        ioExpander.pinMode(IOEXP_LCD_RST, OUTPUT);
        ioExpander.digitalWrite(IOEXP_LCD_CS, HIGH);
        ioExpander.digitalWrite(IOEXP_LCD_RST, HIGH);

        // Configure Touch reset
        ioExpander.pinMode(IOEXP_TOUCH_RST, OUTPUT);
        ioExpander.digitalWrite(IOEXP_TOUCH_RST, HIGH);

        // Configure RP2040 reset
        ioExpander.pinMode(IOEXP_RP2040_RST, OUTPUT);
        ioExpander.digitalWrite(IOEXP_RP2040_RST, HIGH);

        // Check TCXO version
        bool hasTCXO = ioExpander.digitalRead(IOEXP_TCXO_VER);
#if DEBUG_SERIAL
        Serial.printf("[IO_EXP] TCXO detected: %s\n", hasTCXO ? "YES" : "NO");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[IO_EXP] ERROR - TCA9535 not found!");
#endif
    }
}

void initUART() {
    // Use Serial1 for RP2040 communication
    Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    sensecapComm.begin(Serial1, UART_BAUD_RATE);
    sensecapComm.onPacketReceived(handlePacket);

#if DEBUG_SERIAL
    Serial.printf("[UART] RP2040 communication initialized (TX=%d, RX=%d)\n",
                  PIN_UART_TX, PIN_UART_RX);
#endif
}

void initButton() {
    pinMode(PIN_USER_BUTTON, INPUT_PULLUP);

#if DEBUG_SERIAL
    Serial.printf("[BTN] User button on GPIO%d\n", PIN_USER_BUTTON);
#endif
}

void initDisplay() {
    // Enable backlight
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);

    // Reset LCD via IO expander
    ioExpander.digitalWrite(IOEXP_LCD_RST, LOW);
    delay(10);
    ioExpander.digitalWrite(IOEXP_LCD_RST, HIGH);
    delay(50);

    // Reset touch controller
    ioExpander.digitalWrite(IOEXP_TOUCH_RST, LOW);
    delay(10);
    ioExpander.digitalWrite(IOEXP_TOUCH_RST, HIGH);
    delay(50);

#if DEBUG_SERIAL
    Serial.println("[LCD] Display initialized (backlight ON)");
#endif

    // TODO: Initialize LovyanGFX and LVGL here
    // This requires more complex setup - see separate display driver
}

void initLoRa() {
    // Initialize SPI for LoRa
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // Reset LoRa
    ioExpander.digitalWrite(IOEXP_LORA_RST, LOW);
    delay(10);
    ioExpander.digitalWrite(IOEXP_LORA_RST, HIGH);
    delay(50);

    // Wait for LoRa to be ready (BUSY goes low)
    uint32_t timeout = millis() + 1000;
    while (ioExpander.digitalRead(IOEXP_LORA_BUSY) && millis() < timeout) {
        delay(1);
    }

    if (millis() >= timeout) {
#if DEBUG_SERIAL
        Serial.println("[LORA] ERROR - Timeout waiting for BUSY");
#endif
    } else {
#if DEBUG_SERIAL
        Serial.println("[LORA] SX1262 ready");
#endif
    }

    // TODO: Full SX1262 initialization
}

// ============================================================================
// Packet Handler (from RP2040)
// ============================================================================

void handlePacket(const uint8_t *buffer, size_t size) {
    if (size < 1) return;

    uint8_t packetType = buffer[0];

#if DEBUG_SERIAL
    Serial.printf("[RX] Packet from RP2040: type=0x%02X size=%d\n", packetType, size);
#endif

    switch (packetType) {
        case PKT_TYPE_SENSOR_TEMP:
            if (size >= sizeof(SensorPacket)) {
                memcpy(&sensorData.temperature, &buffer[1], sizeof(float));
#if DEBUG_SERIAL
                Serial.printf("  Temperature: %.2f C\n", sensorData.temperature);
#endif
            }
            break;

        case PKT_TYPE_SENSOR_HUMIDITY:
            if (size >= sizeof(SensorPacket)) {
                memcpy(&sensorData.humidity, &buffer[1], sizeof(float));
#if DEBUG_SERIAL
                Serial.printf("  Humidity: %.2f %%\n", sensorData.humidity);
#endif
            }
            break;

        case PKT_TYPE_SENSOR_CO2:
            if (size >= sizeof(SensorPacket)) {
                float co2;
                memcpy(&co2, &buffer[1], sizeof(float));
                sensorData.co2 = (uint16_t)co2;
#if DEBUG_SERIAL
                Serial.printf("  CO2: %d ppm\n", sensorData.co2);
#endif
            }
            break;

        case PKT_TYPE_SENSOR_TVOC:
            if (size >= sizeof(SensorPacket)) {
                float tvoc;
                memcpy(&tvoc, &buffer[1], sizeof(float));
                sensorData.tvoc_index = (uint16_t)tvoc;
#if DEBUG_SERIAL
                Serial.printf("  TVOC: %d\n", sensorData.tvoc_index);
#endif
            }
            break;

        case PKT_TYPE_SENSOR_ADC0:
            if (size >= sizeof(SensorPacket16)) {
                memcpy(&sensorData.adc0, &buffer[1], sizeof(uint16_t));
            }
            break;

        case PKT_TYPE_SENSOR_ADC1:
            if (size >= sizeof(SensorPacket16)) {
                memcpy(&sensorData.adc1, &buffer[1], sizeof(uint16_t));
            }
            break;

        case PKT_TYPE_HEARTBEAT:
            if (size >= sizeof(HeartbeatPacket)) {
                HeartbeatPacket *hb = (HeartbeatPacket *)buffer;
                rp2040Uptime = hb->uptime_ms;
                rp2040StatusFlags = hb->status_flags;
#if DEBUG_SERIAL
                Serial.printf("  RP2040 Heartbeat: uptime=%lu ms, flags=0x%02X\n",
                              rp2040Uptime, rp2040StatusFlags);
#endif
            }
            break;

        case PKT_TYPE_VERSION:
            if (size >= sizeof(VersionPacket)) {
                VersionPacket *ver = (VersionPacket *)buffer;
#if DEBUG_SERIAL
                Serial.printf("  RP2040 Version: %d.%d.%d (%s)\n",
                              ver->major, ver->minor, ver->patch, ver->build);
#endif
            }
            break;

        case PKT_TYPE_ACK:
#if DEBUG_SERIAL
            Serial.println("  ACK received");
#endif
            break;

        case PKT_TYPE_ERROR:
            if (size >= 3) {
                StatusPacket *err = (StatusPacket *)buffer;
#if DEBUG_SERIAL
                Serial.printf("  ERROR from RP2040: code=%d, msg=%s\n",
                              err->code, err->message);
#endif
            }
            break;

        default:
#if DEBUG_SERIAL
            Serial.printf("  Unknown packet type: 0x%02X\n", packetType);
#endif
            break;
    }

    sensorDataValid = true;
    lastSensorUpdate = millis();
}

// ============================================================================
// Button Handler
// ============================================================================

static bool lastButtonState = HIGH;
static uint32_t buttonPressTime = 0;

void handleButton() {
    bool currentState = digitalRead(PIN_USER_BUTTON);

    // Button pressed (active low)
    if (currentState == LOW && lastButtonState == HIGH) {
        buttonPressTime = millis();
#if DEBUG_SERIAL
        Serial.println("[BTN] Pressed");
#endif
    }

    // Button released
    if (currentState == HIGH && lastButtonState == LOW) {
        uint32_t pressDuration = millis() - buttonPressTime;

#if DEBUG_SERIAL
        Serial.printf("[BTN] Released after %lu ms\n", pressDuration);
#endif

        if (pressDuration < 500) {
            // Short press - request sensor reading
            sensecapComm.sendReadSensors();
        } else if (pressDuration < 2000) {
            // Medium press - beep
            sensecapComm.sendBeep(200);
        } else {
            // Long press - toggle sensor power
            static bool sensorsOn = true;
            sensorsOn = !sensorsOn;
            if (sensorsOn) {
                sensecapComm.sendPowerOn();
            } else {
                sensecapComm.sendShutdown();
            }
        }
    }

    lastButtonState = currentState;
}
