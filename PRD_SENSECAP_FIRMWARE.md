# Product Requirements Document: SenseCAP Indicator D1L Custom Firmware

**Version:** 1.0
**Date:** 2024-01-30
**Status:** Active Development

---

## 1. Overview

### 1.1 Purpose
Develop custom firmware for the SenseCAP Indicator D1L that enables full hardware utilization of both MCUs (ESP32-S3 and RP2040), establishes reliable inter-MCU communication, and provides a foundation for future Meshtastic integration.

### 1.2 Goals
- Enable all hardware peripherals on both MCUs
- Establish reliable bidirectional communication between ESP32-S3 and RP2040
- Validate WiFi and BLE connectivity
- Create comprehensive test suite for hardware validation
- Maintain compatibility with Meshtastic firmware architecture

### 1.3 Success Criteria
- All hardware tests pass
- Inter-MCU communication achieves >99% packet delivery
- WiFi can connect to AP and serve web page
- BLE can advertise and accept connections
- Display shows real-time sensor data from RP2040

---

## 2. Hardware Components

### 2.1 ESP32-S3 (Main Controller)
| Component | Interface | Priority | Status |
|-----------|-----------|----------|--------|
| UART to RP2040 | GPIO19/20 | P0 | ✅ Complete |
| TCA9535 IO Expander | I2C 0x20 | P0 | ✅ Complete |
| 4" RGB LCD (ST7701) | 16-bit RGB | P0 | ✅ Complete |
| Touch Panel (FT6336U) | I2C 0x48 | P1 | ✅ Complete |
| SX1262 LoRa Radio | SPI + IO Exp | P1 | 🔴 Not Started |
| User Button | GPIO38 | P1 | ✅ Complete |
| WiFi | Internal | P1 | ✅ Complete |
| Bluetooth LE | Internal | P1 | 🔴 Not Started |
| LCD Backlight | GPIO45 | P2 | ✅ Complete |

### 2.2 RP2040 (Peripheral Controller)

**Note:** The D1L model does NOT have onboard environmental sensors (AHT20, SGP40, SCD41).
Those sensors are only on D1S and D1Pro models. The D1L RP2040 handles SD card, buzzer, and Grove expansion.

| Component | Interface | Priority | Status | D1L |
|-----------|-----------|----------|--------|-----|
| UART to ESP32 | GP16/17 | P0 | ✅ Complete | ✓ |
| Buzzer | GP19 PWM | P1 | ✅ Complete | ✓ |
| SD Card | SPI1 | P2 | 🔴 Not Started | ✓ |
| Grove ADC | GP26/27 | P2 | 🔴 Not Started | ✓ |
| Sensor Power Control | GP18 | P1 | N/A | ✗ |
| AHT20 (Temp/Humidity) | I2C 0x38 | P1 | N/A | ✗ |
| SGP40 (TVOC) | I2C 0x59 | P1 | N/A | ✗ |
| SCD41 (CO2) | I2C 0x62 | P1 | N/A | ✗ |

---

## 3. Functional Requirements

### 3.1 Inter-MCU Communication (FR-001)
**Priority:** P0 - Critical

**Requirements:**
- FR-001.1: ESP32 UART TX/RX operational at 115200 baud
- FR-001.2: RP2040 UART TX/RX operational at 115200 baud
- FR-001.3: PacketSerial protocol with COBS encoding
- FR-001.4: Bidirectional packet delivery >99% reliability
- FR-001.5: Packet types for sensor data, commands, and status
- FR-001.6: Heartbeat mechanism with timeout detection

**Acceptance Tests:**
```
TEST_UART_001: ESP32 sends 1000 packets, RP2040 receives >990
TEST_UART_002: RP2040 sends 1000 packets, ESP32 receives >990
TEST_UART_003: Round-trip latency <10ms for single packet
TEST_UART_004: Heartbeat timeout triggers reconnect
```

### 3.2 IO Expander (FR-002)
**Priority:** P0 - Critical

**Requirements:**
- FR-002.1: TCA9535 I2C communication at 400kHz
- FR-002.2: Read/write individual pins
- FR-002.3: Configure pin direction (input/output)
- FR-002.4: Interrupt handling via GPIO42

**Acceptance Tests:**
```
TEST_IOEXP_001: Detect TCA9535 at 0x20 or 0x39
TEST_IOEXP_002: Toggle output pin, verify with multimeter
TEST_IOEXP_003: Read input pin state correctly
TEST_IOEXP_004: Interrupt triggers on pin change
```

### 3.3 Display System (FR-003)
**Priority:** P0 - Critical

**Requirements:**
- FR-003.1: Initialize ST7701 RGB panel via IO expander
- FR-003.2: LovyanGFX driver operational
- FR-003.3: LVGL integration with touch input
- FR-003.4: Display 480x480 at minimum 30fps
- FR-003.5: Backlight PWM control

**Acceptance Tests:**
```
TEST_LCD_001: Display solid color (red, green, blue, white)
TEST_LCD_002: Display text at multiple positions
TEST_LCD_003: LVGL widget renders and responds to touch
TEST_LCD_004: Framerate measurement >30fps
TEST_LCD_005: Backlight brightness 0-100%
```

### 3.4 Touch Panel (FR-004)
**Priority:** P1 - High

**Requirements:**
- FR-004.1: FT6336U I2C communication
- FR-004.2: Single touch point detection
- FR-004.3: Touch coordinates mapped to display
- FR-004.4: Touch event callbacks (press, release, drag)

**Acceptance Tests:**
```
TEST_TOUCH_001: Detect FT6336U at 0x48
TEST_TOUCH_002: Touch corners reports correct coordinates
TEST_TOUCH_003: Drag gesture detected
TEST_TOUCH_004: LVGL button responds to touch
```

### 3.5 LoRa Radio (FR-005)
**Priority:** P1 - High

**Requirements:**
- FR-005.1: SX1262 SPI communication via IO expander CS
- FR-005.2: Configure frequency, SF, BW, CR
- FR-005.3: Transmit packet
- FR-005.4: Receive packet with RSSI/SNR
- FR-005.5: TCXO detection and configuration

**Acceptance Tests:**
```
TEST_LORA_001: Read SX1262 chip version register
TEST_LORA_002: Configure 915MHz, SF7, 125kHz
TEST_LORA_003: TX packet, verify with SDR or second device
TEST_LORA_004: RX packet from known source
TEST_LORA_005: RSSI reading within expected range
```

### 3.6 Environmental Sensors (FR-006)
**Priority:** P1 - High

**Requirements:**
- FR-006.1: AHT20 temperature reading ±0.3°C accuracy
- FR-006.2: AHT20 humidity reading ±2% accuracy
- FR-006.3: SGP40 TVOC index 0-500
- FR-006.4: SCD41 CO2 reading 400-5000ppm
- FR-006.5: Sensor power control via GP18

**Acceptance Tests:**
```
TEST_SENSOR_001: AHT20 temperature matches reference ±1°C
TEST_SENSOR_002: AHT20 humidity matches reference ±5%
TEST_SENSOR_003: SGP40 returns valid TVOC index
TEST_SENSOR_004: SCD41 CO2 reading in expected range
TEST_SENSOR_005: Power off/on cycle, sensors reinitialize
```

### 3.7 Buzzer (FR-007)
**Priority:** P1 - High

**Requirements:**
- FR-007.1: PWM tone generation
- FR-007.2: Variable frequency 100Hz-10kHz
- FR-007.3: Variable duty cycle (volume)
- FR-007.4: Predefined beep patterns

**Acceptance Tests:**
```
TEST_BUZZER_001: Generate 1kHz tone, audible
TEST_BUZZER_002: Frequency sweep 500Hz-2kHz
TEST_BUZZER_003: Pattern playback (success, error, alarm)
```

### 3.8 SD Card (FR-008)
**Priority:** P2 - Medium

**Requirements:**
- FR-008.1: SPI communication at 1MHz+
- FR-008.2: FAT32 filesystem support
- FR-008.3: File read/write operations
- FR-008.4: Card detect

**Acceptance Tests:**
```
TEST_SD_001: Detect and mount SD card
TEST_SD_002: Write 1KB file, verify contents
TEST_SD_003: Read existing file correctly
TEST_SD_004: List directory contents
```

### 3.9 WiFi (FR-009)
**Priority:** P1 - High

**Requirements:**
- FR-009.1: Station mode - connect to AP
- FR-009.2: AP mode - create hotspot
- FR-009.3: HTTP server capability
- FR-009.4: mDNS/DNS-SD support
- FR-009.5: OTA update support

**Acceptance Tests:**
```
TEST_WIFI_001: Connect to known AP within 10 seconds
TEST_WIFI_002: Create AP, client can connect
TEST_WIFI_003: Serve HTTP page, accessible from browser
TEST_WIFI_004: mDNS name resolves (sensecap.local)
```

### 3.10 Bluetooth LE (FR-010)
**Priority:** P1 - High

**Requirements:**
- FR-010.1: BLE advertising
- FR-010.2: GATT server with custom service
- FR-010.3: Characteristic read/write/notify
- FR-010.4: Connection handling

**Acceptance Tests:**
```
TEST_BLE_001: Device visible in BLE scanner
TEST_BLE_002: Connect from phone app
TEST_BLE_003: Read characteristic returns expected value
TEST_BLE_004: Write characteristic triggers callback
TEST_BLE_005: Notification received on phone
```

### 3.11 User Button (FR-011)
**Priority:** P1 - High

**Requirements:**
- FR-011.1: Debounced button press detection
- FR-011.2: Short press (<500ms) event
- FR-011.3: Long press (>2s) event
- FR-011.4: Interrupt-driven, not polling

**Acceptance Tests:**
```
TEST_BTN_001: Short press triggers event
TEST_BTN_002: Long press triggers different event
TEST_BTN_003: No false triggers from noise
```

### 3.12 Grove ADC (FR-012)
**Priority:** P2 - Medium

**Requirements:**
- FR-012.1: 12-bit ADC reading
- FR-012.2: Voltage calculation 0-3.3V
- FR-012.3: Both channels operational

**Acceptance Tests:**
```
TEST_ADC_001: Read known voltage (1.65V), verify ±5%
TEST_ADC_002: Both channels read independently
```

---

## 4. Non-Functional Requirements

### 4.1 Performance
- NFR-001: Boot time <3 seconds to operational state
- NFR-002: Display refresh >30fps during normal operation
- NFR-003: Sensor polling at configurable 1-60 second intervals
- NFR-004: UART packet processing <1ms latency

### 4.2 Reliability
- NFR-005: No crashes during 24-hour continuous operation
- NFR-006: Graceful handling of I2C/SPI bus errors
- NFR-007: Watchdog timer prevents permanent hangs

### 4.3 Power
- NFR-008: Support sensor power-down for reduced consumption
- NFR-009: Display sleep mode after configurable timeout

### 4.4 Maintainability
- NFR-010: Modular code structure
- NFR-011: Comprehensive logging with log levels
- NFR-012: OTA firmware update capability

---

## 5. Test Infrastructure

### 5.1 Test Framework
- Unity test framework for C/C++
- PlatformIO test runner
- Hardware-in-loop tests where applicable

### 5.2 Test Categories
1. **Unit Tests** - Individual component functions
2. **Integration Tests** - Component interactions
3. **System Tests** - Full system validation
4. **Stress Tests** - Extended operation validation

### 5.3 Test Reporting
- Pass/Fail status for each test
- Timing metrics
- Coverage reporting where applicable

---

## 6. Development Phases

### Phase 1: Foundation (P0)
- [ ] ESP32 UART communication
- [ ] RP2040 UART communication
- [ ] Inter-MCU packet exchange
- [ ] TCA9535 IO Expander
- [ ] Basic LCD initialization

### Phase 2: Core Features (P1)
- [ ] Full LCD + LVGL
- [ ] Touch panel
- [ ] All sensors (AHT20, SGP40, SCD41)
- [ ] Buzzer
- [ ] User button
- [ ] WiFi connectivity
- [ ] BLE connectivity
- [ ] LoRa radio

### Phase 3: Extended Features (P2)
- [ ] SD card
- [ ] Grove ADC
- [ ] OTA updates
- [ ] Power management
- [ ] Advanced UI

### Phase 4: Integration
- [ ] Full system test
- [ ] 24-hour stability test
- [ ] Meshtastic compatibility check

---

## 7. Dependencies

### 7.1 External Libraries
| Library | Version | Purpose |
|---------|---------|---------|
| arduino-esp32 | mverch67 fork | TCA9535 GPIO support |
| LovyanGFX | mverch67 fork | ST7701 RGB display |
| LVGL | 8.3.x | UI framework |
| PacketSerial | 1.4.0 | UART framing |
| Sensirion I2C SGP40 | 1.0.0 | TVOC sensor |
| Sensirion I2C SCD4x | 0.4.0 | CO2 sensor |
| Adafruit AHTX0 | 2.0.5 | Temp/humidity |

### 7.2 Hardware Requirements
- SenseCAP Indicator D1L
- USB cables for both ports
- Known WiFi AP for testing
- BLE-capable phone for testing
- (Optional) Second LoRa device for RF testing
- (Optional) SDR for LoRa verification

---

## 8. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2024-01-30 | - | Initial PRD |
