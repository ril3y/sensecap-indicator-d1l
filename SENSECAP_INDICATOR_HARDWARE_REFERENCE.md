# SenseCAP Indicator D1L - Complete Hardware Reference

*Reverse-engineered from official source code repositories*

---

## Model Variants

| Feature | D1 | D1S | D1L | D1Pro |
|---------|:--:|:---:|:---:|:-----:|
| tVOC Sensor (SGP40) | ✗ | ✓ | ✗ | ✓ |
| CO2 Sensor (SCD41) | ✗ | ✓ | ✗ | ✓ |
| Temp/Humidity (AHT20) | ✗ | ✓ | ✗ | ✓ |
| LoRa (SX1262) | ✗ | ✗ | ✓ | ✓ |
| Wi-Fi | ✓ | ✓ | ✓ | ✓ |
| Bluetooth | ✓ | ✓ | ✓ | ✓ |
| SD Card | ✓ | ✓ | ✓ | ✓ |
| Buzzer | ✓ | ✓ | ✓ | ✓ |
| Grove ADC | ✓ | ✓ | ✓ | ✓ |

**This document covers the D1L model** - LoRa-enabled variant without onboard environmental sensors.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [ESP32-S3 Hardware](#esp32-s3-hardware)
3. [RP2040 Hardware](#rp2040-hardware)
4. [Inter-MCU Communication](#inter-mcu-communication)
5. [Display Subsystem](#display-subsystem)
6. [LoRa Radio Subsystem](#lora-radio-subsystem)
7. [Sensor Subsystem](#sensor-subsystem)
8. [Storage Subsystem](#storage-subsystem)
9. [User Interface Hardware](#user-interface-hardware)
10. [Power Management](#power-management)
11. [Grove Connectors](#grove-connectors)
12. [Complete Pin Tables](#complete-pin-tables)
13. [I2C Address Map](#i2c-address-map)
14. [Code Examples](#code-examples)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        SENSECAP INDICATOR D1L                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────┐              ┌─────────────────────┐              │
│   │     ESP32-S3        │    UART      │       RP2040        │              │
│   │   (Main Controller) │◄────────────►│    (Sensor Hub)     │              │
│   │                     │   115200     │                     │              │
│   │  • WiFi 2.4GHz      │    baud      │  • Dual Cortex-M0+  │              │
│   │  • Bluetooth 5.0    │              │  • 133 MHz          │              │
│   │  • 8MB Flash        │              │  • 264KB SRAM       │              │
│   │  • 240 MHz          │              │  • 2MB Flash        │              │
│   └──────────┬──────────┘              └──────────┬──────────┘              │
│              │                                    │                         │
│   ┌──────────┴──────────┐              ┌──────────┴──────────┐              │
│   │  ESP32 Peripherals  │              │  RP2040 Peripherals │              │
│   ├─────────────────────┤              ├─────────────────────┤              │
│   │ • 4" RGB LCD 480x480│              │ • AHT20 Temp/Humid  │              │
│   │ • Touch Panel       │              │ • SGP40 TVOC        │              │
│   │ • SX1262 LoRa       │              │ • SCD41 CO2         │              │
│   │ • TCA9535 IO Expand │              │ • MicroSD Card      │              │
│   │ • User Button       │              │ • Buzzer            │              │
│   │ • BMP3xx (Pro only) │              │ • Grove ADC         │              │
│   └─────────────────────┘              └─────────────────────┘              │
│                                                                             │
│   ┌─────────────────────┐              ┌─────────────────────┐              │
│   │    USB-C Port 1     │              │    USB-C Port 2     │              │
│   │   (ESP32 Native)    │              │   (RP2040 Native)   │              │
│   └─────────────────────┘              └─────────────────────┘              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## ESP32-S3 Hardware

### Specifications
| Parameter | Value |
|-----------|-------|
| Processor | Xtensa LX7 Dual-Core @ 240 MHz |
| Flash | 8 MB |
| PSRAM | 8 MB (if equipped) |
| WiFi | 802.11 b/g/n 2.4 GHz |
| Bluetooth | BLE 5.0 |
| USB | Native USB 2.0 OTG |

### GPIO Allocation Summary

| GPIO Range | Function | Count |
|------------|----------|-------|
| GPIO0-15 | LCD RGB Data Bus | 16 |
| GPIO16-18, 21 | LCD Control Signals | 4 |
| GPIO38 | User Button | 1 |
| GPIO39-40 | I2C Bus | 2 |
| GPIO41, 47-48 | SPI (LoRa) | 3 |
| GPIO42 | IO Expander Interrupt | 1 |
| GPIO19-20 | UART (to RP2040) | 2 |
| GPIO45 | LCD Backlight | 1 |

---

## RP2040 Hardware

### Specifications
| Parameter | Value |
|-----------|-------|
| Processor | Dual ARM Cortex-M0+ @ 133 MHz |
| Flash | 2 MB |
| SRAM | 264 KB |
| USB | Native USB 1.1 |

### GPIO Allocation Summary

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| GP10 | SPI1 SCK | Output | SD Card Clock |
| GP11 | SPI1 TX (MOSI) | Output | SD Card Data Out |
| GP12 | SPI1 RX (MISO) | Input | SD Card Data In |
| GP13 | SD Card CS | Output | Chip Select |
| GP16 | UART TX | Output | To ESP32 RX |
| GP17 | UART RX | Input | From ESP32 TX |
| GP18 | Sensor Power | Output | HIGH = Sensors ON |
| GP19 | Buzzer | Output | PWM Audio |
| GP20 | I2C SDA | Bidir | Sensor Data |
| GP21 | I2C SCL | Output | Sensor Clock |
| GP26 | ADC0 | Input | Grove Analog 0 |
| GP27 | ADC1 | Input | Grove Analog 1 |

### Unused RP2040 GPIOs (Available for Extension)
```
GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7, GP8, GP9
GP14, GP15
GP22, GP23, GP24, GP25
GP28, GP29
```

---

## Inter-MCU Communication

### UART Configuration
```
Protocol:    PacketSerial (COBS encoding)
Baud Rate:   115200
Data Bits:   8
Parity:      None
Stop Bits:   1

ESP32-S3          RP2040
────────          ──────
GPIO19 (TX) ───► GP17 (RX)
GPIO20 (RX) ◄─── GP16 (TX)
```

### Packet Format
```
┌──────────┬─────────────────────────────┐
│ Type (1B)│ Payload (variable)          │
└──────────┴─────────────────────────────┘
```

### Packet Types

**Sensor Data (RP2040 → ESP32):**
| Type Code | Name | Payload | Description |
|-----------|------|---------|-------------|
| 0xB2 | CO2 | float (4 bytes) | CO2 in ppm |
| 0xB3 | Temperature | float (4 bytes) | Temperature in °C |
| 0xB4 | Humidity | float (4 bytes) | Relative humidity % |
| 0xB5 | TVOC Index | float (4 bytes) | VOC index 0-500 |

**Commands (ESP32 → RP2040):**
| Type Code | Name | Payload | Description |
|-----------|------|---------|-------------|
| 0xA0 | Collect Interval | uint32 | Polling rate in ms |
| 0xA1 | Beep On | none | Trigger buzzer |
| 0xA3 | Shutdown | none | Power off sensors |
| 0x00 | ACK | string | Acknowledgment |

---

## Display Subsystem

### LCD Panel Specifications
| Parameter | Value |
|-----------|-------|
| Size | 4.0 inches |
| Resolution | 480 × 480 pixels |
| Interface | 16-bit RGB565 Parallel |
| Controller | ST7701S |
| Color Depth | 16-bit (65K colors) |
| Backlight | LED, PWM controlled |

### LCD Wiring

```
ESP32-S3                                    ST7701S LCD
────────                                    ──────────

         ┌── RED CHANNEL ──┐
GPIO0  ──┼─► R4 (MSB)      │
GPIO1  ──┼─► R3            │
GPIO2  ──┼─► R2            │
GPIO3  ──┼─► R1            │
GPIO4  ──┼─► R0 (LSB)      │
         └─────────────────┘

         ┌── GREEN CHANNEL ─┐
GPIO5  ──┼─► G5 (MSB)       │
GPIO6  ──┼─► G4             │
GPIO7  ──┼─► G3             │
GPIO8  ──┼─► G2             │
GPIO9  ──┼─► G1             │
GPIO10 ──┼─► G0 (LSB)       │
         └──────────────────┘

         ┌── BLUE CHANNEL ──┐
GPIO11 ──┼─► B4 (MSB)       │
GPIO12 ──┼─► B3             │
GPIO13 ──┼─► B2             │
GPIO14 ──┼─► B1             │
GPIO15 ──┼─► B0 (LSB)       │
         └──────────────────┘

         ┌── CONTROL ───────┐
GPIO16 ──┼─► HSYNC          │
GPIO17 ──┼─► VSYNC          │
GPIO18 ──┼─► DE             │
GPIO21 ──┼─► PCLK           │
GPIO45 ──┼─► Backlight      │
         └──────────────────┘

         ┌── VIA IO EXPANDER ┐
IO4    ──┼─► LCD_CS          │
IO5    ──┼─► LCD_RESET       │
         └───────────────────┘
```

### LCD Timing Parameters
```c
// Resolution
LCD_WIDTH          = 480
LCD_HEIGHT         = 480

// Pixel Clock
LCD_FREQ           = 14000000  // 14 MHz

// Horizontal Timing (in pixel clocks)
HSYNC_BACK_PORCH   = 50
HSYNC_FRONT_PORCH  = 10
HSYNC_PULSE_WIDTH  = 8

// Vertical Timing (in lines)
VSYNC_BACK_PORCH   = 20
VSYNC_FRONT_PORCH  = 10
VSYNC_PULSE_WIDTH  = 8

// Polarity
PCLK_ACTIVE_NEG    = 0   // Sample on rising edge
LCD_COLOR_INV      = false
```

### Touch Panel

```
ESP32-S3                     Touch Controller
────────                     ────────────────
GPIO39 (SDA) ◄──────────────► SDA
GPIO40 (SCL) ───────────────► SCL

TCA9535
───────
IO7 ────────────────────────► RESET

I2C Address: 0x48 (FT6336U - GX variant)
             0x38 (FT5x06  - DX variant)
```

---

## LoRa Radio Subsystem

### SX1262 Specifications
| Parameter | Value |
|-----------|-------|
| Frequency | 868/915 MHz (region dependent) |
| Modulation | LoRa / FSK |
| Max Power | +22 dBm |
| Sensitivity | -137 dBm |
| Interface | SPI + GPIO |

### SX1262 Wiring

The LoRa radio uses SPI for data transfer, but control signals are routed through the TCA9535 IO expander.

```
ESP32-S3                                    SX1262
────────                                    ──────

         ┌── SPI BUS ──────┐
GPIO48 ──┼─► MOSI          │
GPIO47 ◄─┼── MISO          │
GPIO41 ──┼─► SCLK          │
         └─────────────────┘

TCA9535 IO Expander
───────────────────
IO0  ────────────────────────► NSS (Chip Select)
IO1  ────────────────────────► RESET
IO2  ◄───────────────────────── BUSY
IO3  ◄───────────────────────── DIO1 (IRQ)
IO11 ◄───────────────────────── TCXO_VER (version detect)

ESP32-S3
────────
GPIO42 ◄───────────────────────── TCA9535 INT (active low)
```

### TCXO Configuration
```c
// If IO11 reads HIGH, TCXO is present
SX126X_TCXO_CTRL_VOLTAGE = TCXO_CTRL_2_4V
BOARD_TCXO_WAKEUP_TIME   = 5  // milliseconds
```

---

## Sensor Subsystem

All environmental sensors are connected to the RP2040 via I2C.

### Sensor Power Control
```c
// RP2040 GP18 controls power to all sensors
pinMode(18, OUTPUT);
digitalWrite(18, HIGH);  // Sensors ON
digitalWrite(18, LOW);   // Sensors OFF
```

### AHT20 - Temperature & Humidity
```
Manufacturer:  Aosong
Interface:     I2C
Address:       0x38
Measurements:  Temperature (-40 to +85°C, ±0.3°C)
               Humidity (0-100% RH, ±2%)

RP2040 Wiring:
  GP20 (SDA) ◄──► AHT20 SDA
  GP21 (SCL) ───► AHT20 SCL
  GP18 ──────────► VDD (via power switch)
```

### SGP40 - TVOC Sensor
```
Manufacturer:  Sensirion
Interface:     I2C
Address:       0x59
Measurement:   VOC Index (0-500)
               Raw signal (SRAW_VOC)

RP2040 Wiring:
  GP20 (SDA) ◄──► SGP40 SDA
  GP21 (SCL) ───► SGP40 SCL
  GP18 ──────────► VDD (via power switch)
```

### SCD41 - CO2 Sensor
```
Manufacturer:  Sensirion
Interface:     I2C
Address:       0x62
Measurements:  CO2 (400-5000 ppm)
               Temperature (-10 to +60°C)
               Humidity (0-100% RH)

RP2040 Wiring:
  GP20 (SDA) ◄──► SCD41 SDA
  GP21 (SCL) ───► SCD41 SCL
  GP18 ──────────► VDD (via power switch)
```

### BMP3xx - Pressure Sensor (D1Pro only)
```
Manufacturer:  Bosch
Interface:     I2C (on ESP32 bus)
Address:       0x77
Measurements:  Pressure, Temperature

ESP32-S3 Wiring:
  GPIO39 (SDA) ◄──► BMP3xx SDA
  GPIO40 (SCL) ───► BMP3xx SCL

TCA9535:
  IO10 ────────────► VDD (power enable)
```

---

## Storage Subsystem

### MicroSD Card
```
Interface:   SPI (SPI1 peripheral)
Max Size:    32 GB (FAT32)
Speed:       1 MHz default

RP2040 Wiring:
  GP10 (SCK)  ───► SD CLK
  GP11 (MOSI) ───► SD DI (Data In)
  GP12 (MISO) ◄─── SD DO (Data Out)
  GP13        ───► SD CS (Chip Select)
```

### SD Card Code Example
```cpp
#include <SPI.h>
#include <SD.h>

void setup() {
  const int chipSelect = 13;

  // Configure SPI1 pins
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);

  // Initialize at 1 MHz
  if (!SD.begin(chipSelect, 1000000, SPI1)) {
    Serial.println("Card failed");
  }
}
```

---

## User Interface Hardware

### User Button
```
Location:    ESP32-S3 GPIO38
Type:        Momentary push button
Active:      LOW (pressed = 0)
Pull-up:     Internal or external 10K

ESP32-S3 Wiring:
  GPIO38 ────┬──── Button ──── GND
             │
          [10K]
             │
           3.3V
```

### Buzzer
```
Location:    RP2040 GP19
Type:        Piezoelectric
Drive:       PWM (analogWrite)
Frequency:   Variable (PWM duty cycle)

RP2040 Wiring:
  GP19 ────────► Buzzer (+)
  GND  ────────► Buzzer (-)
```

### Buzzer Code Example
```cpp
#define BUZZER_PIN 19

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void beep() {
  analogWrite(BUZZER_PIN, 127);  // 50% duty cycle
  delay(100);
  analogWrite(BUZZER_PIN, 0);    // Off
}
```

---

## Power Management

### Power Distribution
```
USB 5V Input
     │
     ├──► 3.3V Regulator ──┬──► ESP32-S3 VDD
     │                     ├──► RP2040 VDD
     │                     ├──► TCA9535 VDD
     │                     ├──► LCD VDD
     │                     ├──► Touch VDD
     │                     └──► SX1262 VDD
     │
     └──► Sensor Power Switch (GP18)
                │
                └──► AHT20, SGP40, SCD41 VDD
```

### Power Specifications
| Rail | Voltage | Max Current | Notes |
|------|---------|-------------|-------|
| USB Input | 5V | 1A | Main power |
| Logic | 3.3V | - | All digital |
| LoRa TCXO | 2.4V | - | If equipped |

### Sleep/Wake Control

The ESP32 can reset the RP2040 via the IO expander:
```c
// TCA9535 IO8 = RP2040 RESET
io_expander->set_direction(8, 1);  // Output
io_expander->set_level(8, 0);      // Assert reset
delay(10);
io_expander->set_level(8, 1);      // Release reset
```

### Deep Sleep Wake-up Limitations

**IMPORTANT:** The ESP32-S3 can only wake from deep sleep using RTC GPIOs (GPIO 0-21).
Most of these are used by the LCD, severely limiting external wake-up options.

| Wake Source | GPIO | RTC GPIO? | Deep Sleep Wake? |
|-------------|------|-----------|------------------|
| User Button | 38 | No | ❌ Not possible |
| IO Expander INT | 42 | No | ❌ Not possible |
| LoRa DIO1 (via IO Exp) | 42 | No | ❌ Not possible |

**Available RTC GPIOs for Wake-up:**
| GPIO | Status | Notes |
|------|--------|-------|
| 19 | **FREE** | Directly on ESP32-S3, not routed externally |
| 20 | **FREE** | Directly on ESP32-S3, not routed externally |

All other RTC GPIOs (0-18, 21) are used by the LCD RGB interface.

**Practical Wake-up Options:**
- **Timer wake-up** - `esp_sleep_enable_timer_wakeup(microseconds)`
- **ULP coprocessor** - Can monitor IO expander via I2C (complex)
- **Touch wake-up** - Not practical (touch pins conflict with LCD)
- **GPIO 19/20** - If you can access internal test pads

---

## Grove Connectors

The SenseCAP Indicator has two Grove connectors on the RP2040 side.

### Grove I2C Connector
```
Pin 1: GND
Pin 2: VCC (3.3V)
Pin 3: SDA (GP20) ──► Directly connected to sensor I2C bus
Pin 4: SCL (GP21) ──► Directly connected to sensor I2C bus

Note: Directly shared with internal sensors. Use different
      I2C addresses for external devices.
```

### Grove ADC Connector
```
Pin 1: GND
Pin 2: VCC (3.3V)
Pin 3: A0 (GP26) ──► 12-bit ADC input, 0-3.3V
Pin 4: A1 (GP27) ──► 12-bit ADC input, 0-3.3V
```

### Grove ADC Code Example
```cpp
#define GROVE_ADC0 26
#define GROVE_ADC1 27

void setup() {
  Serial.begin(115200);
}

void loop() {
  int value0 = analogRead(GROVE_ADC0);  // 0-4095
  int value1 = analogRead(GROVE_ADC1);  // 0-4095

  float voltage0 = value0 * 3.3 / 4095.0;
  float voltage1 = value1 * 3.3 / 4095.0;

  Serial.printf("A0: %.2fV, A1: %.2fV\n", voltage0, voltage1);
  delay(1000);
}
```

---

## Complete Pin Tables

### ESP32-S3 Complete GPIO Map

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 0 | LCD R4 | Output | Red MSB |
| 1 | LCD R3 | Output | |
| 2 | LCD R2 | Output | |
| 3 | LCD R1 | Output | |
| 4 | LCD R0 | Output | Red LSB |
| 5 | LCD G5 | Output | Green MSB |
| 6 | LCD G4 | Output | |
| 7 | LCD G3 | Output | |
| 8 | LCD G2 | Output | |
| 9 | LCD G1 | Output | |
| 10 | LCD G0 | Output | Green LSB |
| 11 | LCD B4 | Output | Blue MSB |
| 12 | LCD B3 | Output | |
| 13 | LCD B2 | Output | |
| 14 | LCD B1 | Output | |
| 15 | LCD B0 | Output | Blue LSB |
| 16 | LCD HSYNC | Output | |
| 17 | LCD VSYNC | Output | |
| 18 | LCD DE | Output | Data Enable |
| 19-20 | - | - | Not used |
| 21 | LCD PCLK | Output | Pixel Clock |
| 22-37 | - | - | Reserved/Not used |
| 38 | User Button | Input | Active LOW |
| 39 | I2C SDA | Bidir | |
| 40 | I2C SCL | Output | |
| 41 | SPI SCLK | Output | LoRa |
| 42 | IO Exp INT | Input | Active LOW |
| 19 | UART TX | Output | To RP2040 GP17 |
| 20 | UART RX | Input | From RP2040 GP16 |
| 45 | LCD Backlight | Output | Active HIGH |
| 46 | - | - | Not used |
| 47 | SPI MISO | Input | LoRa |
| 48 | SPI MOSI | Output | LoRa |

### RP2040 Complete GPIO Map

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 0-9 | **UNUSED** | - | Available for extension |
| 10 | SPI1 SCK | Output | SD Card |
| 11 | SPI1 MOSI | Output | SD Card |
| 12 | SPI1 MISO | Input | SD Card |
| 13 | SD CS | Output | SD Card |
| 14-15 | **UNUSED** | - | Available for extension |
| 16 | UART TX | Output | To ESP32 |
| 17 | UART RX | Input | From ESP32 |
| 18 | Sensor Power | Output | HIGH = ON |
| 19 | Buzzer | Output | PWM |
| 20 | I2C SDA | Bidir | Sensors + Grove |
| 21 | I2C SCL | Output | Sensors + Grove |
| 22-25 | **UNUSED** | - | Available for extension |
| 26 | ADC0 | Input | Grove Analog |
| 27 | ADC1 | Input | Grove Analog |
| 28-29 | **UNUSED** | - | Available for extension |

### TCA9535 IO Expander Map

| IO Pin | Function | Direction | Notes |
|--------|----------|-----------|-------|
| IO0 | LoRa NSS | Output | SPI Chip Select |
| IO1 | LoRa RST | Output | Reset |
| IO2 | LoRa BUSY | Input | Busy status |
| IO3 | LoRa DIO1 | Input | IRQ |
| IO4 | LCD CS | Output | |
| IO5 | LCD RST | Output | Reset |
| IO6 | **UNUSED** | - | Available |
| IO7 | Touch RST | Output | Reset |
| IO8 | RP2040 RST | Output | Reset control |
| IO9 | **UNUSED** | - | Available |
| IO10 | BMP Power | Output | D1Pro only |
| IO11 | TCXO Detect | Input | HIGH = TCXO present |
| IO12-15 | **UNUSED** | - | Available |

---

## I2C Address Map

### ESP32-S3 I2C Bus (GPIO39/40)

| Address | Device | Notes |
|---------|--------|-------|
| 0x20 | TCA9535 | IO Expander (primary) |
| 0x39 | TCA9535 | IO Expander (alternate) |
| 0x38 | FT5x06 | Touch (DX variant) |
| 0x48 | FT6336U | Touch (GX variant) |
| 0x77 | BMP3xx | Pressure (D1Pro only) |

### RP2040 I2C Bus (GP20/21)

| Address | Device | Notes |
|---------|--------|-------|
| 0x38 | AHT20 | Temperature & Humidity |
| 0x59 | SGP40 | TVOC Sensor |
| 0x62 | SCD41 | CO2 Sensor |
| 0x?? | Grove | External I2C device |

---

## Code Examples

### RP2040 Complete Sensor Reading

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include <VOCGasIndexAlgorithm.h>
#include "AHT20.h"

#define SENSOR_POWER_PIN 18
#define BUZZER_PIN 19
#define I2C_SDA 20
#define I2C_SCL 21

AHT20 aht;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm voc_algorithm;

void setup() {
  Serial.begin(115200);

  // Power on sensors
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(100);

  // Initialize I2C
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  // Initialize sensors
  aht.begin();
  sgp40.begin(Wire);
  scd4x.begin(Wire, 0x62);
  scd4x.startPeriodicMeasurement();

  // Buzzer beep
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 127);
  delay(100);
  analogWrite(BUZZER_PIN, 0);
}

void loop() {
  // Read AHT20
  float humidity, temperature;
  if (aht.getSensor(&humidity, &temperature)) {
    Serial.printf("Temp: %.1f°C, Humidity: %.1f%%\n",
                  temperature, humidity * 100);
  }

  // Read SGP40
  uint16_t srawVoc;
  uint16_t compRh = (uint16_t)(humidity * 65535 / 100);
  uint16_t compT = (uint16_t)((temperature + 45) * 65535 / 175);
  if (sgp40.measureRawSignal(compRh, compT, srawVoc) == 0) {
    int32_t vocIndex = voc_algorithm.process(srawVoc);
    Serial.printf("VOC Index: %d\n", vocIndex);
  }

  // Read SCD41
  uint16_t co2;
  float scdTemp, scdHum;
  if (scd4x.readMeasurement(co2, scdTemp, scdHum) == 0 && co2 > 0) {
    Serial.printf("CO2: %d ppm\n", co2);
  }

  delay(5000);
}
```

### ESP32 LoRa Transmission Example

```c
#include "bsp_board.h"
#include "sx126x-board.h"
#include "radio.h"

void lora_init(void) {
    bsp_sx126x_init();

    Radio.Init(NULL);
    Radio.SetChannel(915000000);  // 915 MHz
    Radio.SetTxConfig(
        MODEM_LORA,
        14,              // Power dBm
        0,               // FSK deviation
        0,               // Bandwidth (125kHz)
        7,               // Spreading Factor
        1,               // Coding Rate 4/5
        8,               // Preamble length
        false,           // Fixed length
        true,            // CRC on
        false,           // Freq hop off
        0,               // Hop period
        false,           // IQ inversion
        3000             // Timeout
    );
}

void lora_send(uint8_t *data, uint8_t len) {
    Radio.Send(data, len);
}
```

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024 | Initial reverse-engineered documentation |

---

*This document was created by analyzing the official Seeed Studio source code repositories. No official schematic was available at time of creation.*
