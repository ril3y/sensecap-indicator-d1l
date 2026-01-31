# SenseCAP Indicator D1L - Reverse-Engineered Pinout Schematic

*Derived from source code analysis of ESP32 and RP2040 firmware repositories*

## System Architecture

```
+------------------+          UART (115200)          +------------------+
|                  |  TX(GPIO43) -----> RX(GP17)    |                  |
|   ESP32-S3       |  RX(GPIO44) <----- TX(GP16)    |     RP2040       |
|   (Main MCU)     |                                 |   (Sensor Hub)   |
|                  |       [PacketSerial Protocol]   |                  |
+------------------+                                 +------------------+
        |                                                    |
        | I2C (GPIO39/40)                                   | I2C (GP20/21)
        |                                                    |
   +----+----+                                          +----+----+
   |         |                                          |         |
[TCA9535]  [Touch]                                   [AHT20]  [SGP40]
IO Expand   Panel                                    Temp/Hum   TVOC
   |        FT6336                                       |
   |        @0x48                                     [SCD41]
   +---> [SX1262 LoRa Radio]                           CO2
```

---

## ESP32-S3 GPIO Assignments

### RGB LCD Interface (16-bit parallel RGB)
| Function | GPIO | Notes |
|----------|------|-------|
| VSYNC | GPIO17 | Vertical sync |
| HSYNC | GPIO16 | Horizontal sync |
| DE | GPIO18 | Data enable |
| PCLK | GPIO21 | Pixel clock |
| BL (Backlight) | GPIO45 | Active HIGH |
| B0 | GPIO15 | Blue bit 0 |
| B1 | GPIO14 | Blue bit 1 |
| B2 | GPIO13 | Blue bit 2 |
| B3 | GPIO12 | Blue bit 3 |
| B4 | GPIO11 | Blue bit 4 |
| G0 | GPIO10 | Green bit 0 |
| G1 | GPIO9 | Green bit 1 |
| G2 | GPIO8 | Green bit 2 |
| G3 | GPIO7 | Green bit 3 |
| G4 | GPIO6 | Green bit 4 |
| G5 | GPIO5 | Green bit 5 |
| R0 | GPIO4 | Red bit 0 |
| R1 | GPIO3 | Red bit 1 |
| R2 | GPIO2 | Red bit 2 |
| R3 | GPIO1 | Red bit 3 |
| R4 | GPIO0 | Red bit 4 |

**LCD Timing (ST7701 controller):**
- HSYNC Back Porch: 50
- HSYNC Front Porch: 10
- HSYNC Pulse Width: 8
- VSYNC Back Porch: 20
- VSYNC Front Porch: 10
- VSYNC Pulse Width: 8

### I2C Bus (ESP32-S3)
| Function | GPIO | Notes |
|----------|------|-------|
| SCL | GPIO40 | I2C Clock |
| SDA | GPIO39 | I2C Data |

**I2C Devices on ESP32 Bus:**
| Device | Address | Function |
|--------|---------|----------|
| TCA9535 | 0x20 or 0x39 | 16-bit IO Expander |
| FT6336U | 0x48 (GX variant) | Touch Controller |
| FT5x06 | 0x38 (DX variant) | Touch Controller |
| BMP3xx | 0x77 | Pressure/Temp (WXM model) |

### SPI Bus (LoRa Radio - SX1262)
| Function | GPIO | Notes |
|----------|------|-------|
| MOSI | GPIO48 | Master Out |
| MISO | GPIO47 | Master In |
| SCLK | GPIO41 | SPI Clock |
| IO Expander INT | GPIO42 | Interrupt from TCA9535 |

### User Input
| Function | GPIO | Notes |
|----------|------|-------|
| User Button | GPIO38 | Active LOW |

### UART to RP2040
| Function | GPIO | Notes |
|----------|------|-------|
| TX | GPIO43 | To RP2040 RX (GP17) |
| RX | GPIO44 | From RP2040 TX (GP16) |

---

## TCA9535 IO Expander Pin Assignments

The TCA9535 provides 16 additional GPIOs controlled via I2C.

### LoRa Radio (SX1262) Control
| Expander Pin | Function | Direction |
|--------------|----------|-----------|
| IO0 | RADIO_NSS | Output (SPI CS) |
| IO1 | RADIO_RST | Output (Reset) |
| IO2 | RADIO_BUSY | Input |
| IO3 | RADIO_DIO_1 | Input (IRQ) |
| IO11 | RADIO_VER | Input (TCXO detect) |

### LCD/Touch Control
| Expander Pin | Function | Direction |
|--------------|----------|-----------|
| IO4 | LCD_CS | Output |
| IO5 | LCD_RESET | Output |
| IO7 | TP_RESET | Output (Touch Reset) |

### Other Control
| Expander Pin | Function | Direction |
|--------------|----------|-----------|
| IO8 | RP2040_RESET | Output |
| IO10 | BMP_PWR | Output (BMP sensor power) |

---

## RP2040 GPIO Assignments

### UART to ESP32
| Function | GPIO | Notes |
|----------|------|-------|
| RX | GP17 | From ESP32 TX |
| TX | GP16 | To ESP32 RX |

### I2C Bus (Sensors)
| Function | GPIO | Notes |
|----------|------|-------|
| SDA | GP20 | I2C Data |
| SCL | GP21 | I2C Clock |

**I2C Devices on RP2040 Bus:**
| Device | Address | Function |
|--------|---------|----------|
| AHT20 | 0x38 | Temperature & Humidity |
| SGP40 | 0x59 | TVOC Sensor |
| SCD41 | 0x62 | CO2 Sensor |

### SPI Bus (SD Card)
| Function | GPIO | Notes |
|----------|------|-------|
| SCK | GP10 | SPI Clock (SPI1) |
| TX/MOSI | GP11 | Master Out (SPI1) |
| RX/MISO | GP12 | Master In (SPI1) |
| CS | GP13 | Chip Select |

### Sensor Power Control
| Function | GPIO | Notes |
|----------|------|-------|
| Sensor Power | GP18 | HIGH = ON |

### Buzzer
| Function | GPIO | Notes |
|----------|------|-------|
| Buzzer | GP19 | PWM Output |

### Grove ADC Inputs
| Function | GPIO | Notes |
|----------|------|-------|
| Grove ADC 0 | GP26 | ADC Input |
| Grove ADC 1 | GP27 | ADC Input |

---

## Inter-MCU Communication Protocol

**Protocol:** PacketSerial (COBS encoded) @ 115200 baud

### Packet Types (RP2040 -> ESP32)
| Type Code | Name | Data |
|-----------|------|------|
| 0xB2 | CO2 | float (ppm) |
| 0xB3 | Temperature | float (C) |
| 0xB4 | Humidity | float (%) |
| 0xB5 | TVOC Index | float |

### Command Types (ESP32 -> RP2040)
| Type Code | Name | Function |
|-----------|------|----------|
| 0xA0 | Collect Interval | Set sensor polling rate |
| 0xA1 | Beep On | Trigger buzzer |
| 0xA3 | Shutdown | Power off sensors |

---

## Block Diagram

```
                           +---------------------------+
                           |     SenseCAP Indicator    |
                           +---------------------------+
                                        |
           +----------------------------+----------------------------+
           |                                                         |
    +------+------+                                           +------+------+
    |  ESP32-S3   |                                           |   RP2040    |
    |  (8MB Flash)|                                           | (2MB Flash) |
    +-------------+                                           +-------------+
           |                                                         |
    +------+------+------+------+                    +---------------+---------------+
    |      |      |      |      |                    |       |       |       |       |
   RGB    SPI    I2C   UART   WiFi/BT              I2C     SPI    GPIO    UART    ADC
   LCD   LoRa   Touch  RP2040  Antenna            Sensors   SD    Buzzer  ESP32   Grove
    |      |      |      |                          |       |       |       |       |
    v      v      v      v                          v       v       v       v       v
 +-----+ +----+ +-----+ +--+                     +-----+ +---+ +---+ +--+ +----+
 |4"TFT| |SX  | |FT   | |  |                     |AHT20| |SD | |PWM| |  | |ADC |
 |480x | |1262| |6336U| |  |                     |SGP40| |   | |   | |  | |x2  |
 |480  | |    | |     | |  |                     |SCD41| |   | |   | |  | |    |
 +-----+ +----+ +-----+ +--+                     +-----+ +---+ +---+ +--+ +----+

                           TCA9535 IO Expander
                    +--------------------------------+
                    |  IO0: LoRa NSS (CS)            |
                    |  IO1: LoRa RST                 |
                    |  IO2: LoRa BUSY                |
                    |  IO3: LoRa DIO1 (IRQ)          |
                    |  IO4: LCD CS                   |
                    |  IO5: LCD RST                  |
                    |  IO7: Touch RST                |
                    |  IO8: RP2040 RST               |
                    |  IO10: BMP Sensor Power        |
                    |  IO11: TCXO Version Detect     |
                    +--------------------------------+
```

---

## Connector Pinouts

### Grove Connectors (on RP2040 side)

**Grove I2C (4-pin JST):**
| Pin | Signal |
|-----|--------|
| 1 | GND |
| 2 | VCC (3.3V) |
| 3 | SDA (GP20) |
| 4 | SCL (GP21) |

**Grove ADC (4-pin JST):**
| Pin | Signal |
|-----|--------|
| 1 | GND |
| 2 | VCC (3.3V) |
| 3 | A0 (GP26) |
| 4 | A1 (GP27) |

---

## Model Variants

| Model | LoRa | BMP3xx Sensor | Notes |
|-------|------|---------------|-------|
| D1 | No | No | Base model |
| D1L | Yes (SX1262) | No | LoRa enabled |
| D1Pro | Yes (SX1262) | Yes | Full sensor suite |

---

## Voltage Levels

- Logic Level: 3.3V
- USB Power: 5V DC, 1A
- LoRa TCXO Voltage: 2.4V (if equipped)

---

## Notes

1. **Screen Variants:**
   - GX: Uses FT6336U touch @ 0x48, ST7701 LCD controller
   - DX: Uses FT5x06 touch @ 0x38

2. **LoRa Radio Access:** The SX1262 is accessed via SPI through the ESP32, but CS/RST/BUSY/DIO1 signals are routed through the TCA9535 IO expander.

3. **ESP32 controls RP2040 reset** via IO expander pin IO8.

4. **Sensor power** on RP2040 side is controlled via GP18 (HIGH = enabled).

5. **PacketSerial library** is used for reliable framed UART communication between MCUs.
