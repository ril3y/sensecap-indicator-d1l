# SenseCAP Indicator D1L - Custom Firmware

PlatformIO project supporting both ESP32-S3 and RP2040 toolchains for the SenseCAP Indicator D1L.

## Project Structure

```
firmware/
├── platformio.ini          # PlatformIO configuration (all environments)
├── include/
│   ├── sensecap_pins.h     # Pin definitions for both MCUs
│   ├── sensecap_protocol.h # Inter-MCU communication protocol
│   └── lv_conf.h           # LVGL configuration
├── lib/
│   └── sensecap_common/    # Shared communication library
├── src/
│   ├── esp32/              # ESP32-S3 main firmware
│   └── rp2040/             # RP2040 main firmware
└── test/
    ├── esp32/              # ESP32 test/demo applications
    │   ├── test_lcd.cpp           # LCD display test
    │   ├── test_lvgl.cpp          # LVGL UI test (Arduino_GFX)
    │   ├── test_lvgl_lgfx.cpp     # LVGL UI test (LovyanGFX)
    │   ├── test_touch.cpp         # Touch panel test
    │   ├── test_button.cpp        # User button test
    │   ├── test_wifi.cpp          # WiFi connectivity test
    │   ├── test_io_expander.cpp   # TCA9535 IO expander test
    │   ├── test_uart_rp2040.cpp   # UART to RP2040 test
    │   └── test_esp32_rp2040.cpp  # Soundboard UI (buzzer control)
    └── rp2040/             # RP2040 test/demo applications
        ├── rp2040_d1l.cpp         # D1L firmware (SD, buzzer, Grove)
        ├── test_rp2040_full.cpp   # D1S/D1Pro firmware (with sensors)
        ├── test_uart_esp32.cpp    # UART to ESP32 test
        └── test_sensors.cpp       # Sensor test
```

## Model Variants

| Feature | D1 | D1S | D1L | D1Pro |
|---------|----|----|-----|-------|
| ESP32-S3 + Display | ✓ | ✓ | ✓ | ✓ |
| RP2040 Co-processor | ✓ | ✓ | ✓ | ✓ |
| SX1262 LoRa | ✓ | ✓ | ✓ | ✓ |
| AHT20 (Temp/Humidity) | ✗ | ✓ | ✗ | ✓ |
| SGP40 (TVOC) | ✗ | ✓ | ✗ | ✓ |
| SCD41 (CO2) | ✗ | ✗ | ✗ | ✓ |
| SD Card | ✓ | ✓ | ✓ | ✓ |
| Buzzer | ✓ | ✓ | ✓ | ✓ |

**This project targets the D1L model** (LoRa, no onboard sensors).

## Quick Start

### Prerequisites

1. Install [PlatformIO](https://platformio.org/install)
2. Install VS Code with PlatformIO extension (recommended)

### Build & Upload

```bash
# ESP32-S3 Soundboard UI
pio run -e esp32s3_rp2040_soundboard -t upload

# RP2040 D1L Firmware
pio run -e rp2040_d1l -t upload

# Monitor serial output
pio device monitor -p COM41   # ESP32
pio device monitor -p COM59   # RP2040
```

## Test Environments

### ESP32-S3 Tests

| Environment | Description | Status |
|-------------|-------------|--------|
| `esp32s3_test_lcd` | LCD display with Arduino_GFX | ✅ Working |
| `esp32s3_test_lvgl` | LVGL UI with Arduino_GFX | ✅ Working |
| `esp32s3_test_lvgl_lgfx` | LVGL UI with LovyanGFX | ✅ Working |
| `esp32s3_test_touch` | FT6336U touch controller | ✅ Working |
| `esp32s3_test_button` | User button (GPIO38) | ✅ Working |
| `esp32s3_test_wifi` | WiFi with on-screen keyboard | ✅ Working |
| `esp32s3_test_ioexp` | TCA9535 IO expander | ✅ Working |
| `esp32s3_rp2040_soundboard` | Buzzer control via UART | ✅ Working |
| `esp32s3_test_lora` | SX1262 LoRa radio | 🔴 Not Started |

### RP2040 Tests

| Environment | Description | Status |
|-------------|-------------|--------|
| `rp2040_d1l` | D1L firmware (SD, buzzer, Grove) | ✅ Working |
| `rp2040_full` | D1S/D1Pro with sensors | N/A (wrong model) |
| `rp2040_test_uart_esp32` | UART communication test | ✅ Working |

## Hardware Pin Assignments

### ESP32-S3

| Function | GPIO | Notes |
|----------|------|-------|
| LCD Data | 0-15 | 16-bit RGB565 |
| LCD HSYNC | 16 | |
| LCD VSYNC | 17 | |
| LCD DE | 18 | |
| LCD PCLK | 21 | |
| LCD Backlight | 45 | PWM capable |
| I2C SDA | 39 | Touch, IO Expander |
| I2C SCL | 40 | |
| SPI SCLK | 41 | LoRa |
| SPI MISO | 47 | LoRa |
| SPI MOSI | 48 | LoRa |
| **UART TX to RP2040** | **19** | |
| **UART RX from RP2040** | **20** | |
| User Button | 38 | Active low |
| IO Expander INT | 42 | TCA9535 |

### RP2040

| Function | GPIO | Notes |
|----------|------|-------|
| UART TX to ESP32 | 16 | -> ESP32 GPIO20 |
| UART RX from ESP32 | 17 | <- ESP32 GPIO19 |
| I2C SDA | 20 | Grove expansion |
| I2C SCL | 21 | |
| SD SCK | 10 | SPI1 |
| SD MOSI | 11 | |
| SD MISO | 12 | |
| SD CS | 13 | |
| SD Detect | 7 | Active low |
| Buzzer | 19 | PWM |
| Grove ADC0 | 26 | |
| Grove ADC1 | 27 | |

### IO Expander (TCA9535 @ 0x39)

| Pin | Function | Direction |
|-----|----------|-----------|
| IO0 | LCD Reset | Output |
| IO1 | LCD CS | Output |
| IO2 | LCD SCK | Output |
| IO3 | LCD MOSI | Output |
| IO4 | LoRa Reset | Output |
| IO5 | LoRa Busy | Input |
| IO6 | LoRa DIO1 | Input |
| IO7 | LoRa CS | Output |
| IO10 | Touch Reset | Output |

## RP2040 CLI Commands (D1L)

The RP2040 accepts text commands over USB serial or UART from ESP32:

```
Buzzer:
  beep [n] [freq] [ms]  - Beep n times
  tone <freq> <ms>      - Play tone
  melody <name>         - Play sound (tick, click, ok, msg, err, etc.)
  note <N> [ms]         - Play note (C4, A#5, Gb3)
  play <notes>          - Play sequence: C4:100 D4:100 E4:200
  vol [0-100]           - Get/set volume %
  quiet                 - Stop sound

SD Card:
  sd init               - Initialize SD
  ls [path]             - List directory
  cat <file>            - Read file
  write <file> <data>   - Write file
  mkdir <path>          - Create directory

Grove I2C:
  i2c scan              - Scan bus
  i2c read <addr> <reg> <len>

ADC:
  adc <0|1>             - Read Grove ADC

System:
  status                - System status
  boot                  - Reboot to UF2 bootloader
```

## Building for Different Tests

Each test has its own PlatformIO environment. To build and upload:

```bash
# LCD test
pio run -e esp32s3_test_lcd -t upload

# LVGL test (LovyanGFX recommended)
pio run -e esp32s3_test_lvgl_lgfx -t upload

# Soundboard (ESP32 UI + RP2040 buzzer)
pio run -e esp32s3_rp2040_soundboard -t upload  # ESP32
pio run -e rp2040_d1l -t upload                  # RP2040

# WiFi test
pio run -e esp32s3_test_wifi -t upload
```

## RP2040 First-Time Upload

The RP2040 requires UF2 bootloader mode for first upload:

1. Send `boot` command via serial, OR
2. Hold BOOTSEL button while pressing reset
3. A USB drive named "RPI-RP2" appears
4. Copy `.pio/build/rp2040_d1l/firmware.uf2` to the drive

After first flash, the `boot` command works for subsequent uploads.

## Next Steps

- [ ] SX1262 LoRa radio driver
- [ ] BLE connectivity
- [ ] Meshtastic integration

## License

MIT License
