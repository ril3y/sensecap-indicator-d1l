# SenseCAP Indicator D1L - Custom Firmware

A complete reference implementation for the SenseCAP Indicator D1L, demonstrating all hardware capabilities of both the ESP32-S3 and RP2040 microcontrollers.

## Hardware Overview

The SenseCAP Indicator D1L is a dual-MCU device:

| Component | Description |
|-----------|-------------|
| **ESP32-S3** | Main controller - Display, Touch, WiFi, BLE, LoRa |
| **RP2040** | Co-processor - SD Card, Buzzer, Grove I2C/ADC |
| **Display** | 4" 480x480 RGB LCD (ST7701S) with capacitive touch (FT6336U) |
| **Radio** | SX1262 LoRa (868/915/920 MHz) |
| **IO Expander** | TCA9535 (I2C GPIO expander for LCD/LoRa control) |

## Quick Start

### 1. Install Toolchain

**Windows:**
```cmd
# Install Python 3.x from python.org, then:
pip install platformio

# Or use the installer:
# Download from https://platformio.org/install/cli
```

**Linux (Ubuntu/Debian):**
```bash
# Install dependencies
sudo apt update
sudo apt install python3 python3-pip python3-venv

# Install PlatformIO
pip3 install platformio

# Add udev rules for USB access
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo usermod -a -G dialout $USER
# Log out and back in for group changes
```

**macOS:**
```bash
# Install Homebrew if not present
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python and PlatformIO
brew install python3
pip3 install platformio
```

### 2. Clone and Build

```bash
git clone <repository-url>
cd sensecap/firmware

# Install dependencies (downloads ESP32 and RP2040 toolchains)
make deps

# Build and upload (adjust COM ports in Makefile or use environment variables)
make esp32-lora ESP32_PORT=COM41    # Windows
make esp32-lora ESP32_PORT=/dev/ttyACM0  # Linux
make rp2040
```

### 3. Monitor Serial Output

```bash
make monitor-esp32   # ESP32 on COM41 / /dev/ttyACM0
make monitor-rp2040  # RP2040 on COM59 / /dev/ttyACM1
```

## Make Targets

| Target | Description |
|--------|-------------|
| `make help` | Show all available targets |
| `make install` | Install PlatformIO CLI |
| `make deps` | Download all toolchains and libraries |
| **ESP32-S3** | |
| `make esp32-lora` | LoRa packet scanner with LVGL UI |
| `make esp32-sound` | Soundboard UI (controls RP2040 buzzer) |
| `make esp32-lcd` | Basic LCD display test |
| `make esp32-lvgl` | LVGL UI framework test |
| `make esp32-touch` | Touch panel test |
| `make esp32-wifi` | WiFi with on-screen keyboard |
| `make esp32-button` | User button test |
| **RP2040** | |
| `make rp2040` | D1L firmware (SD, buzzer, Grove) |
| `make rp2040-uf2` | Build UF2 for manual flashing |
| **Utilities** | |
| `make monitor-esp32` | Serial monitor for ESP32 |
| `make monitor-rp2040` | Serial monitor for RP2040 |
| `make clean` | Clean build artifacts |
| `make ports` | List available serial ports |

## Project Structure

```
firmware/
├── Makefile                # Easy build commands
├── platformio.ini          # PlatformIO configuration
├── include/
│   ├── sensecap_pins.h     # Pin definitions
│   ├── lv_conf.h           # LVGL configuration
│   └── sensecap_protocol.h # Inter-MCU protocol
├── test/
│   ├── esp32/              # ESP32-S3 examples
│   │   ├── test_lora.cpp         # LoRa scanner (Meshtastic compatible)
│   │   ├── test_esp32_rp2040.cpp # Soundboard UI
│   │   ├── test_lcd.cpp          # LCD display
│   │   ├── test_lvgl.cpp         # LVGL with Arduino_GFX
│   │   ├── test_lvgl_lgfx.cpp    # LVGL with LovyanGFX
│   │   ├── test_touch.cpp        # Touch panel
│   │   ├── test_wifi.cpp         # WiFi connectivity
│   │   ├── test_button.cpp       # User button
│   │   └── test_io_expander.cpp  # TCA9535 test
│   └── rp2040/             # RP2040 examples
│       ├── rp2040_d1l.cpp        # Full D1L firmware
│       └── test_uart_esp32.cpp   # UART test
└── lib/
    └── sensecap_common/    # Shared libraries
```

## Test Status

### ESP32-S3

| Example | Description | Status |
|---------|-------------|--------|
| `esp32s3_test_lora` | SX1262 LoRa packet scanner | ✅ Working |
| `esp32s3_rp2040_soundboard` | Buzzer control via UART | ✅ Working |
| `esp32s3_test_lcd` | LCD display (Arduino_GFX) | ✅ Working |
| `esp32s3_test_lvgl` | LVGL UI (Arduino_GFX) | ✅ Working |
| `esp32s3_test_lvgl_lgfx` | LVGL UI (LovyanGFX) | ✅ Working |
| `esp32s3_test_touch` | FT6336U touch controller | ✅ Working |
| `esp32s3_test_button` | User button (GPIO38) | ✅ Working |
| `esp32s3_test_wifi` | WiFi with keyboard | ✅ Working |
| `esp32s3_test_ioexp` | TCA9535 IO expander | ✅ Working |

### RP2040

| Example | Description | Status |
|---------|-------------|--------|
| `rp2040_d1l` | SD card, buzzer, Grove, UART | ✅ Working |
| `rp2040_test_uart_esp32` | UART communication | ✅ Working |

## Hardware Pin Reference

### ESP32-S3 Pinout

| Function | GPIO | Notes |
|----------|------|-------|
| **LCD (RGB565 Parallel)** | | |
| LCD R0-R4 | 4,3,2,1,0 | Red channel |
| LCD G0-G5 | 10,9,8,7,6,5 | Green channel |
| LCD B0-B4 | 15,14,13,12,11 | Blue channel |
| LCD HSYNC | 16 | Horizontal sync |
| LCD VSYNC | 17 | Vertical sync |
| LCD DE | 18 | Data enable |
| LCD PCLK | 21 | Pixel clock |
| LCD Backlight | 45 | PWM capable |
| **I2C** | | |
| I2C SDA | 39 | Touch, IO Expander |
| I2C SCL | 40 | 400kHz |
| **SPI (LoRa)** | | |
| SPI SCLK | 41 | Also used for LCD init |
| SPI MISO | 47 | |
| SPI MOSI | 48 | Also used for LCD init |
| **UART (to RP2040)** | | |
| UART TX | 19 | → RP2040 GPIO17 |
| UART RX | 20 | ← RP2040 GPIO16 |
| **Other** | | |
| User Button | 38 | Active low, internal pullup |
| IO Expander INT | 42 | TCA9535 interrupt (active low, signals input change) |

### RP2040 Pinout

| Function | GPIO | Notes |
|----------|------|-------|
| **UART (to ESP32)** | | |
| UART TX | 16 | → ESP32 GPIO20 |
| UART RX | 17 | ← ESP32 GPIO19 |
| **SPI1 (SD Card)** | | |
| SD SCK | 10 | |
| SD MOSI | 11 | |
| SD MISO | 12 | |
| SD CS | 13 | |
| SD Detect | 7 | Active low |
| **Other** | | |
| Buzzer | 19 | PWM tone generation |
| Grove ADC0 | 26 | Analog input |
| Grove ADC1 | 27 | Analog input |
| Grove I2C SDA | 20 | External sensors |
| Grove I2C SCL | 21 | |

### IO Expander (TCA9535 @ 0x20)

The TCA9535 provides additional GPIO for LCD and LoRa control.

| Pin | Function | Direction | Used By |
|-----|----------|-----------|---------|
| IO0 | LoRa CS | Output | SX1262 chip select |
| IO1 | LoRa RST | Output | SX1262 reset |
| IO2 | LoRa BUSY | Input | SX1262 busy status |
| IO3 | LoRa DIO1 | Input | SX1262 interrupt |
| IO4 | LCD CS | Output | ST7701S chip select |
| IO5 | LCD RST | Output | ST7701S reset |
| IO7 | Touch RST | Output | FT6336U reset |

**Interrupt Output (INT):** The TCA9535 has an open-drain INT pin connected to **ESP32 GPIO42**. This pin goes LOW when any input (IO2, IO3) changes state. Reading the Input Port register clears the interrupt. This enables interrupt-driven LoRa packet detection instead of polling - see [firmware/docs/TCA9535_INTERRUPT_OPTIMIZATION.md](firmware/docs/TCA9535_INTERRUPT_OPTIMIZATION.md).

**Note:** LCD SPI data (SCK/MOSI) uses GPIO41/48 directly via bit-banging, not through the IO expander.

## RP2040 CLI Commands

The RP2040 firmware accepts text commands via USB serial or UART from ESP32:

```
Buzzer Commands:
  beep [n] [freq] [ms]  - Beep n times (default: 1x 1000Hz 100ms)
  tone <freq> <ms>      - Play single tone
  melody <name>         - Play preset: tick, click, ok, msg, err, warn, boot, etc.
  note <N> [ms]         - Play note: C4, A#5, Gb3 (default: 200ms)
  play <sequence>       - Play notes: "C4:100 D4:100 E4:200"
  vol [0-100]           - Get/set volume percentage
  quiet                 - Stop current sound

SD Card Commands:
  sd init               - Initialize SD card
  ls [path]             - List directory (default: /)
  cat <file>            - Display file contents
  write <file> <data>   - Write text to file
  mkdir <path>          - Create directory

Grove I2C Commands:
  i2c scan              - Scan for I2C devices
  i2c read <addr> <reg> <len>  - Read registers

ADC Commands:
  adc <0|1>             - Read Grove analog input

System Commands:
  status                - Show system status
  boot                  - Reboot to UF2 bootloader
```

## RP2040 First-Time Upload

The RP2040 requires UF2 bootloader mode for the first flash:

1. **Enter bootloader mode:**
   - Hold BOOTSEL button while pressing reset, OR
   - Send `boot` command via serial (if firmware is running)

2. **Flash the firmware:**
   - A USB drive named `RPI-RP2` appears
   - Copy `.pio/build/rp2040_d1l/firmware.uf2` to the drive
   - Device reboots automatically

After first flash, use `make rp2040` for subsequent uploads.

## LoRa Configuration

The LoRa scanner is configured for Meshtastic compatibility:

| Parameter | Value |
|-----------|-------|
| Frequency | 906.875 MHz (US primary) |
| Spreading Factor | SF11 |
| Bandwidth | 250 kHz |
| Coding Rate | 4/5 |
| Sync Word | 0x2B |
| Preset | LongFast |

Additional frequencies available via UI: 903.08 MHz, 915 MHz, 868 MHz.

## Model Variants

| Feature | D1 | D1S | D1L | D1Pro |
|---------|:--:|:---:|:---:|:-----:|
| ESP32-S3 + Display | ✓ | ✓ | ✓ | ✓ |
| RP2040 Co-processor | ✓ | ✓ | ✓ | ✓ |
| SX1262 LoRa | ✓ | ✓ | ✓ | ✓ |
| AHT20 (Temp/Humidity) | ✗ | ✓ | ✗ | ✓ |
| SGP40 (TVOC) | ✗ | ✓ | ✗ | ✓ |
| SCD41 (CO2) | ✗ | ✗ | ✗ | ✓ |
| SD Card | ✓ | ✓ | ✓ | ✓ |
| Buzzer | ✓ | ✓ | ✓ | ✓ |

**This project targets the D1L model** (LoRa, no onboard sensors).

## Troubleshooting

### PlatformIO not found
```bash
# Add to PATH (Linux/macOS)
export PATH="$HOME/.platformio/penv/bin:$PATH"

# Or use full path
~/.platformio/penv/bin/pio run -e esp32s3_test_lora
```

### Serial port permission denied (Linux)
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### ESP32 not detected
- Try different USB cable (data cable, not charge-only)
- Check Device Manager (Windows) or `ls /dev/ttyACM*` (Linux)

### RP2040 not detected
- Enter bootloader mode (hold BOOTSEL + reset)
- Look for `RPI-RP2` USB drive

### Build fails with missing dependencies
```bash
make clean
make deps
```

## Next Steps

- [ ] BLE connectivity
- [ ] Meshtastic protocol integration
- [ ] Power management / sleep modes
- [ ] OTA firmware updates

## License

MIT License
