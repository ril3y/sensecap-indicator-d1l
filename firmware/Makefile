# SenseCAP Indicator D1L Firmware - Makefile
# Wrapper for PlatformIO commands with easy-to-remember targets

# Serial ports (adjust for your system)
ESP32_PORT ?= COM41
RP2040_PORT ?= COM59

# Default target
.PHONY: help
help:
	@echo "SenseCAP Indicator D1L Firmware Build System"
	@echo "============================================="
	@echo ""
	@echo "Setup:"
	@echo "  make install        - Install PlatformIO CLI"
	@echo "  make deps           - Download all dependencies"
	@echo ""
	@echo "ESP32-S3 Targets:"
	@echo "  make esp32-lora     - Build & upload LoRa scanner"
	@echo "  make esp32-sound    - Build & upload Soundboard UI"
	@echo "  make esp32-lcd      - Build & upload LCD test"
	@echo "  make esp32-lvgl     - Build & upload LVGL test"
	@echo "  make esp32-touch    - Build & upload Touch test"
	@echo "  make esp32-wifi     - Build & upload WiFi test"
	@echo "  make esp32-button   - Build & upload Button test"
	@echo "  make esp32-ioexp    - Build & upload IO Expander test"
	@echo ""
	@echo "RP2040 Targets:"
	@echo "  make rp2040         - Build & upload D1L firmware"
	@echo "  make rp2040-uart    - Build & upload UART test"
	@echo ""
	@echo "Monitoring:"
	@echo "  make monitor-esp32  - Monitor ESP32 serial ($(ESP32_PORT))"
	@echo "  make monitor-rp2040 - Monitor RP2040 serial ($(RP2040_PORT))"
	@echo ""
	@echo "Build Only (no upload):"
	@echo "  make build-esp32-lora"
	@echo "  make build-rp2040"
	@echo "  make build-all"
	@echo ""
	@echo "Utilities:"
	@echo "  make clean          - Clean all build artifacts"
	@echo "  make list           - List available environments"
	@echo "  make ports          - List available serial ports"

# ============================================================================
# Setup
# ============================================================================

.PHONY: install
install:
	@echo "Installing PlatformIO CLI..."
ifeq ($(OS),Windows_NT)
	pip install platformio
else
	pip3 install platformio
endif
	@echo "Done! Run 'make deps' to download dependencies."

.PHONY: deps
deps:
	@echo "Downloading dependencies for all environments..."
	pio pkg install
	@echo "Done!"

# ============================================================================
# ESP32-S3 Targets
# ============================================================================

.PHONY: esp32-lora
esp32-lora:
	pio run -e esp32s3_test_lora -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-sound
esp32-sound:
	pio run -e esp32s3_rp2040_soundboard -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-lcd
esp32-lcd:
	pio run -e esp32s3_test_lcd -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-lvgl
esp32-lvgl:
	pio run -e esp32s3_test_lvgl_lgfx -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-touch
esp32-touch:
	pio run -e esp32s3_test_touch -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-wifi
esp32-wifi:
	pio run -e esp32s3_test_wifi -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-button
esp32-button:
	pio run -e esp32s3_test_button -t upload --upload-port $(ESP32_PORT)

.PHONY: esp32-ioexp
esp32-ioexp:
	pio run -e esp32s3_test_ioexp -t upload --upload-port $(ESP32_PORT)

# ============================================================================
# RP2040 Targets
# ============================================================================

.PHONY: rp2040
rp2040:
	pio run -e rp2040_d1l -t upload

.PHONY: rp2040-uart
rp2040-uart:
	pio run -e rp2040_test_uart_esp32 -t upload

.PHONY: rp2040-uf2
rp2040-uf2:
	pio run -e rp2040_d1l
	@echo ""
	@echo "UF2 file ready: .pio/build/rp2040_d1l/firmware.uf2"
	@echo "Copy to RPI-RP2 drive to flash."

# ============================================================================
# Monitoring
# ============================================================================

.PHONY: monitor-esp32
monitor-esp32:
	pio device monitor -p $(ESP32_PORT) -b 115200

.PHONY: monitor-rp2040
monitor-rp2040:
	pio device monitor -p $(RP2040_PORT) -b 115200

# ============================================================================
# Build Only (no upload)
# ============================================================================

.PHONY: build-esp32-lora
build-esp32-lora:
	pio run -e esp32s3_test_lora

.PHONY: build-esp32-sound
build-esp32-sound:
	pio run -e esp32s3_rp2040_soundboard

.PHONY: build-rp2040
build-rp2040:
	pio run -e rp2040_d1l

.PHONY: build-all
build-all:
	pio run -e esp32s3_test_lora
	pio run -e esp32s3_rp2040_soundboard
	pio run -e rp2040_d1l

# ============================================================================
# Utilities
# ============================================================================

.PHONY: clean
clean:
	pio run -t clean
	rm -rf .pio

.PHONY: list
list:
	pio project config --list-targets

.PHONY: ports
ports:
	pio device list
