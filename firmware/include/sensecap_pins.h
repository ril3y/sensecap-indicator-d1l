/**
 * @file sensecap_pins.h
 * @brief Pin definitions for SenseCAP Indicator D1L
 *
 * This header provides unified pin definitions for both ESP32-S3 and RP2040.
 * Include this file in your application code for hardware abstraction.
 */

#ifndef SENSECAP_PINS_H
#define SENSECAP_PINS_H

#ifdef ESP32_MAIN_MCU
// ============================================================================
// ESP32-S3 Pin Definitions
// ============================================================================

// LCD RGB Data Bus (directly from GPIO to LCD)
#define PIN_LCD_R0          4
#define PIN_LCD_R1          3
#define PIN_LCD_R2          2
#define PIN_LCD_R3          1
#define PIN_LCD_R4          0   // MSB
#define PIN_LCD_G0          10
#define PIN_LCD_G1          9
#define PIN_LCD_G2          8
#define PIN_LCD_G3          7
#define PIN_LCD_G4          6
#define PIN_LCD_G5          5   // MSB
#define PIN_LCD_B0          15
#define PIN_LCD_B1          14
#define PIN_LCD_B2          13
#define PIN_LCD_B3          12
#define PIN_LCD_B4          11  // MSB

// LCD Control Signals
#define PIN_LCD_HSYNC       16
#define PIN_LCD_VSYNC       17
#define PIN_LCD_DE          18
#define PIN_LCD_PCLK        21
#define PIN_LCD_BL          45  // Backlight, HIGH = ON

// LCD Timing Parameters
#define LCD_WIDTH           480
#define LCD_HEIGHT          480
#define LCD_FREQ_HZ         14000000
#define LCD_HSYNC_BP        50
#define LCD_HSYNC_FP        10
#define LCD_HSYNC_PW        8
#define LCD_VSYNC_BP        20
#define LCD_VSYNC_FP        10
#define LCD_VSYNC_PW        8

// I2C Bus (Touch, IO Expander, BMP3xx)
#define PIN_I2C_SDA         39
#define PIN_I2C_SCL         40

// SPI Bus (LoRa Radio)
#define PIN_SPI_MOSI        48
#define PIN_SPI_MISO        47
#define PIN_SPI_SCLK        41

// IO Expander Interrupt (TCA9535)
// INT is open-drain, active-low. Goes LOW when any INPUT pin changes.
// Reading the Input Port register clears the interrupt.
// Use this for interrupt-driven LoRa packet detection instead of polling.
#define PIN_IO_EXP_INT      42

// UART to RP2040
#define PIN_UART_TX         43
#define PIN_UART_RX         44
#define UART_BAUD_RATE      115200

// User Button
#define PIN_USER_BUTTON     38
#define BUTTON_ACTIVE_LOW   true

// I2C Addresses
#define I2C_ADDR_IO_EXPANDER    0x20
#define I2C_ADDR_IO_EXPANDER_ALT 0x39
#define I2C_ADDR_TOUCH_GX       0x48    // FT6336U
#define I2C_ADDR_TOUCH_DX       0x38    // FT5x06
#define I2C_ADDR_BMP3XX         0x77    // D1Pro only

// IO Expander Pin Assignments (TCA9535)
// Pins marked [INPUT] trigger the INT signal on GPIO42 when they change
#define IOEXP_LORA_NSS      0   // Output: SX1262 chip select
#define IOEXP_LORA_RST      1   // Output: SX1262 reset
#define IOEXP_LORA_BUSY     2   // [INPUT]: SX1262 busy status - triggers INT
#define IOEXP_LORA_DIO1     3   // [INPUT]: SX1262 packet RX/TX done - triggers INT
#define IOEXP_LCD_CS        4   // Output: ST7701S chip select
#define IOEXP_LCD_RST       5   // Output: ST7701S reset
#define IOEXP_TOUCH_RST     7   // Output: FT6336U reset
#define IOEXP_RP2040_RST    8   // Output: RP2040 reset (active low)
#define IOEXP_BMP_PWR       10  // Output: BMP3xx power (D1Pro only)
#define IOEXP_TCXO_VER      11  // [INPUT]: TCXO version detect - triggers INT

#endif // ESP32_MAIN_MCU

#ifdef RP2040_SENSOR_HUB
// ============================================================================
// RP2040 Pin Definitions
// ============================================================================

// UART to ESP32
#define PIN_UART_TX         16
#define PIN_UART_RX         17
#define UART_BAUD_RATE      115200

// I2C Bus (Sensors)
#define PIN_I2C_SDA         20
#define PIN_I2C_SCL         21

// SPI1 Bus (SD Card)
#define PIN_SD_SCK          10
#define PIN_SD_MOSI         11
#define PIN_SD_MISO         12
#define PIN_SD_CS           13
#define SD_SPI_SPEED        1000000

// Sensor Power Control
#define PIN_SENSOR_POWER    18
#define SENSOR_POWER_ON     HIGH
#define SENSOR_POWER_OFF    LOW

// Buzzer
#define PIN_BUZZER          19

// Grove ADC Inputs
#define PIN_GROVE_ADC0      26
#define PIN_GROVE_ADC1      27

// I2C Addresses (Sensors)
#define I2C_ADDR_AHT20      0x38
#define I2C_ADDR_SGP40      0x59
#define I2C_ADDR_SCD41      0x62

// Unused GPIOs (available for extension)
#define PIN_UNUSED_0        0
#define PIN_UNUSED_1        1
#define PIN_UNUSED_2        2
#define PIN_UNUSED_3        3
#define PIN_UNUSED_4        4
#define PIN_UNUSED_5        5
#define PIN_UNUSED_6        6
#define PIN_UNUSED_7        7
#define PIN_UNUSED_8        8
#define PIN_UNUSED_9        9
#define PIN_UNUSED_14       14
#define PIN_UNUSED_15       15
#define PIN_UNUSED_22       22
#define PIN_UNUSED_23       23
#define PIN_UNUSED_24       24
#define PIN_UNUSED_25       25  // Often onboard LED
#define PIN_UNUSED_28       28  // ADC2
#define PIN_UNUSED_29       29  // ADC3

#endif // RP2040_SENSOR_HUB

#endif // SENSECAP_PINS_H
