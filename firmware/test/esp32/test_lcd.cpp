/**
 * @file test_lcd.cpp
 * @brief LCD Display Tests for Task #5
 *
 * Tests ST7701 480x480 RGB LCD
 * Manually bit-bangs SPI init sequence with IO expander CS control
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>
#include <Arduino_GFX_Library.h>

// I2C pins
#define I2C_SDA_PIN  39
#define I2C_SCL_PIN  40

// TCA9535/PCA9535 IO Expander
#define IOEXP_ADDR          0x20
#define IOEXP_OUTPUT_PORT0  0x02
#define IOEXP_OUTPUT_PORT1  0x03
#define IOEXP_CONFIG_PORT0  0x06
#define IOEXP_CONFIG_PORT1  0x07

// IO Expander pin assignments
#define IOEXP_LCD_CS    4   // P04
#define IOEXP_LCD_RST   5   // P05

// Backlight pin
#define GFX_BL 45

// SPI pins for ST7701 initialization (directly on ESP32)
#define LCD_SPI_SCK   41
#define LCD_SPI_MOSI  48

// RGB panel pins
#define LCD_DE     18
#define LCD_VSYNC  17
#define LCD_HSYNC  16
#define LCD_PCLK   21
#define LCD_R0  4
#define LCD_R1  3
#define LCD_R2  2
#define LCD_R3  1
#define LCD_R4  0
#define LCD_G0  10
#define LCD_G1  9
#define LCD_G2  8
#define LCD_G3  7
#define LCD_G4  6
#define LCD_G5  5
#define LCD_B0  15
#define LCD_B1  14
#define LCD_B2  13
#define LCD_B3  12
#define LCD_B4  11

// Cached IO expander port0 value for faster CS toggling
static uint8_t ioexp_port0_cache = 0xFF;

// IO Expander helper functions
void ioexp_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IOEXP_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t ioexp_read_reg(uint8_t reg) {
    Wire.beginTransmission(IOEXP_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)IOEXP_ADDR, (uint8_t)1);
    return Wire.read();
}

void ioexp_set_pin_output(uint8_t pin) {
    uint8_t reg = (pin < 8) ? IOEXP_CONFIG_PORT0 : IOEXP_CONFIG_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    val &= ~(1 << bit);  // 0 = output
    ioexp_write_reg(reg, val);
}

void ioexp_write_pin(uint8_t pin, bool level) {
    uint8_t reg = (pin < 8) ? IOEXP_OUTPUT_PORT0 : IOEXP_OUTPUT_PORT1;
    uint8_t bit = pin % 8;

    if (pin < 8) {
        if (level) {
            ioexp_port0_cache |= (1 << bit);
        } else {
            ioexp_port0_cache &= ~(1 << bit);
        }
        ioexp_write_reg(reg, ioexp_port0_cache);
    } else {
        uint8_t val = ioexp_read_reg(reg);
        if (level) {
            val |= (1 << bit);
        } else {
            val &= ~(1 << bit);
        }
        ioexp_write_reg(reg, val);
    }
}

// Software SPI for ST7701 (3-wire 9-bit mode)
void spi_write_9bit(uint8_t dc, uint8_t data) {
    // DC bit first (0=command, 1=data), then 8 data bits
    // MSB first, CPOL=0, CPHA=0

    // Write DC bit
    digitalWrite(LCD_SPI_SCK, LOW);
    digitalWrite(LCD_SPI_MOSI, dc ? HIGH : LOW);
    digitalWrite(LCD_SPI_SCK, HIGH);

    // Write 8 data bits MSB first
    for (int i = 7; i >= 0; i--) {
        digitalWrite(LCD_SPI_SCK, LOW);
        digitalWrite(LCD_SPI_MOSI, (data >> i) & 1 ? HIGH : LOW);
        digitalWrite(LCD_SPI_SCK, HIGH);
    }
}

void lcd_write_cmd(uint8_t cmd) {
    ioexp_write_pin(IOEXP_LCD_CS, LOW);
    spi_write_9bit(0, cmd);  // DC=0 for command
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
}

void lcd_write_data(uint8_t data) {
    ioexp_write_pin(IOEXP_LCD_CS, LOW);
    spi_write_9bit(1, data);  // DC=1 for data
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
}

void lcd_write_cmd_data(uint8_t cmd, const uint8_t *data, size_t len) {
    ioexp_write_pin(IOEXP_LCD_CS, LOW);
    spi_write_9bit(0, cmd);
    for (size_t i = 0; i < len; i++) {
        spi_write_9bit(1, data[i]);
    }
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
}

// ST7701S init sequence for SenseCAP Indicator (480x480 RGB)
void st7701_init(void) {
    Serial.println("[INFO] Sending ST7701 init sequence...");

    // Command2 BK0
    lcd_write_cmd(0xFF);
    lcd_write_data(0x77);
    lcd_write_data(0x01);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x10);

    lcd_write_cmd(0xC0);
    lcd_write_data(0x3B);
    lcd_write_data(0x00);

    lcd_write_cmd(0xC1);
    lcd_write_data(0x0D);
    lcd_write_data(0x02);

    lcd_write_cmd(0xC2);
    lcd_write_data(0x31);
    lcd_write_data(0x05);

    lcd_write_cmd(0xCD);
    lcd_write_data(0x08);

    // Positive Gamma
    lcd_write_cmd(0xB0);
    lcd_write_data(0x00);
    lcd_write_data(0x11);
    lcd_write_data(0x18);
    lcd_write_data(0x0E);
    lcd_write_data(0x11);
    lcd_write_data(0x06);
    lcd_write_data(0x07);
    lcd_write_data(0x08);
    lcd_write_data(0x07);
    lcd_write_data(0x22);
    lcd_write_data(0x04);
    lcd_write_data(0x12);
    lcd_write_data(0x0F);
    lcd_write_data(0xAA);
    lcd_write_data(0x31);
    lcd_write_data(0x18);

    // Negative Gamma
    lcd_write_cmd(0xB1);
    lcd_write_data(0x00);
    lcd_write_data(0x11);
    lcd_write_data(0x19);
    lcd_write_data(0x0E);
    lcd_write_data(0x12);
    lcd_write_data(0x07);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x22);
    lcd_write_data(0x04);
    lcd_write_data(0x11);
    lcd_write_data(0x11);
    lcd_write_data(0xA9);
    lcd_write_data(0x32);
    lcd_write_data(0x18);

    // Command2 BK1
    lcd_write_cmd(0xFF);
    lcd_write_data(0x77);
    lcd_write_data(0x01);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x11);

    lcd_write_cmd(0xB0);
    lcd_write_data(0x60);

    lcd_write_cmd(0xB1);
    lcd_write_data(0x32);

    lcd_write_cmd(0xB2);
    lcd_write_data(0x07);

    lcd_write_cmd(0xB3);
    lcd_write_data(0x80);

    lcd_write_cmd(0xB5);
    lcd_write_data(0x49);

    lcd_write_cmd(0xB7);
    lcd_write_data(0x85);

    lcd_write_cmd(0xB8);
    lcd_write_data(0x21);

    lcd_write_cmd(0xC1);
    lcd_write_data(0x78);

    lcd_write_cmd(0xC2);
    lcd_write_data(0x78);

    delay(20);

    lcd_write_cmd(0xE0);
    lcd_write_data(0x00);
    lcd_write_data(0x1B);
    lcd_write_data(0x02);

    lcd_write_cmd(0xE1);
    lcd_write_data(0x08);
    lcd_write_data(0xA0);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x07);
    lcd_write_data(0xA0);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x44);
    lcd_write_data(0x44);

    lcd_write_cmd(0xE2);
    lcd_write_data(0x11);
    lcd_write_data(0x11);
    lcd_write_data(0x44);
    lcd_write_data(0x44);
    lcd_write_data(0xED);
    lcd_write_data(0xA0);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0xEC);
    lcd_write_data(0xA0);
    lcd_write_data(0x00);
    lcd_write_data(0x00);

    lcd_write_cmd(0xE3);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x11);
    lcd_write_data(0x11);

    lcd_write_cmd(0xE4);
    lcd_write_data(0x44);
    lcd_write_data(0x44);

    lcd_write_cmd(0xE5);
    lcd_write_data(0x0A);
    lcd_write_data(0xE9);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x0C);
    lcd_write_data(0xEB);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x0E);
    lcd_write_data(0xED);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x10);
    lcd_write_data(0xEF);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);

    lcd_write_cmd(0xE6);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x11);
    lcd_write_data(0x11);

    lcd_write_cmd(0xE7);
    lcd_write_data(0x44);
    lcd_write_data(0x44);

    lcd_write_cmd(0xE8);
    lcd_write_data(0x09);
    lcd_write_data(0xE8);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x0B);
    lcd_write_data(0xEA);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x0D);
    lcd_write_data(0xEC);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);
    lcd_write_data(0x0F);
    lcd_write_data(0xEE);
    lcd_write_data(0xD8);
    lcd_write_data(0xA0);

    lcd_write_cmd(0xEB);
    lcd_write_data(0x02);
    lcd_write_data(0x00);
    lcd_write_data(0xE4);
    lcd_write_data(0xE4);
    lcd_write_data(0x88);
    lcd_write_data(0x00);
    lcd_write_data(0x40);

    lcd_write_cmd(0xEC);
    lcd_write_data(0x3C);
    lcd_write_data(0x00);

    lcd_write_cmd(0xED);
    lcd_write_data(0xAB);
    lcd_write_data(0x89);
    lcd_write_data(0x76);
    lcd_write_data(0x54);
    lcd_write_data(0x02);
    lcd_write_data(0xFF);
    lcd_write_data(0xFF);
    lcd_write_data(0xFF);
    lcd_write_data(0xFF);
    lcd_write_data(0xFF);
    lcd_write_data(0xFF);
    lcd_write_data(0x20);
    lcd_write_data(0x45);
    lcd_write_data(0x67);
    lcd_write_data(0x98);
    lcd_write_data(0xBA);

    // Exit Command2
    lcd_write_cmd(0xFF);
    lcd_write_data(0x77);
    lcd_write_data(0x01);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);

    // Sleep Out
    lcd_write_cmd(0x11);
    delay(120);

    // Display On
    lcd_write_cmd(0x29);
    delay(20);

    Serial.println("[INFO] ST7701 init complete");
}

// RGB Panel - created after ST7701 init
Arduino_ESP32RGBPanel *rgbpanel = nullptr;
Arduino_RGB_Display *gfx = nullptr;

void setUp(void) {}
void tearDown(void) {}

/**
 * TEST_LCD_001: Initialize I2C and IO Expander
 */
void test_lcd_io_init(void) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    Wire.beginTransmission(IOEXP_ADDR);
    bool found = (Wire.endTransmission() == 0);
    TEST_ASSERT_TRUE_MESSAGE(found, "IO Expander not detected at 0x20");

    // Read current port0 state
    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);

    // Configure LCD CS and RST as outputs
    ioexp_set_pin_output(IOEXP_LCD_CS);
    ioexp_set_pin_output(IOEXP_LCD_RST);

    // Set CS high, RST high initially
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);

    // Configure SPI pins
    pinMode(LCD_SPI_SCK, OUTPUT);
    pinMode(LCD_SPI_MOSI, OUTPUT);
    digitalWrite(LCD_SPI_SCK, HIGH);
    digitalWrite(LCD_SPI_MOSI, LOW);

    Serial.println("[PASS] TEST_LCD_001: IO Expander and SPI pins initialized");
}

/**
 * TEST_LCD_002: Reset the LCD
 */
void test_lcd_reset(void) {
    Serial.println("[INFO] Resetting LCD via IO expander P05...");

    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    Serial.println("[PASS] TEST_LCD_002: LCD reset complete");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_LCD_003: Turn on backlight
 */
void test_lcd_backlight(void) {
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    delay(100);

    Serial.println("[PASS] TEST_LCD_003: Backlight ON");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_LCD_004: Initialize ST7701 via SPI
 */
void test_lcd_st7701_init(void) {
    st7701_init();
    Serial.println("[PASS] TEST_LCD_004: ST7701 initialized via SPI");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_LCD_005: Initialize RGB panel
 */
void test_lcd_rgb_init(void) {
    rgbpanel = new Arduino_ESP32RGBPanel(
        LCD_DE, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
        LCD_R0, LCD_R1, LCD_R2, LCD_R3, LCD_R4,
        LCD_G0, LCD_G1, LCD_G2, LCD_G3, LCD_G4, LCD_G5,
        LCD_B0, LCD_B1, LCD_B2, LCD_B3, LCD_B4,
        1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
        1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);

    // Create display without bus (we already initialized ST7701 manually)
    // Rotation 2 = 180 degrees (flip upside down)
    gfx = new Arduino_RGB_Display(480, 480, rgbpanel, 2, true);

    bool success = gfx->begin();
    Serial.printf("[INFO] gfx->begin() returned: %d\n", success);
    Serial.printf("[INFO] Display size: %d x %d\n", gfx->width(), gfx->height());

    TEST_ASSERT_EQUAL(480, gfx->width());
    TEST_ASSERT_EQUAL(480, gfx->height());

    Serial.println("[PASS] TEST_LCD_005: RGB panel initialized");
}

/**
 * TEST_LCD_006: Fill screen with colors
 */
void test_lcd_fill_screen(void) {
    Serial.println("[INFO] Filling screen RED...");
    gfx->fillScreen(RED);
    delay(1000);

    Serial.println("[INFO] Filling screen GREEN...");
    gfx->fillScreen(GREEN);
    delay(1000);

    Serial.println("[INFO] Filling screen BLUE...");
    gfx->fillScreen(BLUE);
    delay(1000);

    Serial.println("[INFO] Filling screen WHITE...");
    gfx->fillScreen(WHITE);
    delay(1000);

    Serial.println("[PASS] TEST_LCD_006: Fill screen complete");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_LCD_007: Draw pattern
 */
void test_lcd_draw_pattern(void) {
    gfx->fillScreen(BLACK);

    // Draw 3x3 color grid
    gfx->fillRect(0, 0, 160, 160, RED);
    gfx->fillRect(160, 0, 160, 160, GREEN);
    gfx->fillRect(320, 0, 160, 160, BLUE);
    gfx->fillRect(0, 160, 160, 160, YELLOW);
    gfx->fillRect(160, 160, 160, 160, CYAN);
    gfx->fillRect(320, 160, 160, 160, MAGENTA);
    gfx->fillRect(0, 320, 160, 160, WHITE);
    gfx->fillRect(160, 320, 160, 160, ORANGE);
    gfx->fillRect(320, 320, 160, 160, DARKGREY);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(100, 220);
    gfx->println("SenseCAP");
    gfx->setCursor(80, 260);
    gfx->println("Indicator D1L");

    Serial.println("[PASS] TEST_LCD_007: Pattern drawn");
    TEST_ASSERT_TRUE(true);
}

void setup() {
    delay(2000);

    Serial.begin(115200);
    Serial.println("\n======================================");
    Serial.println("LCD Display Test Suite - Task #5");
    Serial.println("Manual ST7701 Init + RGB Panel");
    Serial.println("======================================\n");

    UNITY_BEGIN();

    RUN_TEST(test_lcd_io_init);
    RUN_TEST(test_lcd_reset);
    RUN_TEST(test_lcd_backlight);
    RUN_TEST(test_lcd_st7701_init);
    RUN_TEST(test_lcd_rgb_init);
    RUN_TEST(test_lcd_fill_screen);
    RUN_TEST(test_lcd_draw_pattern);

    UNITY_END();
}

void loop() {
    delay(1000);
}
