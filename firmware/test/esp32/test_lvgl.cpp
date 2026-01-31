/**
 * @file test_lvgl.cpp
 * @brief LVGL Integration Test for SenseCAP Indicator
 *
 * Tests LVGL graphics library with ST7701 480x480 RGB LCD
 * Compatible with SquareLine Studio generated UI code
 */

#include <Arduino.h>
#include <Wire.h>

// LVGL must be included before Arduino_GFX to avoid conflicts
#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

// Display configuration
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 480

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

// SPI pins for ST7701 initialization
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

// LVGL buffer configuration
// Larger buffer reduces tearing, uses more RAM
#define LVGL_BUFFER_LINES 80  // Number of lines per buffer (uses ~76KB each)

// Global objects
static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

// Cached IO expander port0 value
static uint8_t ioexp_port0_cache = 0xFF;

// LVGL tick timer
static unsigned long lvgl_last_tick = 0;

//=============================================================================
// IO Expander Helper Functions
//=============================================================================

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
    val &= ~(1 << bit);
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

//=============================================================================
// Software SPI for ST7701 (3-wire 9-bit mode)
//=============================================================================

void spi_write_9bit(uint8_t dc, uint8_t data) {
    digitalWrite(LCD_SPI_SCK, LOW);
    digitalWrite(LCD_SPI_MOSI, dc ? HIGH : LOW);
    digitalWrite(LCD_SPI_SCK, HIGH);

    for (int i = 7; i >= 0; i--) {
        digitalWrite(LCD_SPI_SCK, LOW);
        digitalWrite(LCD_SPI_MOSI, (data >> i) & 1 ? HIGH : LOW);
        digitalWrite(LCD_SPI_SCK, HIGH);
    }
}

void lcd_write_cmd(uint8_t cmd) {
    ioexp_write_pin(IOEXP_LCD_CS, LOW);
    spi_write_9bit(0, cmd);
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
}

void lcd_write_data(uint8_t data) {
    ioexp_write_pin(IOEXP_LCD_CS, LOW);
    spi_write_9bit(1, data);
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
}

//=============================================================================
// ST7701S Initialization Sequence
//=============================================================================

void st7701_init(void) {
    Serial.println("[LCD] Sending ST7701 init sequence...");

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
    lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x18); lcd_write_data(0x0E);
    lcd_write_data(0x11); lcd_write_data(0x06); lcd_write_data(0x07); lcd_write_data(0x08);
    lcd_write_data(0x07); lcd_write_data(0x22); lcd_write_data(0x04); lcd_write_data(0x12);
    lcd_write_data(0x0F); lcd_write_data(0xAA); lcd_write_data(0x31); lcd_write_data(0x18);

    // Negative Gamma
    lcd_write_cmd(0xB1);
    lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x19); lcd_write_data(0x0E);
    lcd_write_data(0x12); lcd_write_data(0x07); lcd_write_data(0x08); lcd_write_data(0x08);
    lcd_write_data(0x08); lcd_write_data(0x22); lcd_write_data(0x04); lcd_write_data(0x11);
    lcd_write_data(0x11); lcd_write_data(0xA9); lcd_write_data(0x32); lcd_write_data(0x18);

    // Command2 BK1
    lcd_write_cmd(0xFF);
    lcd_write_data(0x77);
    lcd_write_data(0x01);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x11);

    lcd_write_cmd(0xB0); lcd_write_data(0x60);
    lcd_write_cmd(0xB1); lcd_write_data(0x32);
    lcd_write_cmd(0xB2); lcd_write_data(0x07);
    lcd_write_cmd(0xB3); lcd_write_data(0x80);
    lcd_write_cmd(0xB5); lcd_write_data(0x49);
    lcd_write_cmd(0xB7); lcd_write_data(0x85);
    lcd_write_cmd(0xB8); lcd_write_data(0x21);
    lcd_write_cmd(0xC1); lcd_write_data(0x78);
    lcd_write_cmd(0xC2); lcd_write_data(0x78);

    delay(20);

    lcd_write_cmd(0xE0);
    lcd_write_data(0x00); lcd_write_data(0x1B); lcd_write_data(0x02);

    lcd_write_cmd(0xE1);
    lcd_write_data(0x08); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0x07); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0x00); lcd_write_data(0x44); lcd_write_data(0x44);

    lcd_write_cmd(0xE2);
    lcd_write_data(0x11); lcd_write_data(0x11); lcd_write_data(0x44); lcd_write_data(0x44);
    lcd_write_data(0xED); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0xEC); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);

    lcd_write_cmd(0xE3);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x11);

    lcd_write_cmd(0xE4);
    lcd_write_data(0x44); lcd_write_data(0x44);

    lcd_write_cmd(0xE5);
    lcd_write_data(0x0A); lcd_write_data(0xE9); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0C); lcd_write_data(0xEB); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0E); lcd_write_data(0xED); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x10); lcd_write_data(0xEF); lcd_write_data(0xD8); lcd_write_data(0xA0);

    lcd_write_cmd(0xE6);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x11);

    lcd_write_cmd(0xE7);
    lcd_write_data(0x44); lcd_write_data(0x44);

    lcd_write_cmd(0xE8);
    lcd_write_data(0x09); lcd_write_data(0xE8); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0B); lcd_write_data(0xEA); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0D); lcd_write_data(0xEC); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0F); lcd_write_data(0xEE); lcd_write_data(0xD8); lcd_write_data(0xA0);

    lcd_write_cmd(0xEB);
    lcd_write_data(0x02); lcd_write_data(0x00); lcd_write_data(0xE4); lcd_write_data(0xE4);
    lcd_write_data(0x88); lcd_write_data(0x00); lcd_write_data(0x40);

    lcd_write_cmd(0xEC);
    lcd_write_data(0x3C); lcd_write_data(0x00);

    lcd_write_cmd(0xED);
    lcd_write_data(0xAB); lcd_write_data(0x89); lcd_write_data(0x76); lcd_write_data(0x54);
    lcd_write_data(0x02); lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0xFF);
    lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0x20);
    lcd_write_data(0x45); lcd_write_data(0x67); lcd_write_data(0x98); lcd_write_data(0xBA);

    // Exit Command2
    lcd_write_cmd(0xFF);
    lcd_write_data(0x77);
    lcd_write_data(0x01);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);

    // Set pixel format: 0x60 = RGB666 (18-bit), 0x50 = RGB565 (16-bit)
    lcd_write_cmd(0x3A);
    lcd_write_data(0x60);  // RGB666 for better compatibility

    // MADCTL - Memory Access Control
    // Bit 3 (0x08): BGR order. 0=RGB, 1=BGR
    lcd_write_cmd(0x36);
    lcd_write_data(0x00);  // RGB order

    // Enable display inversion to correct color polarity
    lcd_write_cmd(0x21);

    lcd_write_cmd(0x11);  // Sleep Out
    delay(120);
    lcd_write_cmd(0x29);  // Display On
    delay(20);

    Serial.println("[LCD] ST7701 init complete");
}

//=============================================================================
// LVGL Display Driver Callback
//=============================================================================

void lvgl_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Use Arduino_GFX to draw the buffer
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);

    // Notify LVGL that flushing is done
    lv_disp_flush_ready(disp);
}

//=============================================================================
// Hardware Initialization
//=============================================================================

bool init_hardware(void) {
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // Check IO expander
    Wire.beginTransmission(IOEXP_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[ERROR] IO Expander not detected at 0x20");
        return false;
    }
    Serial.println("[OK] IO Expander detected");

    // Read current port0 state
    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);

    // Configure LCD CS and RST as outputs
    ioexp_set_pin_output(IOEXP_LCD_CS);
    ioexp_set_pin_output(IOEXP_LCD_RST);

    // Set CS high, RST high
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);

    // Configure SPI pins
    pinMode(LCD_SPI_SCK, OUTPUT);
    pinMode(LCD_SPI_MOSI, OUTPUT);
    digitalWrite(LCD_SPI_SCK, HIGH);
    digitalWrite(LCD_SPI_MOSI, LOW);

    // Reset LCD
    Serial.println("[LCD] Resetting display...");
    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    // Initialize ST7701
    st7701_init();

    // Initialize RGB panel
    rgbpanel = new Arduino_ESP32RGBPanel(
        LCD_DE, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
        LCD_R0, LCD_R1, LCD_R2, LCD_R3, LCD_R4,
        LCD_G0, LCD_G1, LCD_G2, LCD_G3, LCD_G4, LCD_G5,
        LCD_B0, LCD_B1, LCD_B2, LCD_B3, LCD_B4,
        1, 10, 8, 50,   // hsync: polarity, front_porch, pulse_width, back_porch
        1, 10, 8, 20);  // vsync: polarity, front_porch, pulse_width, back_porch

    gfx = new Arduino_RGB_Display(SCREEN_WIDTH, SCREEN_HEIGHT, rgbpanel, 2, true);

    if (!gfx->begin()) {
        Serial.println("[ERROR] RGB display init failed");
        return false;
    }
    Serial.printf("[OK] RGB display: %dx%d\n", gfx->width(), gfx->height());

    // Enable backlight
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    Serial.println("[OK] Backlight enabled");

    // Clear screen
    gfx->fillScreen(BLACK);

    return true;
}

//=============================================================================
// LVGL Initialization
//=============================================================================

bool init_lvgl(void) {
    Serial.println("[LVGL] Initializing...");

    lv_init();

    // Allocate draw buffers from PSRAM if available
    size_t buf_size = SCREEN_WIDTH * LVGL_BUFFER_LINES * sizeof(lv_color_t);
    Serial.printf("[LVGL] Allocating %d bytes per buffer\n", buf_size);

    buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) {
        // Fallback to internal RAM
        buf1 = (lv_color_t *)malloc(buf_size);
    }

    buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf2) {
        buf2 = (lv_color_t *)malloc(buf_size);
    }

    if (!buf1 || !buf2) {
        Serial.println("[ERROR] Failed to allocate LVGL buffers");
        return false;
    }
    Serial.println("[OK] LVGL buffers allocated");

    // Initialize display buffer
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_WIDTH * LVGL_BUFFER_LINES);

    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    Serial.println("[OK] LVGL display driver registered");

    return true;
}

//=============================================================================
// Demo UI Creation
//=============================================================================

void create_demo_ui(void) {
    Serial.println("[UI] Creating demo interface...");

    // Get the active screen
    lv_obj_t *scr = lv_scr_act();

    // Set background color
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // Create a title label
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SenseCAP Indicator");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    // Create a subtitle
    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "LVGL Demo");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_18, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 70);

    // Create a container for stats
    lv_obj_t *container = lv_obj_create(scr);
    lv_obj_set_size(container, 420, 280);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_radius(container, 15, 0);

    // Temperature display
    lv_obj_t *temp_label = lv_label_create(container);
    lv_label_set_text(temp_label, LV_SYMBOL_HOME " Temperature");
    lv_obj_set_style_text_color(temp_label, lv_color_hex(0xff6b6b), 0);
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 20, 20);

    lv_obj_t *temp_value = lv_label_create(container);
    lv_label_set_text(temp_value, "23.5 C");
    lv_obj_set_style_text_color(temp_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(temp_value, &lv_font_montserrat_28, 0);
    lv_obj_align(temp_value, LV_ALIGN_TOP_LEFT, 20, 50);

    // Humidity display
    lv_obj_t *hum_label = lv_label_create(container);
    lv_label_set_text(hum_label, LV_SYMBOL_WIFI " Humidity");
    lv_obj_set_style_text_color(hum_label, lv_color_hex(0x4ecdc4), 0);
    lv_obj_align(hum_label, LV_ALIGN_TOP_LEFT, 220, 20);

    lv_obj_t *hum_value = lv_label_create(container);
    lv_label_set_text(hum_value, "65 %");
    lv_obj_set_style_text_color(hum_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(hum_value, &lv_font_montserrat_28, 0);
    lv_obj_align(hum_value, LV_ALIGN_TOP_LEFT, 220, 50);

    // CO2 display
    lv_obj_t *co2_label = lv_label_create(container);
    lv_label_set_text(co2_label, LV_SYMBOL_CHARGE " CO2");
    lv_obj_set_style_text_color(co2_label, lv_color_hex(0xffe66d), 0);
    lv_obj_align(co2_label, LV_ALIGN_TOP_LEFT, 20, 100);

    lv_obj_t *co2_value = lv_label_create(container);
    lv_label_set_text(co2_value, "412 ppm");
    lv_obj_set_style_text_color(co2_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(co2_value, &lv_font_montserrat_28, 0);
    lv_obj_align(co2_value, LV_ALIGN_TOP_LEFT, 20, 130);

    // TVOC display
    lv_obj_t *tvoc_label = lv_label_create(container);
    lv_label_set_text(tvoc_label, LV_SYMBOL_WARNING " TVOC");
    lv_obj_set_style_text_color(tvoc_label, lv_color_hex(0xa29bfe), 0);
    lv_obj_align(tvoc_label, LV_ALIGN_TOP_LEFT, 220, 100);

    lv_obj_t *tvoc_value = lv_label_create(container);
    lv_label_set_text(tvoc_value, "125 ppb");
    lv_obj_set_style_text_color(tvoc_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(tvoc_value, &lv_font_montserrat_28, 0);
    lv_obj_align(tvoc_value, LV_ALIGN_TOP_LEFT, 220, 130);

    // Progress bar
    lv_obj_t *bar = lv_bar_create(container);
    lv_obj_set_size(bar, 380, 20);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_bar_set_value(bar, 70, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x0f3460), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);

    lv_obj_t *bar_label = lv_label_create(container);
    lv_label_set_text(bar_label, "Air Quality: Good");
    lv_obj_set_style_text_color(bar_label, lv_color_hex(0x00ff88), 0);
    lv_obj_align(bar_label, LV_ALIGN_BOTTOM_MID, 0, -20);

    // Footer
    lv_obj_t *footer = lv_label_create(scr);
    lv_label_set_text(footer, "SquareLine Studio Compatible");
    lv_obj_set_style_text_color(footer, lv_color_hex(0x666666), 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -20);

    Serial.println("[OK] Demo UI created");
}

//=============================================================================
// Main Setup and Loop
//=============================================================================

void setup() {
    delay(2000);

    Serial.begin(115200);
    Serial.println("\n==========================================");
    Serial.println("LVGL Integration Test - SenseCAP Indicator");
    Serial.println("==========================================\n");

    // Initialize hardware
    if (!init_hardware()) {
        Serial.println("[FATAL] Hardware initialization failed!");
        while (1) delay(1000);
    }

    // Initialize LVGL
    if (!init_lvgl()) {
        Serial.println("[FATAL] LVGL initialization failed!");
        while (1) delay(1000);
    }

    // Create demo UI
    create_demo_ui();

    lvgl_last_tick = millis();

    Serial.println("\n[OK] Setup complete - LVGL running\n");
}

void loop() {
    // Update LVGL tick
    unsigned long now = millis();
    lv_tick_inc(now - lvgl_last_tick);
    lvgl_last_tick = now;

    // Run LVGL task handler
    lv_timer_handler();

    delay(5);  // ~200 fps maximum
}
