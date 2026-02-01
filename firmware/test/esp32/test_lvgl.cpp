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
#define IOEXP_TP_INT    6   // P06 - Touch interrupt
#define IOEXP_TP_RST    7   // P07 - Touch reset

// FT6336U Touch Controller
#define FT6336U_ADDR    0x48  // Touch controller I2C address
#define FT_REG_TD_STATUS 0x02  // Number of touch points
#define FT_REG_P1_XH     0x03  // Point 1 X high + event
#define FT_REG_P1_XL     0x04  // Point 1 X low
#define FT_REG_P1_YH     0x05  // Point 1 Y high + ID
#define FT_REG_P1_YL     0x06  // Point 1 Y low

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
// Full screen buffer for clean refresh (requires PSRAM)
#define LVGL_BUFFER_LINES 480  // Full screen height (uses ~460KB in PSRAM)

// Global objects
static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

// LCD framebuffer pointer for direct access (zero-copy optimization)
static uint16_t *lcd_framebuffer = nullptr;

// Cached IO expander port0 value
static uint8_t ioexp_port0_cache = 0xFF;

// LVGL tick timer
static unsigned long lvgl_last_tick = 0;

// Touch state
static bool touch_detected = false;
static int16_t touch_last_x = 0;
static int16_t touch_last_y = 0;

// Animation state
static bool animation_running = false;
static lv_obj_t *gauge_meter = nullptr;
static lv_meter_scale_t *gauge_scale = nullptr;
static lv_meter_indicator_t *gauge_needle_indic = nullptr;
static lv_meter_indicator_t *gauge_arc_indic = nullptr;
static lv_obj_t *fps_label = nullptr;
static lv_obj_t *start_stop_btn = nullptr;
static lv_obj_t *start_stop_label = nullptr;
static int32_t gauge_value = 0;
static int32_t gauge_direction = 2;  // Slower needle movement to reduce tearing
static unsigned long last_gauge_update = 0;
static const unsigned long GAUGE_UPDATE_INTERVAL = 33;  // ~30Hz update rate

// FPS calculation
static unsigned long fps_last_time = 0;
static uint32_t fps_frame_count = 0;
static float current_fps = 0;

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

// Debug: track flush timing
static unsigned long flush_total_us = 0;
static uint32_t flush_count = 0;
static uint32_t flush_pixels = 0;
static unsigned long last_flush_print = 0;

void lvgl_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    unsigned long start = micros();

    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    if (lcd_framebuffer != nullptr) {
        uint16_t *src = (uint16_t *)color_p;

        // Check if this is a full-screen flush (for full_refresh mode with 180° rotation)
        if (w == SCREEN_WIDTH && h == SCREEN_HEIGHT) {
            // Full screen: 180° rotation = reverse the entire buffer
            // Much faster than per-pixel x,y calculation
            uint32_t total_pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
            uint16_t *dst = lcd_framebuffer + total_pixels - 1;
            for (uint32_t i = 0; i < total_pixels; i++) {
                *dst-- = *src++;
            }
        } else {
            // Partial update: direct copy (used when sw_rotate is enabled)
            uint16_t *dst = lcd_framebuffer + (area->y1 * SCREEN_WIDTH + area->x1);
            for (uint32_t y = 0; y < h; y++) {
                memcpy(dst, src, w * sizeof(uint16_t));
                src += w;
                dst += SCREEN_WIDTH;
            }
        }
    } else {
        gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
    }

    unsigned long elapsed = micros() - start;
    flush_total_us += elapsed;
    flush_count++;
    flush_pixels += w * h;

    // Print timing every second
    unsigned long now = millis();
    if (now - last_flush_print >= 1000) {
        Serial.printf("[FLUSH] %u/sec, %lu us avg, %uK px\n",
                      flush_count, flush_count > 0 ? flush_total_us / flush_count : 0,
                      flush_pixels / 1000);
        flush_total_us = 0;
        flush_count = 0;
        flush_pixels = 0;
        last_flush_print = now;
    }

    lv_disp_flush_ready(disp);
}

//=============================================================================
// Touch Controller Functions
//=============================================================================

bool ft6336u_read_touch(int16_t *x, int16_t *y) {
    uint8_t buf[5];

    Wire.beginTransmission(FT6336U_ADDR);
    Wire.write(FT_REG_TD_STATUS);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom((uint8_t)FT6336U_ADDR, (uint8_t)5);
    for (int i = 0; i < 5 && Wire.available(); i++) {
        buf[i] = Wire.read();
    }

    uint8_t num_points = buf[0] & 0x0F;
    if (num_points == 0 || num_points > 2) {
        return false;
    }

    // Extract coordinates (12-bit values)
    *x = ((buf[1] & 0x0F) << 8) | buf[2];
    *y = ((buf[3] & 0x0F) << 8) | buf[4];

    return true;
}

void ft6336u_reset(void) {
    ioexp_set_pin_output(IOEXP_TP_RST);
    ioexp_write_pin(IOEXP_TP_RST, LOW);
    delay(10);
    ioexp_write_pin(IOEXP_TP_RST, HIGH);
    delay(300);
}

bool ft6336u_init(void) {
    ft6336u_reset();

    Wire.beginTransmission(FT6336U_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.printf("[TOUCH] FT6336U detected at 0x%02X\n", FT6336U_ADDR);
        return true;
    }
    Serial.println("[TOUCH] FT6336U not detected");
    return false;
}

//=============================================================================
// LVGL Touch Input Callback
//=============================================================================

void lvgl_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x, y;

    if (ft6336u_read_touch(&x, &y)) {
        // Touch detected - apply 180° rotation to match display
        int16_t rx = SCREEN_WIDTH - 1 - x;
        int16_t ry = SCREEN_HEIGHT - 1 - y;

        // Clamp to screen bounds
        if (rx < 0) rx = 0;
        if (rx >= SCREEN_WIDTH) rx = SCREEN_WIDTH - 1;
        if (ry < 0) ry = 0;
        if (ry >= SCREEN_HEIGHT) ry = SCREEN_HEIGHT - 1;

        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = rx;
        data->point.y = ry;
        touch_last_x = rx;
        touch_last_y = ry;
    } else {
        // No touch
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = touch_last_x;
        data->point.y = touch_last_y;
    }
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

    // Rotation 0 for direct framebuffer access (no coordinate transformation needed)
    // Touch coordinates are adjusted in lvgl_touch_read() to compensate
    gfx = new Arduino_RGB_Display(SCREEN_WIDTH, SCREEN_HEIGHT, rgbpanel, 0, true);

    if (!gfx->begin()) {
        Serial.println("[ERROR] RGB display init failed");
        return false;
    }
    Serial.printf("[OK] RGB display: %dx%d\n", gfx->width(), gfx->height());

    // Get direct framebuffer pointer for zero-copy LVGL rendering
    lcd_framebuffer = (uint16_t *)gfx->getFramebuffer();
    if (lcd_framebuffer != nullptr) {
        Serial.printf("[OK] LCD framebuffer: 0x%08X (direct access enabled)\n", (uint32_t)lcd_framebuffer);
    } else {
        Serial.println("[WARN] LCD framebuffer not available, using slow path");
    }

    // Enable backlight
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    Serial.println("[OK] Backlight enabled");

    // Clear screen
    gfx->fillScreen(BLACK);

    // Initialize touch controller
    touch_detected = ft6336u_init();

    return true;
}

//=============================================================================
// LVGL Initialization
//=============================================================================

bool init_lvgl(void) {
    Serial.println("[LVGL] Initializing...");

    lv_init();

    // Buffer allocation strategy:
    // Use PSRAM double buffers - LVGL draws to PSRAM, then we copy to LCD framebuffer.
    // This avoids tearing (drawing to active display buffer) and allows rotation handling.
    // Zero-copy mode was removed because it caused tearing and orientation issues.

    size_t buf_size = SCREEN_WIDTH * LVGL_BUFFER_LINES * sizeof(lv_color_t);
    Serial.printf("[LVGL] Buffer size: %d bytes\n", buf_size);

    // Allocate both buffers from PSRAM for double buffering
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!buf1 || !buf2) {
        Serial.println("[WARN] PSRAM double buffer failed, trying single buffer");
        if (buf1) heap_caps_free(buf1);
        if (buf2) heap_caps_free(buf2);
        buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        buf2 = NULL;
    }

    if (!buf1) {
        Serial.println("[WARN] PSRAM allocation failed, trying internal RAM");
        buf1 = (lv_color_t *)malloc(buf_size);
        buf2 = NULL;
    }

    if (!buf1) {
        Serial.println("[ERROR] Failed to allocate LVGL buffer");
        return false;
    }
    Serial.printf("[OK] LVGL buffer(s) allocated (double: %s)\n", buf2 ? "yes" : "no");

    // Initialize display buffer
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_WIDTH * LVGL_BUFFER_LINES);

    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1;  // Full refresh - reduces tearing artifacts
    // Note: sw_rotate disabled - we handle 180° rotation in flush callback
    disp_drv.sw_rotate = 0;
    disp_drv.rotated = LV_DISP_ROT_NONE;
    lv_disp_drv_register(&disp_drv);

    Serial.println("[OK] LVGL display driver registered");

    // Initialize touch input device
    if (touch_detected) {
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read;
        lv_indev_drv_register(&indev_drv);
        Serial.println("[OK] LVGL touch input registered");
    }

    return true;
}

//=============================================================================
// UI Event Callbacks
//=============================================================================

static void btn_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    if (code == LV_EVENT_CLICKED) {
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        const char *txt = lv_label_get_text(label);
        Serial.printf("[UI] Button clicked: %s\n", txt);
    }
}

static void switch_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *sw = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
        Serial.printf("[UI] Switch toggled: %s\n", state ? "ON" : "OFF");
    }
}

static void slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t val = lv_slider_get_value(slider);
    Serial.printf("[UI] Slider value: %d\n", val);
}

static void start_stop_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        animation_running = !animation_running;

        if (start_stop_label) {
            if (animation_running) {
                lv_label_set_text(start_stop_label, LV_SYMBOL_PAUSE " Stop");
                lv_obj_set_style_bg_color(start_stop_btn, lv_color_hex(0xff6b6b), 0);
            } else {
                lv_label_set_text(start_stop_label, LV_SYMBOL_PLAY " Start");
                lv_obj_set_style_bg_color(start_stop_btn, lv_color_hex(0x00ff88), 0);
            }
        }
        Serial.printf("[UI] Animation %s\n", animation_running ? "started" : "stopped");
    }
}

//=============================================================================
// Demo UI Creation - Tabbed Interface
//=============================================================================

void create_demo_ui(void) {
    Serial.println("[UI] Creating tabbed interface...");

    // Get the active screen and set background
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // Create tabview
    lv_obj_t *tabview = lv_tabview_create(scr, LV_DIR_TOP, 50);
    lv_obj_set_size(tabview, 480, 480);
    lv_obj_set_style_bg_color(tabview, lv_color_hex(0x1a1a2e), 0);

    // Style the tab buttons
    lv_obj_t *tab_btns = lv_tabview_get_tab_btns(tabview);
    lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(0x00ff88), LV_PART_ITEMS | LV_STATE_CHECKED);

    // Create tabs
    lv_obj_t *tab_sensors = lv_tabview_add_tab(tabview, LV_SYMBOL_HOME " Sensors");
    lv_obj_t *tab_settings = lv_tabview_add_tab(tabview, LV_SYMBOL_SETTINGS " Settings");
    lv_obj_t *tab_info = lv_tabview_add_tab(tabview, LV_SYMBOL_LIST " Info");

    // Style all tab content areas
    lv_obj_set_style_bg_color(tab_sensors, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_bg_color(tab_settings, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_bg_color(tab_info, lv_color_hex(0x1a1a2e), 0);

    //=========================================================================
    // TAB 1: Sensors
    //=========================================================================

    // Container for sensor data
    lv_obj_t *sensor_cont = lv_obj_create(tab_sensors);
    lv_obj_set_size(sensor_cont, 440, 340);
    lv_obj_align(sensor_cont, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(sensor_cont, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_color(sensor_cont, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_border_width(sensor_cont, 2, 0);
    lv_obj_set_style_radius(sensor_cont, 15, 0);
    lv_obj_set_style_pad_all(sensor_cont, 20, 0);

    // Temperature
    lv_obj_t *temp_label = lv_label_create(sensor_cont);
    lv_label_set_text(temp_label, LV_SYMBOL_HOME " Temperature");
    lv_obj_set_style_text_color(temp_label, lv_color_hex(0xff6b6b), 0);
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *temp_value = lv_label_create(sensor_cont);
    lv_label_set_text(temp_value, "23.5 C");
    lv_obj_set_style_text_color(temp_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(temp_value, &lv_font_montserrat_32, 0);
    lv_obj_align(temp_value, LV_ALIGN_TOP_LEFT, 0, 30);

    // Humidity
    lv_obj_t *hum_label = lv_label_create(sensor_cont);
    lv_label_set_text(hum_label, LV_SYMBOL_WIFI " Humidity");
    lv_obj_set_style_text_color(hum_label, lv_color_hex(0x4ecdc4), 0);
    lv_obj_align(hum_label, LV_ALIGN_TOP_LEFT, 210, 0);

    lv_obj_t *hum_value = lv_label_create(sensor_cont);
    lv_label_set_text(hum_value, "65 %");
    lv_obj_set_style_text_color(hum_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(hum_value, &lv_font_montserrat_32, 0);
    lv_obj_align(hum_value, LV_ALIGN_TOP_LEFT, 210, 30);

    // CO2
    lv_obj_t *co2_label = lv_label_create(sensor_cont);
    lv_label_set_text(co2_label, LV_SYMBOL_CHARGE " CO2");
    lv_obj_set_style_text_color(co2_label, lv_color_hex(0xffe66d), 0);
    lv_obj_align(co2_label, LV_ALIGN_TOP_LEFT, 0, 90);

    lv_obj_t *co2_value = lv_label_create(sensor_cont);
    lv_label_set_text(co2_value, "412 ppm");
    lv_obj_set_style_text_color(co2_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(co2_value, &lv_font_montserrat_32, 0);
    lv_obj_align(co2_value, LV_ALIGN_TOP_LEFT, 0, 120);

    // TVOC
    lv_obj_t *tvoc_label = lv_label_create(sensor_cont);
    lv_label_set_text(tvoc_label, LV_SYMBOL_WARNING " TVOC");
    lv_obj_set_style_text_color(tvoc_label, lv_color_hex(0xa29bfe), 0);
    lv_obj_align(tvoc_label, LV_ALIGN_TOP_LEFT, 210, 90);

    lv_obj_t *tvoc_value = lv_label_create(sensor_cont);
    lv_label_set_text(tvoc_value, "125 ppb");
    lv_obj_set_style_text_color(tvoc_value, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(tvoc_value, &lv_font_montserrat_32, 0);
    lv_obj_align(tvoc_value, LV_ALIGN_TOP_LEFT, 210, 120);

    // Air Quality meter (animated speedometer)
    gauge_meter = lv_meter_create(sensor_cont);
    lv_obj_set_size(gauge_meter, 180, 180);
    lv_obj_align(gauge_meter, LV_ALIGN_BOTTOM_LEFT, 10, 10);
    lv_obj_set_style_bg_color(gauge_meter, lv_color_hex(0x0f1a2e), 0);
    lv_obj_set_style_border_color(gauge_meter, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_border_width(gauge_meter, 2, 0);

    gauge_scale = lv_meter_add_scale(gauge_meter);
    lv_meter_set_scale_ticks(gauge_meter, gauge_scale, 21, 2, 10, lv_color_hex(0x666666));
    lv_meter_set_scale_major_ticks(gauge_meter, gauge_scale, 4, 3, 15, lv_color_hex(0xffffff), 10);
    lv_meter_set_scale_range(gauge_meter, gauge_scale, 0, 100, 270, 135);

    // Arc indicator (shows value as colored arc)
    gauge_arc_indic = lv_meter_add_arc(gauge_meter, gauge_scale, 10, lv_color_hex(0x00ff88), 0);
    lv_meter_set_indicator_end_value(gauge_meter, gauge_arc_indic, 0);

    // Needle indicator
    gauge_needle_indic = lv_meter_add_needle_line(gauge_meter, gauge_scale, 4, lv_color_hex(0xff6b6b), -15);
    lv_meter_set_indicator_value(gauge_meter, gauge_needle_indic, 0);

    // Start/Stop button
    start_stop_btn = lv_btn_create(sensor_cont);
    lv_obj_set_size(start_stop_btn, 110, 45);
    lv_obj_align(start_stop_btn, LV_ALIGN_BOTTOM_RIGHT, -30, -50);
    lv_obj_set_style_bg_color(start_stop_btn, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_radius(start_stop_btn, 10, 0);
    start_stop_label = lv_label_create(start_stop_btn);
    lv_label_set_text(start_stop_label, LV_SYMBOL_PLAY " Start");
    lv_obj_center(start_stop_label);
    lv_obj_add_event_cb(start_stop_btn, start_stop_event_cb, LV_EVENT_CLICKED, NULL);

    // FPS counter label
    fps_label = lv_label_create(sensor_cont);
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_style_text_color(fps_label, lv_color_hex(0xffe66d), 0);
    lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_18, 0);
    lv_obj_align(fps_label, LV_ALIGN_BOTTOM_RIGHT, -30, -10);

    //=========================================================================
    // TAB 2: Settings
    //=========================================================================

    lv_obj_t *settings_cont = lv_obj_create(tab_settings);
    lv_obj_set_size(settings_cont, 440, 340);
    lv_obj_align(settings_cont, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(settings_cont, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_color(settings_cont, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_border_width(settings_cont, 2, 0);
    lv_obj_set_style_radius(settings_cont, 15, 0);
    lv_obj_set_flex_flow(settings_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(settings_cont, 20, 0);
    lv_obj_set_style_pad_row(settings_cont, 15, 0);

    // WiFi toggle
    lv_obj_t *wifi_row = lv_obj_create(settings_cont);
    lv_obj_set_size(wifi_row, 380, 50);
    lv_obj_set_style_bg_opa(wifi_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wifi_row, 0, 0);
    lv_obj_set_style_pad_all(wifi_row, 0, 0);

    lv_obj_t *wifi_label = lv_label_create(wifi_row);
    lv_label_set_text(wifi_label, LV_SYMBOL_WIFI " WiFi");
    lv_obj_set_style_text_color(wifi_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(wifi_label, &lv_font_montserrat_18, 0);
    lv_obj_align(wifi_label, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *wifi_sw = lv_switch_create(wifi_row);
    lv_obj_align(wifi_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(wifi_sw, LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(wifi_sw, lv_color_hex(0x00ff88), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(wifi_sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Bluetooth toggle
    lv_obj_t *bt_row = lv_obj_create(settings_cont);
    lv_obj_set_size(bt_row, 380, 50);
    lv_obj_set_style_bg_opa(bt_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(bt_row, 0, 0);
    lv_obj_set_style_pad_all(bt_row, 0, 0);

    lv_obj_t *bt_label = lv_label_create(bt_row);
    lv_label_set_text(bt_label, LV_SYMBOL_BLUETOOTH " Bluetooth");
    lv_obj_set_style_text_color(bt_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(bt_label, &lv_font_montserrat_18, 0);
    lv_obj_align(bt_label, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *bt_sw = lv_switch_create(bt_row);
    lv_obj_align(bt_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(bt_sw, lv_color_hex(0x00ff88), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(bt_sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Brightness slider
    lv_obj_t *bright_label = lv_label_create(settings_cont);
    lv_label_set_text(bright_label, LV_SYMBOL_IMAGE " Brightness");
    lv_obj_set_style_text_color(bright_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(bright_label, &lv_font_montserrat_18, 0);

    lv_obj_t *bright_slider = lv_slider_create(settings_cont);
    lv_obj_set_width(bright_slider, 380);
    lv_slider_set_value(bright_slider, 80, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bright_slider, lv_color_hex(0x0f3460), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bright_slider, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bright_slider, lv_color_hex(0xffffff), LV_PART_KNOB);
    lv_obj_add_event_cb(bright_slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Action buttons row
    lv_obj_t *btn_row = lv_obj_create(settings_cont);
    lv_obj_set_size(btn_row, 380, 60);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Refresh button
    lv_obj_t *btn_refresh = lv_btn_create(btn_row);
    lv_obj_set_size(btn_refresh, 110, 45);
    lv_obj_set_style_bg_color(btn_refresh, lv_color_hex(0x4ecdc4), 0);
    lv_obj_set_style_radius(btn_refresh, 10, 0);
    lv_obj_t *lbl_refresh = lv_label_create(btn_refresh);
    lv_label_set_text(lbl_refresh, LV_SYMBOL_REFRESH " Refresh");
    lv_obj_center(lbl_refresh);
    lv_obj_add_event_cb(btn_refresh, btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Save button
    lv_obj_t *btn_save = lv_btn_create(btn_row);
    lv_obj_set_size(btn_save, 110, 45);
    lv_obj_set_style_bg_color(btn_save, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_radius(btn_save, 10, 0);
    lv_obj_t *lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, LV_SYMBOL_SAVE " Save");
    lv_obj_center(lbl_save);
    lv_obj_add_event_cb(btn_save, btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Reset button
    lv_obj_t *btn_reset = lv_btn_create(btn_row);
    lv_obj_set_size(btn_reset, 110, 45);
    lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0xff6b6b), 0);
    lv_obj_set_style_radius(btn_reset, 10, 0);
    lv_obj_t *lbl_reset = lv_label_create(btn_reset);
    lv_label_set_text(lbl_reset, LV_SYMBOL_TRASH " Reset");
    lv_obj_center(lbl_reset);
    lv_obj_add_event_cb(btn_reset, btn_event_cb, LV_EVENT_CLICKED, NULL);

    //=========================================================================
    // TAB 3: Info
    //=========================================================================

    lv_obj_t *info_cont = lv_obj_create(tab_info);
    lv_obj_set_size(info_cont, 440, 340);
    lv_obj_align(info_cont, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(info_cont, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_color(info_cont, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_border_width(info_cont, 2, 0);
    lv_obj_set_style_radius(info_cont, 15, 0);
    lv_obj_set_style_pad_all(info_cont, 25, 0);

    // Title
    lv_obj_t *info_title = lv_label_create(info_cont);
    lv_label_set_text(info_title, "SenseCAP Indicator D1L");
    lv_obj_set_style_text_color(info_title, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(info_title, &lv_font_montserrat_24, 0);
    lv_obj_align(info_title, LV_ALIGN_TOP_MID, 0, 0);

    // Info text
    lv_obj_t *info_text = lv_label_create(info_cont);
    lv_label_set_text(info_text,
        "Firmware: v1.0.0-dev\n"
        "LVGL: v8.3\n"
        "ESP-IDF: v4.4\n\n"
        "Hardware:\n"
        "  CPU: ESP32-S3 @ 240MHz\n"
        "  Co-processor: RP2040\n"
        "  Display: 480x480 RGB\n"
        "  Touch: FT6336U\n\n"
        "Sensors:\n"
        "  AHT20 (Temp/Humidity)\n"
        "  SGP40 (TVOC)\n"
        "  SCD41 (CO2)");
    lv_obj_set_style_text_color(info_text, lv_color_hex(0xcccccc), 0);
    lv_obj_set_style_text_font(info_text, &lv_font_montserrat_16, 0);
    lv_obj_align(info_text, LV_ALIGN_TOP_LEFT, 0, 40);

    // Version badge
    lv_obj_t *badge = lv_obj_create(info_cont);
    lv_obj_set_size(badge, 180, 35);
    lv_obj_align(badge, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(badge, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_radius(badge, 17, 0);
    lv_obj_set_style_border_width(badge, 0, 0);

    lv_obj_t *badge_text = lv_label_create(badge);
    lv_label_set_text(badge_text, "SquareLine Ready");
    lv_obj_set_style_text_color(badge_text, lv_color_hex(0x00ff88), 0);
    lv_obj_center(badge_text);

    Serial.println("[OK] Tabbed UI created");
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
    fps_last_time = millis();  // Initialize FPS timer

    Serial.println("\n[OK] Setup complete - LVGL running\n");
}

// Minimum loop interval for efficient polling (10ms = 100 Hz max)
// This reduces touch I2C traffic from 1000+ to ~100 reads/sec
#define MIN_LOOP_INTERVAL_MS 10

void loop() {
    static unsigned long last_loop_time = 0;
    unsigned long now = millis();

    // Throttle loop to reduce I2C polling overhead
    // Without this, touch polling runs at 1000+ Hz wasting CPU on I2C
    unsigned long elapsed = now - last_loop_time;
    if (elapsed < MIN_LOOP_INTERVAL_MS) {
        delay(MIN_LOOP_INTERVAL_MS - elapsed);
        now = millis();
    }
    last_loop_time = now;

    // Update LVGL tick
    lv_tick_inc(now - lvgl_last_tick);
    lvgl_last_tick = now;

    // FPS calculation
    fps_frame_count++;
    if (now - fps_last_time >= 1000) {
        current_fps = fps_frame_count * 1000.0f / (now - fps_last_time);
        fps_last_time = now;
        fps_frame_count = 0;

        // Update FPS label
        if (fps_label) {
            static char fps_buf[16];
            snprintf(fps_buf, sizeof(fps_buf), "FPS: %.1f", current_fps);
            lv_label_set_text(fps_label, fps_buf);
        }
    }

    // Animate gauge if running (rate-limited to reduce tearing)
    if (animation_running && gauge_meter && gauge_needle_indic && gauge_arc_indic) {
        if (now - last_gauge_update >= GAUGE_UPDATE_INTERVAL) {
            last_gauge_update = now;

            gauge_value += gauge_direction;

            // Bounce at limits
            if (gauge_value >= 100) {
                gauge_value = 100;
                gauge_direction = -2;
            } else if (gauge_value <= 0) {
                gauge_value = 0;
                gauge_direction = 2;
            }

            // Update meter indicators
            lv_meter_set_indicator_value(gauge_meter, gauge_needle_indic, gauge_value);
            lv_meter_set_indicator_end_value(gauge_meter, gauge_arc_indic, gauge_value);

            // Change arc color based on value (green -> yellow -> red)
            lv_color_t arc_color;
            if (gauge_value < 33) {
                arc_color = lv_color_hex(0x00ff88);  // Green
            } else if (gauge_value < 66) {
                arc_color = lv_color_hex(0xffe66d);  // Yellow
            } else {
                arc_color = lv_color_hex(0xff6b6b);  // Red
            }
            lv_meter_set_indicator_start_value(gauge_meter, gauge_arc_indic, 0);
        }
    }

    // Run LVGL task handler
    lv_timer_handler();
}
