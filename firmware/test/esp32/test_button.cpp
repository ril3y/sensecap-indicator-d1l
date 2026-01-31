/**
 * @file test_button.cpp
 * @brief User Button Test with LVGL UI for SenseCAP Indicator
 *
 * Tests GPIO38 user button with visual feedback on LCD
 */

#include <Arduino.h>
#include <Wire.h>

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

// Display configuration
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 480

// User Button
#define USER_BUTTON_PIN 38

// I2C pins
#define I2C_SDA_PIN  39
#define I2C_SCL_PIN  40

// TCA9535 IO Expander
#define IOEXP_ADDR          0x20
#define IOEXP_OUTPUT_PORT0  0x02
#define IOEXP_CONFIG_PORT0  0x06

// IO Expander pins
#define IOEXP_LCD_CS    4
#define IOEXP_LCD_RST   5

// Backlight & SPI pins
#define GFX_BL 45
#define LCD_SPI_SCK   41
#define LCD_SPI_MOSI  48

// RGB panel pins
#define LCD_DE     18
#define LCD_VSYNC  17
#define LCD_HSYNC  16
#define LCD_PCLK   21

// LVGL buffer
#define LVGL_BUFFER_LINES 120

// Global objects
static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t *buf1 = nullptr;
static uint16_t *lcd_framebuffer = nullptr;
static uint8_t ioexp_port0_cache = 0xFF;

// UI elements
static lv_obj_t *status_label = nullptr;
static lv_obj_t *count_label = nullptr;
static lv_obj_t *instruction_label = nullptr;
static lv_obj_t *indicator = nullptr;

// Button state
static volatile bool button_pressed = false;
static volatile uint32_t press_count = 0;
static volatile unsigned long last_press_time = 0;
static bool last_button_state = true;  // HIGH = not pressed

// LVGL tick
static unsigned long lvgl_last_tick = 0;

//=============================================================================
// IO Expander Functions
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
    uint8_t reg = IOEXP_CONFIG_PORT0 + (pin / 8);
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    val &= ~(1 << bit);
    ioexp_write_reg(reg, val);
}

void ioexp_write_pin(uint8_t pin, bool level) {
    uint8_t bit = pin % 8;
    if (level) {
        ioexp_port0_cache |= (1 << bit);
    } else {
        ioexp_port0_cache &= ~(1 << bit);
    }
    ioexp_write_reg(IOEXP_OUTPUT_PORT0, ioexp_port0_cache);
}

//=============================================================================
// Software SPI for ST7701
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
// ST7701S Init (abbreviated - same as test_lvgl.cpp)
//=============================================================================

void st7701_init(void) {
    lcd_write_cmd(0xFF); lcd_write_data(0x77); lcd_write_data(0x01);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x10);
    lcd_write_cmd(0xC0); lcd_write_data(0x3B); lcd_write_data(0x00);
    lcd_write_cmd(0xC1); lcd_write_data(0x0D); lcd_write_data(0x02);
    lcd_write_cmd(0xC2); lcd_write_data(0x31); lcd_write_data(0x05);
    lcd_write_cmd(0xCD); lcd_write_data(0x08);

    lcd_write_cmd(0xB0);
    lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x18); lcd_write_data(0x0E);
    lcd_write_data(0x11); lcd_write_data(0x06); lcd_write_data(0x07); lcd_write_data(0x08);
    lcd_write_data(0x07); lcd_write_data(0x22); lcd_write_data(0x04); lcd_write_data(0x12);
    lcd_write_data(0x0F); lcd_write_data(0xAA); lcd_write_data(0x31); lcd_write_data(0x18);

    lcd_write_cmd(0xB1);
    lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x19); lcd_write_data(0x0E);
    lcd_write_data(0x12); lcd_write_data(0x07); lcd_write_data(0x08); lcd_write_data(0x08);
    lcd_write_data(0x08); lcd_write_data(0x22); lcd_write_data(0x04); lcd_write_data(0x11);
    lcd_write_data(0x11); lcd_write_data(0xA9); lcd_write_data(0x32); lcd_write_data(0x18);

    lcd_write_cmd(0xFF); lcd_write_data(0x77); lcd_write_data(0x01);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x11);
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

    lcd_write_cmd(0xE0); lcd_write_data(0x00); lcd_write_data(0x1B); lcd_write_data(0x02);
    lcd_write_cmd(0xE1);
    lcd_write_data(0x08); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0x07); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0x00); lcd_write_data(0x44); lcd_write_data(0x44);
    lcd_write_cmd(0xE2);
    lcd_write_data(0x11); lcd_write_data(0x11); lcd_write_data(0x44); lcd_write_data(0x44);
    lcd_write_data(0xED); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_data(0xEC); lcd_write_data(0xA0); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_cmd(0xE3); lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x11);
    lcd_write_cmd(0xE4); lcd_write_data(0x44); lcd_write_data(0x44);
    lcd_write_cmd(0xE5);
    lcd_write_data(0x0A); lcd_write_data(0xE9); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0C); lcd_write_data(0xEB); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0E); lcd_write_data(0xED); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x10); lcd_write_data(0xEF); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_cmd(0xE6); lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x11); lcd_write_data(0x11);
    lcd_write_cmd(0xE7); lcd_write_data(0x44); lcd_write_data(0x44);
    lcd_write_cmd(0xE8);
    lcd_write_data(0x09); lcd_write_data(0xE8); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0B); lcd_write_data(0xEA); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0D); lcd_write_data(0xEC); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_data(0x0F); lcd_write_data(0xEE); lcd_write_data(0xD8); lcd_write_data(0xA0);
    lcd_write_cmd(0xEB);
    lcd_write_data(0x02); lcd_write_data(0x00); lcd_write_data(0xE4); lcd_write_data(0xE4);
    lcd_write_data(0x88); lcd_write_data(0x00); lcd_write_data(0x40);
    lcd_write_cmd(0xEC); lcd_write_data(0x3C); lcd_write_data(0x00);
    lcd_write_cmd(0xED);
    lcd_write_data(0xAB); lcd_write_data(0x89); lcd_write_data(0x76); lcd_write_data(0x54);
    lcd_write_data(0x02); lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0xFF);
    lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0xFF); lcd_write_data(0x20);
    lcd_write_data(0x45); lcd_write_data(0x67); lcd_write_data(0x98); lcd_write_data(0xBA);

    lcd_write_cmd(0xFF); lcd_write_data(0x77); lcd_write_data(0x01);
    lcd_write_data(0x00); lcd_write_data(0x00); lcd_write_data(0x00);
    lcd_write_cmd(0x3A); lcd_write_data(0x60);
    lcd_write_cmd(0x36); lcd_write_data(0x00);
    lcd_write_cmd(0x21);
    lcd_write_cmd(0x11); delay(120);
    lcd_write_cmd(0x29); delay(20);
}

//=============================================================================
// LVGL Flush Callback
//=============================================================================

void lvgl_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    if (lcd_framebuffer != nullptr) {
        uint16_t *src = (uint16_t *)color_p;
        // Apply 180° rotation for full screen updates
        if (w == SCREEN_WIDTH && h == SCREEN_HEIGHT) {
            uint32_t total_pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
            uint16_t *dst = lcd_framebuffer + total_pixels - 1;
            for (uint32_t i = 0; i < total_pixels; i++) {
                *dst-- = *src++;
            }
        } else {
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
    lv_disp_flush_ready(disp);
}

//=============================================================================
// Hardware Init
//=============================================================================

bool init_hardware(void) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    Wire.beginTransmission(IOEXP_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[ERROR] IO Expander not found");
        return false;
    }

    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);
    ioexp_set_pin_output(IOEXP_LCD_CS);
    ioexp_set_pin_output(IOEXP_LCD_RST);
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);

    pinMode(LCD_SPI_SCK, OUTPUT);
    pinMode(LCD_SPI_MOSI, OUTPUT);
    digitalWrite(LCD_SPI_SCK, HIGH);

    // Reset LCD
    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    st7701_init();

    rgbpanel = new Arduino_ESP32RGBPanel(
        LCD_DE, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
        4, 3, 2, 1, 0,      // R0-R4
        10, 9, 8, 7, 6, 5,  // G0-G5
        15, 14, 13, 12, 11, // B0-B4
        1, 10, 8, 50,
        1, 10, 8, 20);

    gfx = new Arduino_RGB_Display(SCREEN_WIDTH, SCREEN_HEIGHT, rgbpanel, 0, true);
    if (!gfx->begin()) {
        Serial.println("[ERROR] Display init failed");
        return false;
    }

    lcd_framebuffer = (uint16_t *)gfx->getFramebuffer();
    Serial.printf("[OK] LCD framebuffer: 0x%08X\n", (uint32_t)lcd_framebuffer);

    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);

    gfx->fillScreen(BLACK);

    // Configure user button with internal pull-up
    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
    Serial.println("[OK] User button configured on GPIO38");

    return true;
}

//=============================================================================
// LVGL Init
//=============================================================================

bool init_lvgl(void) {
    lv_init();

    size_t buf_size = SCREEN_WIDTH * LVGL_BUFFER_LINES * sizeof(lv_color_t);
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) {
        buf1 = (lv_color_t *)malloc(buf_size);
    }
    if (!buf1) return false;

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_WIDTH * LVGL_BUFFER_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_180;
    lv_disp_drv_register(&disp_drv);

    return true;
}

//=============================================================================
// Create Button Test UI
//=============================================================================

void create_button_test_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "User Button Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    // Instruction
    instruction_label = lv_label_create(scr);
    lv_label_set_text(instruction_label, "Press the button on the\nLEFT side of the device");
    lv_obj_set_style_text_color(instruction_label, lv_color_hex(0xcccccc), 0);
    lv_obj_set_style_text_font(instruction_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_align(instruction_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 80);

    // Big circular indicator
    indicator = lv_obj_create(scr);
    lv_obj_set_size(indicator, 200, 200);
    lv_obj_align(indicator, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(indicator, 100, 0);
    lv_obj_set_style_bg_color(indicator, lv_color_hex(0x333355), 0);
    lv_obj_set_style_border_color(indicator, lv_color_hex(0x555577), 0);
    lv_obj_set_style_border_width(indicator, 4, 0);

    // Status text inside indicator
    status_label = lv_label_create(indicator);
    lv_label_set_text(status_label, "RELEASED");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);
    lv_obj_center(status_label);

    // Press count
    count_label = lv_label_create(scr);
    lv_label_set_text(count_label, "Press count: 0");
    lv_obj_set_style_text_color(count_label, lv_color_hex(0xffe66d), 0);
    lv_obj_set_style_text_font(count_label, &lv_font_montserrat_22, 0);
    lv_obj_align(count_label, LV_ALIGN_BOTTOM_MID, 0, -80);

    // GPIO info
    lv_obj_t *gpio_label = lv_label_create(scr);
    lv_label_set_text(gpio_label, "GPIO38 - Active LOW with pull-up");
    lv_obj_set_style_text_color(gpio_label, lv_color_hex(0x666688), 0);
    lv_obj_set_style_text_font(gpio_label, &lv_font_montserrat_14, 0);
    lv_obj_align(gpio_label, LV_ALIGN_BOTTOM_MID, 0, -40);

    Serial.println("[OK] Button test UI created");
}

//=============================================================================
// Update UI based on button state
//=============================================================================

void update_button_ui(bool pressed) {
    if (pressed) {
        lv_obj_set_style_bg_color(indicator, lv_color_hex(0x00ff88), 0);
        lv_obj_set_style_border_color(indicator, lv_color_hex(0x00cc66), 0);
        lv_label_set_text(status_label, "PRESSED!");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0x000000), 0);
    } else {
        lv_obj_set_style_bg_color(indicator, lv_color_hex(0x333355), 0);
        lv_obj_set_style_border_color(indicator, lv_color_hex(0x555577), 0);
        lv_label_set_text(status_label, "RELEASED");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0xaaaaaa), 0);
    }

    static char count_buf[32];
    snprintf(count_buf, sizeof(count_buf), "Press count: %lu", press_count);
    lv_label_set_text(count_label, count_buf);
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("User Button Test - SenseCAP Indicator");
    Serial.println("========================================\n");

    if (!init_hardware()) {
        Serial.println("[FATAL] Hardware init failed");
        while (1) delay(1000);
    }

    if (!init_lvgl()) {
        Serial.println("[FATAL] LVGL init failed");
        while (1) delay(1000);
    }

    create_button_test_ui();
    lvgl_last_tick = millis();

    Serial.println("\n[OK] Ready - Press the user button!\n");
}

void loop() {
    unsigned long now = millis();
    lv_tick_inc(now - lvgl_last_tick);
    lvgl_last_tick = now;

    // Read button state (LOW = pressed due to pull-up)
    bool current_state = digitalRead(USER_BUTTON_PIN);

    // Detect state change with debounce
    if (current_state != last_button_state) {
        if (now - last_press_time > 50) {  // 50ms debounce
            last_button_state = current_state;
            last_press_time = now;

            if (!current_state) {
                // Button just pressed
                press_count++;
                Serial.printf("[BUTTON] PRESSED! Count: %lu\n", press_count);
                update_button_ui(true);
            } else {
                // Button just released
                Serial.println("[BUTTON] Released");
                update_button_ui(false);
            }
        }
    }

    lv_timer_handler();
    yield();
}
