/**
 * @file test_wifi.cpp
 * @brief WiFi Test with LVGL UI and On-Screen Keyboard
 *
 * Tests WiFi connectivity with visual feedback and touch keyboard input
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

// Display configuration
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 480

// I2C pins
#define I2C_SDA_PIN  39
#define I2C_SCL_PIN  40

// TCA9535 IO Expander
#define IOEXP_ADDR          0x20
#define IOEXP_OUTPUT_PORT0  0x02
#define IOEXP_CONFIG_PORT0  0x06
#define IOEXP_LCD_CS    4
#define IOEXP_LCD_RST   5
#define IOEXP_TP_RST    7

// Touch
#define FT6336U_ADDR    0x48
#define FT_REG_TD_STATUS 0x02

// Backlight & SPI pins
#define GFX_BL 45
#define LCD_SPI_SCK   41
#define LCD_SPI_MOSI  48

// RGB panel pins
#define LCD_DE     18
#define LCD_VSYNC  17
#define LCD_HSYNC  16
#define LCD_PCLK   21

// LVGL buffer - full screen for tear-free full_refresh mode
#define LVGL_BUFFER_PIXELS (SCREEN_WIDTH * SCREEN_HEIGHT)

// Global objects
static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;
static lv_color_t *buf1 = nullptr;
static uint16_t *lcd_framebuffer = nullptr;
static uint8_t ioexp_port0_cache = 0xFF;

// UI elements
static lv_obj_t *ssid_ta = nullptr;
static lv_obj_t *pass_ta = nullptr;
static lv_obj_t *keyboard = nullptr;
static lv_obj_t *status_label = nullptr;
static lv_obj_t *connect_btn = nullptr;
static lv_obj_t *info_label = nullptr;

// Touch state
static bool touch_detected = false;
static int16_t touch_last_x = 0;
static int16_t touch_last_y = 0;

// LVGL tick
static unsigned long lvgl_last_tick = 0;

// WiFi state
static bool wifi_connecting = false;
static unsigned long wifi_connect_start = 0;
#define WIFI_TIMEOUT_MS 15000

//=============================================================================
// IO Expander Functions (same as other tests)
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
    if (level) ioexp_port0_cache |= (1 << bit);
    else ioexp_port0_cache &= ~(1 << bit);
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
// ST7701S Init (abbreviated)
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
// Touch Functions
//=============================================================================

bool ft6336u_read_touch(int16_t *x, int16_t *y) {
    uint8_t buf[5];
    Wire.beginTransmission(FT6336U_ADDR);
    Wire.write(FT_REG_TD_STATUS);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)FT6336U_ADDR, (uint8_t)5);
    for (int i = 0; i < 5 && Wire.available(); i++) buf[i] = Wire.read();
    uint8_t num_points = buf[0] & 0x0F;
    if (num_points == 0 || num_points > 2) return false;
    *x = ((buf[1] & 0x0F) << 8) | buf[2];
    *y = ((buf[3] & 0x0F) << 8) | buf[4];
    return true;
}

void ft6336u_init(void) {
    ioexp_set_pin_output(IOEXP_TP_RST);
    ioexp_write_pin(IOEXP_TP_RST, LOW);
    delay(10);
    ioexp_write_pin(IOEXP_TP_RST, HIGH);
    delay(300);
    Wire.beginTransmission(FT6336U_ADDR);
    touch_detected = (Wire.endTransmission() == 0);
    Serial.printf("[TOUCH] %s\n", touch_detected ? "OK" : "Not found");
}

//=============================================================================
// LVGL Callbacks
//=============================================================================

void lvgl_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    if (lcd_framebuffer != nullptr) {
        // Full refresh with 180° rotation - reverse entire buffer
        uint16_t *src = (uint16_t *)color_p;
        uint32_t total_pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
        uint16_t *dst = lcd_framebuffer + total_pixels - 1;
        for (uint32_t i = 0; i < total_pixels; i++) {
            *dst-- = *src++;
        }
    }
    lv_disp_flush_ready(disp);
}

void lvgl_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x, y;
    if (ft6336u_read_touch(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
        // 180° rotation to match manual display rotation
        data->point.x = SCREEN_WIDTH - 1 - x;
        data->point.y = SCREEN_HEIGHT - 1 - y;
        touch_last_x = data->point.x;
        touch_last_y = data->point.y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = touch_last_x;
        data->point.y = touch_last_y;
    }
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

    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    st7701_init();

    rgbpanel = new Arduino_ESP32RGBPanel(
        LCD_DE, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
        4, 3, 2, 1, 0, 10, 9, 8, 7, 6, 5, 15, 14, 13, 12, 11,
        1, 10, 8, 50, 1, 10, 8, 20);

    gfx = new Arduino_RGB_Display(SCREEN_WIDTH, SCREEN_HEIGHT, rgbpanel, 0, true);
    if (!gfx->begin()) return false;

    lcd_framebuffer = (uint16_t *)gfx->getFramebuffer();
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);

    ft6336u_init();
    return true;
}

//=============================================================================
// LVGL Init
//=============================================================================

bool init_lvgl(void) {
    lv_init();

    size_t buf_size = LVGL_BUFFER_PIXELS * sizeof(lv_color_t);
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) buf1 = (lv_color_t *)malloc(buf_size);
    if (!buf1) return false;

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LVGL_BUFFER_PIXELS);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1;  // Tear-free, no sw_rotate (manual rotation in flush)
    lv_disp_drv_register(&disp_drv);

    if (touch_detected) {
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read;
        lv_indev_drv_register(&indev_drv);
    }

    return true;
}

//=============================================================================
// WiFi UI Event Handlers
//=============================================================================

static void ta_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e);

    if (code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(keyboard, ta);
        lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if (code == LV_EVENT_DEFOCUSED) {
        lv_keyboard_set_textarea(keyboard, NULL);
        lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void connect_btn_cb(lv_event_t *e) {
    if (wifi_connecting) return;

    const char *ssid = lv_textarea_get_text(ssid_ta);
    const char *pass = lv_textarea_get_text(pass_ta);

    if (strlen(ssid) == 0) {
        lv_label_set_text(status_label, "Please enter SSID");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0xff6b6b), 0);
        return;
    }

    // Hide keyboard
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);

    // Start connection
    wifi_connecting = true;
    wifi_connect_start = millis();

    lv_label_set_text(status_label, "Connecting...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xffe66d), 0);
    lv_obj_add_state(connect_btn, LV_STATE_DISABLED);

    Serial.printf("[WIFI] Connecting to: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
}

static void update_wifi_info(void) {
    static char buf[128];
    snprintf(buf, sizeof(buf),
        "IP: %s\nRSSI: %d dBm\nChannel: %d",
        WiFi.localIP().toString().c_str(),
        WiFi.RSSI(),
        WiFi.channel());
    lv_label_set_text(info_label, buf);
}

//=============================================================================
// Create WiFi Test UI
//=============================================================================

void create_wifi_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, LV_SYMBOL_WIFI " WiFi Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // SSID Label
    lv_obj_t *ssid_lbl = lv_label_create(scr);
    lv_label_set_text(ssid_lbl, "SSID:");
    lv_obj_set_style_text_color(ssid_lbl, lv_color_hex(0xcccccc), 0);
    lv_obj_align(ssid_lbl, LV_ALIGN_TOP_LEFT, 20, 50);

    // SSID Text Area
    ssid_ta = lv_textarea_create(scr);
    lv_textarea_set_one_line(ssid_ta, true);
    lv_textarea_set_placeholder_text(ssid_ta, "Enter WiFi name");
    lv_obj_set_width(ssid_ta, 350);
    lv_obj_align(ssid_ta, LV_ALIGN_TOP_LEFT, 100, 45);
    lv_obj_add_event_cb(ssid_ta, ta_event_cb, LV_EVENT_ALL, NULL);

    // Password Label
    lv_obj_t *pass_lbl = lv_label_create(scr);
    lv_label_set_text(pass_lbl, "Pass:");
    lv_obj_set_style_text_color(pass_lbl, lv_color_hex(0xcccccc), 0);
    lv_obj_align(pass_lbl, LV_ALIGN_TOP_LEFT, 20, 95);

    // Password Text Area
    pass_ta = lv_textarea_create(scr);
    lv_textarea_set_one_line(pass_ta, true);
    lv_textarea_set_password_mode(pass_ta, true);
    lv_textarea_set_placeholder_text(pass_ta, "Enter password");
    lv_obj_set_width(pass_ta, 350);
    lv_obj_align(pass_ta, LV_ALIGN_TOP_LEFT, 100, 90);
    lv_obj_add_event_cb(pass_ta, ta_event_cb, LV_EVENT_ALL, NULL);

    // Connect Button
    connect_btn = lv_btn_create(scr);
    lv_obj_set_size(connect_btn, 140, 40);
    lv_obj_align(connect_btn, LV_ALIGN_TOP_MID, 0, 140);
    lv_obj_set_style_bg_color(connect_btn, lv_color_hex(0x00ff88), 0);
    lv_obj_t *btn_lbl = lv_label_create(connect_btn);
    lv_label_set_text(btn_lbl, LV_SYMBOL_WIFI " Connect");
    lv_obj_center(btn_lbl);
    lv_obj_add_event_cb(connect_btn, connect_btn_cb, LV_EVENT_CLICKED, NULL);

    // Status Label
    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Enter WiFi credentials above");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_18, 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 190);

    // Info Label (IP, RSSI, etc)
    info_label = lv_label_create(scr);
    lv_label_set_text(info_label, "");
    lv_obj_set_style_text_color(info_label, lv_color_hex(0x4ecdc4), 0);
    lv_obj_set_style_text_font(info_label, &lv_font_montserrat_16, 0);
    lv_obj_align(info_label, LV_ALIGN_TOP_MID, 0, 220);

    // Keyboard (hidden by default)
    keyboard = lv_keyboard_create(scr);
    lv_obj_set_size(keyboard, SCREEN_WIDTH, SCREEN_HEIGHT / 2);
    lv_obj_align(keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);

    Serial.println("[OK] WiFi UI created");
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("WiFi Test - SenseCAP Indicator");
    Serial.println("========================================\n");

    if (!init_hardware()) {
        Serial.println("[FATAL] Hardware init failed");
        while (1) delay(1000);
    }

    if (!init_lvgl()) {
        Serial.println("[FATAL] LVGL init failed");
        while (1) delay(1000);
    }

    create_wifi_ui();
    lvgl_last_tick = millis();

    Serial.println("\n[OK] Ready - Enter WiFi credentials\n");
}

void loop() {
    unsigned long now = millis();
    lv_tick_inc(now - lvgl_last_tick);
    lvgl_last_tick = now;

    // Handle WiFi connection state
    if (wifi_connecting) {
        wl_status_t status = WiFi.status();

        if (status == WL_CONNECTED) {
            wifi_connecting = false;
            lv_label_set_text(status_label, "CONNECTED!");
            lv_obj_set_style_text_color(status_label, lv_color_hex(0x00ff88), 0);
            lv_obj_clear_state(connect_btn, LV_STATE_DISABLED);
            update_wifi_info();
            Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        }
        else if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
            wifi_connecting = false;
            lv_label_set_text(status_label, "Connection FAILED");
            lv_obj_set_style_text_color(status_label, lv_color_hex(0xff6b6b), 0);
            lv_obj_clear_state(connect_btn, LV_STATE_DISABLED);
            lv_label_set_text(info_label, status == WL_NO_SSID_AVAIL ? "SSID not found" : "Wrong password?");
            Serial.println("[WIFI] Connection failed");
        }
        else if (now - wifi_connect_start > WIFI_TIMEOUT_MS) {
            wifi_connecting = false;
            WiFi.disconnect();
            lv_label_set_text(status_label, "TIMEOUT");
            lv_obj_set_style_text_color(status_label, lv_color_hex(0xff6b6b), 0);
            lv_obj_clear_state(connect_btn, LV_STATE_DISABLED);
            lv_label_set_text(info_label, "Connection timed out");
            Serial.println("[WIFI] Timeout");
        }
        else {
            // Show progress dots
            static int dots = 0;
            static unsigned long last_dot = 0;
            if (now - last_dot > 500) {
                dots = (dots + 1) % 4;
                char buf[20];
                snprintf(buf, sizeof(buf), "Connecting%.*s", dots, "...");
                lv_label_set_text(status_label, buf);
                last_dot = now;
            }
        }
    }

    lv_timer_handler();
    delay(5);
}
