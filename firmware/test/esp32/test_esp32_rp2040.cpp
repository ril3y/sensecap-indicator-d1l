/**
 * @file test_esp32_rp2040.cpp
 * @brief ESP32 <-> RP2040 Integration Test
 *
 * LVGL UI that sends commands to RP2040 over UART.
 * - Soundboard with touch buttons for buzzer melodies
 * - Volume control slider
 * - SD card file browser
 * - Shows RP2040 responses
 */

#include <Arduino.h>
#include <Wire.h>

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

//=============================================================================
// Pin Definitions
//=============================================================================

// Display
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 480
#define I2C_SDA_PIN   39
#define I2C_SCL_PIN   40
#define GFX_BL        45

// LCD RGB pins
#define LCD_DE    18
#define LCD_VSYNC 17
#define LCD_HSYNC 16
#define LCD_PCLK  21

// UART to RP2040 (ESP_GPIO19 <-> RP_GPIO17, ESP_GPIO20 <-> RP_GPIO16)
#define UART_TX_PIN   19  // ESP32 TX -> RP2040 RX (GP17)
#define UART_RX_PIN   20  // ESP32 RX <- RP2040 TX (GP16)
#define UART_BAUD     115200

// IO Expander
#define IOEXP_ADDR        0x20
#define IOEXP_OUTPUT_PORT0 0x02
#define IOEXP_CONFIG_PORT0 0x06
#define IOEXP_LCD_CS   4
#define IOEXP_LCD_RST  5
#define IOEXP_TP_RST   7

// Touch
#define FT6336U_ADDR   0x48

//=============================================================================
// Globals
//=============================================================================

static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;
static lv_color_t *buf1 = nullptr;
static uint16_t *lcd_framebuffer = nullptr;
static uint8_t ioexp_port0_cache = 0xFF;
static bool touch_ok = false;

// UART response buffer
#define RESP_BUF_SIZE 512
static char resp_buf[RESP_BUF_SIZE];
static int resp_pos = 0;

// UI elements
static lv_obj_t *status_label = nullptr;
static lv_obj_t *response_label = nullptr;
static lv_obj_t *volume_slider = nullptr;
static lv_obj_t *volume_label = nullptr;

// Touch state
static int16_t touch_last_x = 0, touch_last_y = 0;

//=============================================================================
// IO Expander
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

#define LCD_SPI_SCK  41
#define LCD_SPI_MOSI 48

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
// ST7701S Init
//=============================================================================

void st7701_init() {
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
// Touch
//=============================================================================

bool ft6336u_read(int16_t *x, int16_t *y) {
    uint8_t buf[5];
    Wire.beginTransmission(FT6336U_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)FT6336U_ADDR, (uint8_t)5);
    for (int i = 0; i < 5 && Wire.available(); i++) buf[i] = Wire.read();
    uint8_t n = buf[0] & 0x0F;
    if (n == 0 || n > 2) return false;
    *x = ((buf[1] & 0x0F) << 8) | buf[2];
    *y = ((buf[3] & 0x0F) << 8) | buf[4];
    return true;
}

void ft6336u_init() {
    ioexp_set_pin_output(IOEXP_TP_RST);
    ioexp_write_pin(IOEXP_TP_RST, LOW);
    delay(10);
    ioexp_write_pin(IOEXP_TP_RST, HIGH);
    delay(300);
    Wire.beginTransmission(FT6336U_ADDR);
    touch_ok = (Wire.endTransmission() == 0);
}

//=============================================================================
// LVGL Callbacks
//=============================================================================

void lvgl_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    if (lcd_framebuffer) {
        uint16_t *src = (uint16_t *)color_p;

        // 180° rotation for all updates (full or partial)
        int32_t rot_x1 = SCREEN_WIDTH - 1 - area->x2;
        int32_t rot_y1 = SCREEN_HEIGHT - 1 - area->y2;

        for (uint32_t sy = 0; sy < h; sy++) {
            for (uint32_t sx = 0; sx < w; sx++) {
                uint16_t pixel = src[sy * w + sx];
                uint32_t dx = rot_x1 + (w - 1 - sx);
                uint32_t dy = rot_y1 + (h - 1 - sy);
                lcd_framebuffer[dy * SCREEN_WIDTH + dx] = pixel;
            }
        }
    }
    lv_disp_flush_ready(disp);
}

void lvgl_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x, y;
    if (ft6336u_read(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
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
// RP2040 Communication
//=============================================================================

void rp2040_send(const char* cmd) {
    Serial.printf("[TX->RP2040] %s\n", cmd);
    Serial1.print(cmd);
    Serial1.print("\r\n");
    Serial1.flush();  // Wait for transmission to complete

    // Update status
    if (status_label) {
        static char buf[64];
        snprintf(buf, sizeof(buf), "Sent: %s", cmd);
        lv_label_set_text(status_label, buf);
    }
}

void rp2040_process_response() {
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (resp_pos > 0) {
                resp_buf[resp_pos] = '\0';
                Serial.printf("[RX] %s\n", resp_buf);

                // Update response label
                if (response_label) {
                    lv_label_set_text(response_label, resp_buf);
                }
                resp_pos = 0;
            }
        } else if (resp_pos < RESP_BUF_SIZE - 1) {
            resp_buf[resp_pos++] = c;
        }
    }
}

//=============================================================================
// UI Event Handlers
//=============================================================================

static void sound_btn_cb(lv_event_t *e) {
    const char* melody = (const char*)lv_event_get_user_data(e);
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "melody %s", melody);
    rp2040_send(cmd);

    // Play tick for button feedback
    rp2040_send("melody tick");
}

static void volume_slider_cb(lv_event_t *e) {
    int val = lv_slider_get_value(volume_slider);
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "vol %d", val);
    rp2040_send(cmd);

    // Update label
    if (volume_label) {
        char buf[16];
        snprintf(buf, sizeof(buf), "Vol: %d%%", val);
        lv_label_set_text(volume_label, buf);
    }
}

static void sd_btn_cb(lv_event_t *e) {
    rp2040_send("ls /");
}

static void status_btn_cb(lv_event_t *e) {
    rp2040_send("status");
}

//=============================================================================
// Create UI
//=============================================================================

lv_obj_t* create_sound_btn(lv_obj_t* parent, const char* label, const char* melody,
                            lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                            uint32_t color) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
    lv_obj_set_style_radius(btn, 10, 0);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, sound_btn_cb, LV_EVENT_CLICKED, (void*)melody);
    return btn;
}

void create_ui() {
    lv_obj_t* scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);  // Disable scrolling

    // Title
    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " RP2040 Soundboard");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Sound buttons grid
    int bw = 105, bh = 50, gap = 10;
    int startx = 20, starty = 50;

    // Row 1 - UI Sounds
    create_sound_btn(scr, "Tick",   "tick",   startx + 0*(bw+gap), starty + 0*(bh+gap), bw, bh, 0x4a69bd);
    create_sound_btn(scr, "Click",  "click",  startx + 1*(bw+gap), starty + 0*(bh+gap), bw, bh, 0x4a69bd);
    create_sound_btn(scr, "Key",    "key",    startx + 2*(bw+gap), starty + 0*(bh+gap), bw, bh, 0x4a69bd);
    create_sound_btn(scr, "Enter",  "enter",  startx + 3*(bw+gap), starty + 0*(bh+gap), bw, bh, 0x4a69bd);

    // Row 2 - Short
    create_sound_btn(scr, "Pip",    "pip",    startx + 0*(bw+gap), starty + 1*(bh+gap), bw, bh, 0x6c5ce7);
    create_sound_btn(scr, "Blip",   "blip",   startx + 1*(bw+gap), starty + 1*(bh+gap), bw, bh, 0x6c5ce7);
    create_sound_btn(scr, "OK",     "ok",     startx + 2*(bw+gap), starty + 1*(bh+gap), bw, bh, 0x00b894);
    create_sound_btn(scr, "No",     "no",     startx + 3*(bw+gap), starty + 1*(bh+gap), bw, bh, 0xd63031);

    // Row 3 - Messages
    create_sound_btn(scr, "Msg",    "msg",    startx + 0*(bw+gap), starty + 2*(bh+gap), bw, bh, 0x0984e3);
    create_sound_btn(scr, "Msg2",   "msg2",   startx + 1*(bw+gap), starty + 2*(bh+gap), bw, bh, 0x0984e3);
    create_sound_btn(scr, "DM",     "dm",     startx + 2*(bw+gap), starty + 2*(bh+gap), bw, bh, 0x0984e3);
    create_sound_btn(scr, "Sent",   "sent",   startx + 3*(bw+gap), starty + 2*(bh+gap), bw, bh, 0x00cec9);

    // Row 4 - Status
    create_sound_btn(scr, "Conn",   "conn",   startx + 0*(bw+gap), starty + 3*(bh+gap), bw, bh, 0x00b894);
    create_sound_btn(scr, "Disc",   "disc",   startx + 1*(bw+gap), starty + 3*(bh+gap), bw, bh, 0xfdcb6e);
    create_sound_btn(scr, "Boot",   "boot",   startx + 2*(bw+gap), starty + 3*(bh+gap), bw, bh, 0x00b894);
    create_sound_btn(scr, "Low",    "low",    startx + 3*(bw+gap), starty + 3*(bh+gap), bw, bh, 0xfdcb6e);

    // Row 5 - Alerts
    create_sound_btn(scr, "Error",  "err",    startx + 0*(bw+gap), starty + 4*(bh+gap), bw, bh, 0xd63031);
    create_sound_btn(scr, "Warn",   "warn",   startx + 1*(bw+gap), starty + 4*(bh+gap), bw, bh, 0xe17055);
    create_sound_btn(scr, "Alert",  "alert",  startx + 2*(bw+gap), starty + 4*(bh+gap), bw, bh, 0xd63031);
    create_sound_btn(scr, "SOS",    "sos",    startx + 3*(bw+gap), starty + 4*(bh+gap), bw, bh, 0xff0000);

    // Volume slider
    int vol_y = starty + 5*(bh+gap) + 10;

    volume_label = lv_label_create(scr);
    lv_label_set_text(volume_label, "Vol: 100%");
    lv_obj_set_style_text_color(volume_label, lv_color_hex(0xcccccc), 0);
    lv_obj_set_pos(volume_label, startx, vol_y);

    volume_slider = lv_slider_create(scr);
    lv_obj_set_pos(volume_slider, startx + 80, vol_y - 5);
    lv_obj_set_size(volume_slider, 300, 20);
    lv_slider_set_range(volume_slider, 0, 100);
    lv_slider_set_value(volume_slider, 100, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(volume_slider, lv_color_hex(0x333355), LV_PART_MAIN);
    lv_obj_set_style_bg_color(volume_slider, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(volume_slider, lv_color_hex(0x00ff88), LV_PART_KNOB);
    lv_obj_add_event_cb(volume_slider, volume_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // SD and Status buttons
    int btn_y = vol_y + 40;

    lv_obj_t* sd_btn = lv_btn_create(scr);
    lv_obj_set_pos(sd_btn, startx, btn_y);
    lv_obj_set_size(sd_btn, 120, 40);
    lv_obj_set_style_bg_color(sd_btn, lv_color_hex(0x636e72), 0);
    lv_obj_t* sd_lbl = lv_label_create(sd_btn);
    lv_label_set_text(sd_lbl, LV_SYMBOL_SD_CARD " List SD");
    lv_obj_center(sd_lbl);
    lv_obj_add_event_cb(sd_btn, sd_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* status_btn = lv_btn_create(scr);
    lv_obj_set_pos(status_btn, startx + 130, btn_y);
    lv_obj_set_size(status_btn, 120, 40);
    lv_obj_set_style_bg_color(status_btn, lv_color_hex(0x636e72), 0);
    lv_obj_t* st_lbl = lv_label_create(status_btn);
    lv_label_set_text(st_lbl, LV_SYMBOL_REFRESH " Status");
    lv_obj_center(st_lbl);
    lv_obj_add_event_cb(status_btn, status_btn_cb, LV_EVENT_CLICKED, NULL);

    // Status label
    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Ready");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x888888), 0);
    lv_obj_set_pos(status_label, startx, btn_y + 50);

    // Response label
    response_label = lv_label_create(scr);
    lv_label_set_text(response_label, "");
    lv_obj_set_style_text_color(response_label, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(response_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(response_label, startx, btn_y + 75);
    lv_obj_set_width(response_label, 440);
    lv_label_set_long_mode(response_label, LV_LABEL_LONG_WRAP);
}

//=============================================================================
// Hardware Init
//=============================================================================

bool init_hardware() {
    // I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // IO Expander
    Wire.beginTransmission(IOEXP_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[ERR] IO Expander not found");
        return false;
    }

    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);
    ioexp_set_pin_output(IOEXP_LCD_CS);
    ioexp_set_pin_output(IOEXP_LCD_RST);
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);

    // LCD SPI pins
    pinMode(LCD_SPI_SCK, OUTPUT);
    pinMode(LCD_SPI_MOSI, OUTPUT);
    digitalWrite(LCD_SPI_SCK, HIGH);

    // LCD Reset
    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    st7701_init();

    // RGB Panel
    rgbpanel = new Arduino_ESP32RGBPanel(
        LCD_DE, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
        4, 3, 2, 1, 0, 10, 9, 8, 7, 6, 5, 15, 14, 13, 12, 11,
        1, 10, 8, 50, 1, 10, 8, 20);

    gfx = new Arduino_RGB_Display(SCREEN_WIDTH, SCREEN_HEIGHT, rgbpanel, 0, true);
    if (!gfx->begin()) return false;

    lcd_framebuffer = (uint16_t*)gfx->getFramebuffer();
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);

    ft6336u_init();
    Serial.printf("[TOUCH] %s\n", touch_ok ? "OK" : "Not found");

    return true;
}

bool init_lvgl() {
    lv_init();

    size_t buf_size = SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(lv_color_t);
    buf1 = (lv_color_t*)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) buf1 = (lv_color_t*)malloc(buf_size);
    if (!buf1) return false;

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_WIDTH * SCREEN_HEIGHT);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 0;  // Partial refresh for better performance
    lv_disp_drv_register(&disp_drv);

    if (touch_ok) {
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read;
        lv_indev_drv_register(&indev_drv);
    }

    return true;
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    delay(500);
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("ESP32 <-> RP2040 Soundboard Test");
    Serial.println("========================================\n");

    // UART to RP2040
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("[UART] Initialized");

    if (!init_hardware()) {
        Serial.println("[FATAL] Hardware init failed");
        while (1) delay(1000);
    }

    if (!init_lvgl()) {
        Serial.println("[FATAL] LVGL init failed");
        while (1) delay(1000);
    }

    create_ui();
    Serial.println("[OK] UI Ready\n");

    // Send initial status request
    delay(500);
    rp2040_send("status");
}

static unsigned long last_tick = 0;

void loop() {
    unsigned long now = millis();
    lv_tick_inc(now - last_tick);
    last_tick = now;

    // Process RP2040 responses
    rp2040_process_response();

    lv_timer_handler();
    delay(5);
}
