/**
 * @file test_lora.cpp
 * @brief SX1262 LoRa Packet Scanner with LVGL UI for SenseCAP Indicator D1L
 *
 * Uses RadioLib with mverch67's modified Arduino framework that integrates
 * TCA9535 IO Expander into standard digitalWrite/digitalRead.
 *
 * Pin Mapping (from Meshtastic variant.h):
 *   SPI (direct GPIO): SCK=41, MISO=47, MOSI=48
 *   IO Expander (pins | 0x40): CS=IO0, RST=IO1, BUSY=IO2, DIO1=IO3
 *
 * Build: pio run -e esp32s3_test_lora -t upload
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>

//=============================================================================
// Pin Definitions
//=============================================================================

// I2C
#define I2C_SDA         39
#define I2C_SCL         40

// SPI for LoRa
#define LORA_SCK        41
#define LORA_MISO       47
#define LORA_MOSI       48

// IO Expander I2C address
static uint8_t ioexp_addr = 0x20;  // Will try 0x20 first, then 0x39

// TCA9535 Interrupt pin - goes LOW when any input changes
#define PIN_IO_EXP_INT   42

// LoRa pins via IO Expander (manual control)
// These are IO expander pin numbers, not GPIO!
#define LORA_IOEXP_CS    0
#define LORA_IOEXP_RST   1
#define LORA_IOEXP_BUSY  2
#define LORA_IOEXP_DIO1  3

// Virtual GPIO numbers for RadioLib (offset to avoid conflicts)
#define LORA_VIRT_CS     200
#define LORA_VIRT_RST    201
#define LORA_VIRT_BUSY   202
#define LORA_VIRT_DIO1   203

// LCD pins
#define GFX_BL          45
#define LCD_DE          18
#define LCD_VSYNC       17
#define LCD_HSYNC       16
#define LCD_PCLK        21

// Touch
#define TOUCH_ADDR      0x48

// Screen
#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   480

// LCD SPI (direct GPIO - NOT through IO expander!)
#define LCD_SPI_SCK   41
#define LCD_SPI_MOSI  48

// IO Expander pin assignments
#define IOEXP_LCD_CS  4
#define IOEXP_LCD_RST 5
#define IOEXP_TP_RST  7

// IO Expander registers
#define IOEXP_INPUT_PORT0   0x00
#define IOEXP_OUTPUT_PORT0  0x02
#define IOEXP_CONFIG_PORT0  0x06

static uint8_t ioexp_port0_cache = 0xFF;

// Forward declarations
void ioexp_write_reg(uint8_t reg, uint8_t val);
uint8_t ioexp_read_reg(uint8_t reg);
void ioexp_set_pin_output(uint8_t pin);
void ioexp_set_pin_input(uint8_t pin);
void ioexp_write_pin(uint8_t pin, bool level);
bool ioexp_read_pin(uint8_t pin);

//=============================================================================
// Globals
//=============================================================================

// RadioLib - will be initialized in setup() after IO expander is ready
SX1262 *radio = nullptr;

// Display
static Arduino_ESP32RGBPanel *rgbpanel = nullptr;
static Arduino_RGB_Display *gfx = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;
static lv_color_t *buf1 = nullptr;
static uint16_t *lcd_framebuffer = nullptr;
static bool touch_ok = false;

// UI Elements
static lv_obj_t *lbl_status = nullptr;
static lv_obj_t *lbl_freq = nullptr;
static lv_obj_t *lbl_packets = nullptr;
static lv_obj_t *txt_log = nullptr;
static lv_obj_t *btn_scan = nullptr;

// State
static bool lora_ok = false;
static bool scanning = false;
static int packet_count = 0;
static float current_freq = 906.875;  // Meshtastic US primary

// Interrupt-driven IO expander support
// The TCA9535 INT pin (GPIO42) goes LOW when DIO1 or BUSY change state
static volatile bool ioexp_interrupt_pending = false;
static volatile uint32_t ioexp_interrupt_count = 0;

// ISR for TCA9535 interrupt - must be in IRAM
void IRAM_ATTR ioexp_interrupt_isr() {
    ioexp_interrupt_pending = true;
    ioexp_interrupt_count++;
}

// Frequency presets - Meshtastic US primary channel is 906.875 MHz
static const float freq_presets[] = {906.875, 903.08, 915.0, 868.0};
static const char* freq_names[] = {"906.9 Mesh", "903.1 Mesh", "915 US", "868 EU"};
static int current_preset = 0;  // Start on Meshtastic frequency

// Meshtastic LoRa settings (LongFast preset)
#define MESHTASTIC_SYNC_WORD  0x2B
#define MESHTASTIC_SF         11
#define MESHTASTIC_BW         250.0
#define MESHTASTIC_CR         5

//=============================================================================
// IO Expander (for LCD init - LoRa uses framework's built-in support)
//=============================================================================

void ioexp_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ioexp_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t ioexp_read_reg(uint8_t reg) {
    Wire.beginTransmission(ioexp_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ioexp_addr, (uint8_t)1);
    return Wire.read();
}

bool ioexp_detect() {
    // Try 0x20 first (like working soundboard code)
    Wire.beginTransmission(0x20);
    if (Wire.endTransmission() == 0) {
        ioexp_addr = 0x20;
        Serial.println("[IOEXP] Found at 0x20");
        return true;
    }
    // Try 0x39
    Wire.beginTransmission(0x39);
    if (Wire.endTransmission() == 0) {
        ioexp_addr = 0x39;
        Serial.println("[IOEXP] Found at 0x39");
        return true;
    }
    Serial.println("[IOEXP] Not found!");
    return false;
}

// Read IO expander input pin
bool ioexp_read_pin(uint8_t pin) {
    uint8_t reg = IOEXP_INPUT_PORT0 + (pin / 8);
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    return (val >> bit) & 1;
}

// Set IO expander pin as input
void ioexp_set_pin_input(uint8_t pin) {
    uint8_t reg = IOEXP_CONFIG_PORT0 + (pin / 8);
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    val |= (1 << bit);  // 1 = input
    ioexp_write_reg(reg, val);
}

//=============================================================================
// Custom RadioLib HAL for IO Expander pins
//=============================================================================

class SenseCapHal : public RadioLibHal {
public:
    SenseCapHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin >= 200) {
            // Virtual pin - IO expander
            uint8_t iopin = pin - 200;
            if (mode == OUTPUT) {
                ioexp_set_pin_output(iopin);
            } else {
                ioexp_set_pin_input(iopin);
            }
        } else {
            // Regular GPIO
            ::pinMode(pin, mode);
        }
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin >= 200) {
            // Virtual pin - IO expander
            uint8_t iopin = pin - 200;
            ioexp_write_pin(iopin, value == HIGH);
        } else {
            // Regular GPIO
            ::digitalWrite(pin, value);
        }
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin >= 200) {
            // Virtual pin - IO expander
            uint8_t iopin = pin - 200;
            return ioexp_read_pin(iopin) ? HIGH : LOW;
        } else {
            // Regular GPIO
            return ::digitalRead(pin);
        }
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
        if (interruptNum >= 200) {
            // Virtual pin on IO expander - handled via GPIO42 interrupt
            Serial.println("[HAL] DIO1 interrupt routed through GPIO42");
        } else {
            ::attachInterrupt(interruptNum, interruptCb, mode);
        }
    }

    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum < 200) {
            ::detachInterrupt(interruptNum);
        }
    }

    void delay(unsigned long ms) override {
        ::delay(ms);
    }

    void delayMicroseconds(unsigned long us) override {
        ::delayMicroseconds(us);
    }

    unsigned long millis() override {
        return ::millis();
    }

    unsigned long micros() override {
        return ::micros();
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
        return ::pulseIn(pin, state, timeout);
    }

    void spiBegin() override {
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
    }

    void spiBeginTransaction() override {
        SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        for (size_t i = 0; i < len; i++) {
            in[i] = SPI.transfer(out[i]);
        }
    }

    void spiEndTransaction() override {
        SPI.endTransaction();
    }

    void spiEnd() override {
        SPI.end();
    }

    void tone(uint32_t pin, unsigned int frequency, unsigned long duration = 0) override {
        (void)pin; (void)frequency; (void)duration;
    }

    void noTone(uint32_t pin) override {
        (void)pin;
    }

    void yield() override {
        ::yield();
    }

    uint32_t pinToInterrupt(uint32_t pin) override {
        return pin;
    }
};

static SenseCapHal *customHal = nullptr;

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
// Software SPI for ST7701 (direct GPIO - fast!)
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
// ST7701S Full Initialization
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

bool touch_read(int16_t *x, int16_t *y) {
    if (!touch_ok) return false;

    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)5);

    uint8_t n = Wire.read() & 0x0F;
    if (n == 0) return false;

    uint8_t xh = Wire.read(), xl = Wire.read();
    uint8_t yh = Wire.read(), yl = Wire.read();

    *x = ((xh & 0x0F) << 8) | xl;
    *y = ((yh & 0x0F) << 8) | yl;

    // Rotate 180 degrees
    *x = SCREEN_WIDTH - 1 - *x;
    *y = SCREEN_HEIGHT - 1 - *y;

    return true;
}

void touch_init() {
    // Reset touch via IO expander pin 7
    ioexp_set_pin_output(IOEXP_TP_RST);
    ioexp_write_pin(IOEXP_TP_RST, LOW);
    delay(10);
    ioexp_write_pin(IOEXP_TP_RST, HIGH);
    delay(300);

    Wire.beginTransmission(TOUCH_ADDR);
    touch_ok = (Wire.endTransmission() == 0);
}

//=============================================================================
// LVGL Callbacks
//=============================================================================

void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    if (lcd_framebuffer) {
        // Simple 180-degree rotation: reverse entire buffer
        uint16_t *src = (uint16_t *)color_p;
        uint32_t total = SCREEN_WIDTH * SCREEN_HEIGHT;
        uint16_t *dst = lcd_framebuffer + total - 1;
        for (uint32_t i = 0; i < total; i++) {
            *dst-- = *src++;
        }
    }
    lv_disp_flush_ready(drv);
}

void lvgl_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    static int16_t last_x = 0, last_y = 0;
    int16_t x, y;

    if (touch_read(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = last_x = x;
        data->point.y = last_y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_x;
        data->point.y = last_y;
    }
}

//=============================================================================
// Display Init
//=============================================================================

bool init_display() {
    // Detect IO Expander - try 0x20 first (like soundboard), then 0x39
    Wire.beginTransmission(0x20);
    if (Wire.endTransmission() == 0) {
        ioexp_addr = 0x20;
        Serial.println("[IOEXP] Found at 0x20");
    } else {
        Wire.beginTransmission(0x39);
        if (Wire.endTransmission() == 0) {
            ioexp_addr = 0x39;
            Serial.println("[IOEXP] Found at 0x39");
        } else {
            Serial.println("[IOEXP] Not found!");
            return false;
        }
    }

    // Init IO Expander cache
    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);

    // Setup IO expander pins
    ioexp_set_pin_output(IOEXP_LCD_CS);
    ioexp_set_pin_output(IOEXP_LCD_RST);
    ioexp_write_pin(IOEXP_LCD_CS, HIGH);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);

    // Setup direct GPIO for LCD SPI
    pinMode(LCD_SPI_SCK, OUTPUT);
    pinMode(LCD_SPI_MOSI, OUTPUT);
    digitalWrite(LCD_SPI_SCK, HIGH);

    // Reset LCD
    ioexp_write_pin(IOEXP_LCD_RST, LOW);
    delay(20);
    ioexp_write_pin(IOEXP_LCD_RST, HIGH);
    delay(150);

    // Full ST7701S initialization
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

    touch_init();
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
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_read;
    lv_indev_drv_register(&indev_drv);

    return true;
}

//=============================================================================
// LoRa Functions
//=============================================================================

void log_msg(const char* msg) {
    if (txt_log) {
        lv_textarea_add_text(txt_log, msg);
        lv_textarea_add_text(txt_log, "\n");
        lv_textarea_set_cursor_pos(txt_log, LV_TEXTAREA_CURSOR_LAST);
    }
    Serial.println(msg);
}

bool lora_init() {
    Serial.println("Initializing LoRa...");
    log_msg("Init LoRa SX1262...");

    // Create custom HAL for IO expander pin routing
    customHal = new SenseCapHal();

    // Create radio module with custom HAL and virtual pins for IO expander
    Module *mod = new Module(customHal, LORA_VIRT_CS, LORA_VIRT_DIO1, LORA_VIRT_RST, LORA_VIRT_BUSY);
    radio = new SX1262(mod);

    // Initialize SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

    // Try with TCXO 2.4V (SenseCAP has TCXO)
    log_msg("Configuring for Meshtastic...");

    // Meshtastic LongFast: SF11, BW250, CR4/5, sync 0x2B
    int state = radio->begin(
        current_freq,           // 906.875 MHz
        MESHTASTIC_BW,          // 250 kHz
        MESHTASTIC_SF,          // SF11
        MESHTASTIC_CR,          // CR 4/5
        MESHTASTIC_SYNC_WORD,   // 0x2B
        10,                     // output power
        8,                      // preamble length
        2.4                     // TCXO voltage
    );

    if (state != RADIOLIB_ERR_NONE) {
        char buf[64];
        snprintf(buf, sizeof(buf), "FAIL: error %d", state);
        log_msg(buf);
        return false;
    }

    log_msg("OK - Meshtastic LongFast");
    char buf[48];
    snprintf(buf, sizeof(buf), "%.3f MHz SF%d BW%.0f", current_freq, MESHTASTIC_SF, MESHTASTIC_BW);
    log_msg(buf);

    radio->setDio2AsRfSwitch(true);
    log_msg("Ready to scan!");
    return true;
}

//=============================================================================
// UI Callbacks
//=============================================================================

void btn_scan_cb(lv_event_t *e) {
    if (!radio) return;
    if (scanning) {
        radio->standby();
        scanning = false;
        lv_label_set_text(lv_obj_get_child(btn_scan, 0), LV_SYMBOL_PLAY " Start");
        lv_label_set_text(lbl_status, "Stopped");
        lv_obj_set_style_bg_color(btn_scan, lv_color_hex(0x006644), 0);
    } else {
        int state = radio->startReceive();
        if (state == RADIOLIB_ERR_NONE) {
            scanning = true;
            lv_label_set_text(lv_obj_get_child(btn_scan, 0), LV_SYMBOL_STOP " Stop");
            lv_label_set_text(lbl_status, "Scanning...");
            lv_obj_set_style_bg_color(btn_scan, lv_color_hex(0x664400), 0);
        }
    }
}

void btn_freq_cb(lv_event_t *e) {
    if (!radio) return;
    current_preset = (current_preset + 1) % 4;
    current_freq = freq_presets[current_preset];
    radio->setFrequency(current_freq);

    char buf[32];
    snprintf(buf, sizeof(buf), "%s MHz", freq_names[current_preset]);
    lv_label_set_text(lbl_freq, buf);
}

void btn_tx_cb(lv_event_t *e) {
    if (!radio) return;
    if (scanning) {
        radio->standby();
        scanning = false;
        lv_label_set_text(lv_obj_get_child(btn_scan, 0), LV_SYMBOL_PLAY " Start");
    }

    log_msg("TX: Hello LoRa!");
    int state = radio->transmit("Hello from SenseCAP!");

    if (state == RADIOLIB_ERR_NONE) {
        log_msg("TX OK");
    } else {
        char buf[32];
        snprintf(buf, sizeof(buf), "TX FAIL: %d", state);
        log_msg(buf);
    }
}

void btn_clear_cb(lv_event_t *e) {
    lv_textarea_set_text(txt_log, "");
    packet_count = 0;
    lv_label_set_text(lbl_packets, "Packets: 0");
}

//=============================================================================
// UI Setup
//=============================================================================

void create_ui() {
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x1a1a2e), 0);

    // Title
    lv_obj_t *title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, LV_SYMBOL_WIFI " LoRa Scanner");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff88), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status
    lbl_status = lv_label_create(lv_scr_act());
    lv_label_set_text(lbl_status, "Initializing...");
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xffffff), 0);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 15, 50);

    // Frequency
    lbl_freq = lv_label_create(lv_scr_act());
    char buf[32];
    snprintf(buf, sizeof(buf), "%s MHz", freq_names[current_preset]);
    lv_label_set_text(lbl_freq, buf);
    lv_obj_set_style_text_color(lbl_freq, lv_color_hex(0x88ccff), 0);
    lv_obj_align(lbl_freq, LV_ALIGN_TOP_LEFT, 15, 75);

    // Packet count
    lbl_packets = lv_label_create(lv_scr_act());
    lv_label_set_text(lbl_packets, "Packets: 0");
    lv_obj_set_style_text_color(lbl_packets, lv_color_hex(0xffcc00), 0);
    lv_obj_align(lbl_packets, LV_ALIGN_TOP_RIGHT, -15, 50);

    // Log area
    txt_log = lv_textarea_create(lv_scr_act());
    lv_obj_set_size(txt_log, 450, 260);
    lv_obj_align(txt_log, LV_ALIGN_TOP_MID, 0, 105);
    lv_textarea_set_text(txt_log, "");
    lv_obj_set_style_bg_color(txt_log, lv_color_hex(0x0a0a15), 0);
    lv_obj_set_style_text_color(txt_log, lv_color_hex(0x00ff00), 0);
    lv_obj_set_style_text_font(txt_log, &lv_font_montserrat_14, 0);
    lv_obj_set_style_border_color(txt_log, lv_color_hex(0x333366), 0);

    // Buttons - row 1
    btn_scan = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_scan, 140, 50);
    lv_obj_align(btn_scan, LV_ALIGN_BOTTOM_LEFT, 15, -70);
    lv_obj_set_style_bg_color(btn_scan, lv_color_hex(0x006644), 0);
    lv_obj_t *lbl = lv_label_create(btn_scan);
    lv_label_set_text(lbl, LV_SYMBOL_PLAY " Start");
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn_scan, btn_scan_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_tx = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_tx, 140, 50);
    lv_obj_align(btn_tx, LV_ALIGN_BOTTOM_MID, 0, -70);
    lv_obj_set_style_bg_color(btn_tx, lv_color_hex(0x446600), 0);
    lbl = lv_label_create(btn_tx);
    lv_label_set_text(lbl, LV_SYMBOL_UPLOAD " Transmit");
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn_tx, btn_tx_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_freq = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_freq, 140, 50);
    lv_obj_align(btn_freq, LV_ALIGN_BOTTOM_RIGHT, -15, -70);
    lv_obj_set_style_bg_color(btn_freq, lv_color_hex(0x444466), 0);
    lbl = lv_label_create(btn_freq);
    lv_label_set_text(lbl, LV_SYMBOL_LOOP " Freq");
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn_freq, btn_freq_cb, LV_EVENT_CLICKED, NULL);

    // Button row 2
    lv_obj_t *btn_clear = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_clear, 220, 50);
    lv_obj_align(btn_clear, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(btn_clear, lv_color_hex(0x663333), 0);
    lbl = lv_label_create(btn_clear);
    lv_label_set_text(lbl, LV_SYMBOL_TRASH " Clear Log");
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn_clear, btn_clear_cb, LV_EVENT_CLICKED, NULL);
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LoRa Scanner ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!init_display()) {
        Serial.println("Display init failed!");
        while(1) delay(100);
    }

    if (!init_lvgl()) {
        Serial.println("LVGL init failed!");
        while(1) delay(100);
    }

    create_ui();
    lv_timer_handler();

    // Init LoRa
    lora_ok = lora_init();

    // Setup GPIO42 interrupt for IO expander (TCA9535 INT pin)
    // This triggers when DIO1 or BUSY change state - no more polling!
    pinMode(PIN_IO_EXP_INT, INPUT_PULLUP);
    attachInterrupt(PIN_IO_EXP_INT, ioexp_interrupt_isr, FALLING);
    Serial.println("[INT] GPIO42 interrupt attached");

    // Clear any pending interrupt by reading input register
    ioexp_read_reg(IOEXP_INPUT_PORT0);

    if (lora_ok) {
        lv_label_set_text(lbl_status, "Ready");
        log_msg("Ready - tap Start to scan");
        log_msg("Using GPIO42 interrupt!");
    } else {
        lv_label_set_text(lbl_status, "LoRa FAILED");
        lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xff4444), 0);
    }
}

static unsigned long last_tick = 0;

void loop() {
    unsigned long now = millis();
    lv_tick_inc(now - last_tick);
    last_tick = now;

    lv_timer_handler();

    // Check for received packets using interrupt flag (NOT polling!)
    // The ISR sets ioexp_interrupt_pending when GPIO42 goes LOW
    if (scanning && lora_ok && radio && ioexp_interrupt_pending) {
        ioexp_interrupt_pending = false;

        // Read input register to get pin states AND clear the interrupt
        uint8_t port0 = ioexp_read_reg(IOEXP_INPUT_PORT0);
        bool dio1_high = (port0 >> LORA_IOEXP_DIO1) & 1;

        // DIO1 HIGH means packet received
        if (dio1_high) {
            uint8_t data[256];
            size_t len = radio->getPacketLength();
            if (len > sizeof(data)) len = sizeof(data);

            int state = radio->readData(data, len);

            if (state == RADIOLIB_ERR_NONE) {
                packet_count++;

                char buf[128];
                snprintf(buf, sizeof(buf), "[%d] %dB RSSI:%.0f SNR:%.1f (INT#%lu)",
                    packet_count, (int)len, radio->getRSSI(), radio->getSNR(),
                    ioexp_interrupt_count);
                log_msg(buf);

                // Hex preview
                char hex[64] = " ";
                int n = (len > 12) ? 12 : len;
                for (int i = 0; i < n; i++) {
                    sprintf(hex + strlen(hex), "%02X ", data[i]);
                }
                if (len > 12) strcat(hex, "...");
                log_msg(hex);

                snprintf(buf, sizeof(buf), "Packets: %d", packet_count);
                lv_label_set_text(lbl_packets, buf);
            }

            radio->startReceive();
        }
    }

    // No fixed delay needed - we respond to interrupts
    // Small yield to allow other tasks
    yield();
}
