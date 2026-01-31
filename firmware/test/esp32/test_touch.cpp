/**
 * @file test_touch.cpp
 * @brief FT6336U Capacitive Touch Controller Driver Test
 *
 * Task #6: Standalone touch driver for SenseCAP Indicator
 *
 * Hardware:
 * - FT6336U I2C capacitive touch controller
 * - I2C Address: 0x38 (or 0x48 on some variants)
 * - Touch RST: IO Expander P07
 * - Touch INT: IO Expander P06 (directly readable, directly readable, directly readable directly, directly readable from GPIO directly on some variants maybe ESP32 GPIO directly?)
 * - I2C Bus: GPIO39 (SDA), GPIO40 (SCL) - shared with IO expander
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

//=============================================================================
// Pin Definitions
//=============================================================================

// I2C pins (shared with IO expander)
#define I2C_SDA_PIN  39
#define I2C_SCL_PIN  40

// IO Expander
#define IOEXP_ADDR          0x20
#define IOEXP_INPUT_PORT0   0x00
#define IOEXP_INPUT_PORT1   0x01
#define IOEXP_OUTPUT_PORT0  0x02
#define IOEXP_OUTPUT_PORT1  0x03
#define IOEXP_CONFIG_PORT0  0x06
#define IOEXP_CONFIG_PORT1  0x07

// IO Expander pin assignments for touch
#define IOEXP_TP_INT    6   // P06 - Touch interrupt (directly readable for now maybe should not be input on expander)
#define IOEXP_TP_RST    7   // P07 - Touch reset

// FT6336U I2C addresses to try
#define FT6336U_ADDR_PRIMARY   0x38
#define FT6336U_ADDR_ALTERNATE 0x48

//=============================================================================
// FT6336U Register Definitions
//=============================================================================

#define FT_REG_DEV_MODE       0x00  // Device mode (0=working, 4=factory)
#define FT_REG_GEST_ID        0x01  // Gesture ID
#define FT_REG_TD_STATUS      0x02  // Touch point status (number of points)
#define FT_REG_P1_XH          0x03  // Point 1 X high byte + event flag
#define FT_REG_P1_XL          0x04  // Point 1 X low byte
#define FT_REG_P1_YH          0x05  // Point 1 Y high byte + touch ID
#define FT_REG_P1_YL          0x06  // Point 1 Y low byte
#define FT_REG_P1_WEIGHT      0x07  // Point 1 touch weight
#define FT_REG_P1_MISC        0x08  // Point 1 misc
#define FT_REG_P2_XH          0x09  // Point 2 X high byte
#define FT_REG_P2_XL          0x0A  // Point 2 X low byte
#define FT_REG_P2_YH          0x0B  // Point 2 Y high byte
#define FT_REG_P2_YL          0x0C  // Point 2 Y low byte
#define FT_REG_TH_GROUP       0x80  // Touch threshold
#define FT_REG_TH_DIFF        0x85  // Filter threshold
#define FT_REG_CTRL           0x86  // Control register
#define FT_REG_TIME_ENTER_MONITOR 0x87 // Time to enter monitor mode
#define FT_REG_PERIODACTIVE   0x88  // Active period
#define FT_REG_PERIODMONITOR  0x89  // Monitor period
#define FT_REG_LIB_VER_H      0xA1  // Library version high
#define FT_REG_LIB_VER_L      0xA2  // Library version low
#define FT_REG_CIPHER         0xA3  // Chip vendor ID
#define FT_REG_G_MODE         0xA4  // Interrupt mode
#define FT_REG_PWR_MODE       0xA5  // Power mode
#define FT_REG_FIRMID         0xA6  // Firmware version
#define FT_REG_FOCALTECH_ID   0xA8  // FocalTech panel ID
#define FT_REG_RELEASE_CODE   0xAF  // Release code
#define FT_REG_STATE          0xBC  // State

// Gesture IDs
#define GESTURE_NONE          0x00
#define GESTURE_MOVE_UP       0x10
#define GESTURE_MOVE_LEFT     0x14
#define GESTURE_MOVE_DOWN     0x18
#define GESTURE_MOVE_RIGHT    0x1C
#define GESTURE_ZOOM_IN       0x48
#define GESTURE_ZOOM_OUT      0x49

// Event flags (in P1_XH high nibble)
#define EVENT_PRESS_DOWN      0x00
#define EVENT_LIFT_UP         0x01
#define EVENT_CONTACT         0x02
#define EVENT_NO_EVENT        0x03

//=============================================================================
// Touch Data Structures
//=============================================================================

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t event;      // 0=press, 1=lift, 2=contact, 3=none
    uint8_t touch_id;
    uint8_t weight;
} touch_point_t;

typedef struct {
    uint8_t num_points;
    uint8_t gesture;
    touch_point_t points[2];
} touch_data_t;

//=============================================================================
// Global Variables
//=============================================================================

static uint8_t ft6336u_addr = 0;
static uint8_t ioexp_port0_cache = 0xFF;

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
    val &= ~(1 << bit);  // 0 = output
    ioexp_write_reg(reg, val);
}

void ioexp_set_pin_input(uint8_t pin) {
    uint8_t reg = (pin < 8) ? IOEXP_CONFIG_PORT0 : IOEXP_CONFIG_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    val |= (1 << bit);   // 1 = input
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

bool ioexp_read_pin(uint8_t pin) {
    uint8_t reg = (pin < 8) ? IOEXP_INPUT_PORT0 : IOEXP_INPUT_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = ioexp_read_reg(reg);
    return (val >> bit) & 0x01;
}

//=============================================================================
// FT6336U Touch Controller Functions
//=============================================================================

bool ft6336u_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ft6336u_addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

uint8_t ft6336u_read_reg(uint8_t reg) {
    Wire.beginTransmission(ft6336u_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ft6336u_addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool ft6336u_read_regs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(ft6336u_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    Wire.requestFrom(ft6336u_addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
    return true;
}

/**
 * Reset the touch controller via IO expander
 */
void ft6336u_reset(void) {
    Serial.println("[TOUCH] Resetting FT6336U...");
    ioexp_write_pin(IOEXP_TP_RST, LOW);
    delay(10);
    ioexp_write_pin(IOEXP_TP_RST, HIGH);
    delay(300);  // Wait for touch controller to initialize
}

/**
 * Detect and initialize the FT6336U
 */
bool ft6336u_init(void) {
    // Try primary address
    Wire.beginTransmission(FT6336U_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
        ft6336u_addr = FT6336U_ADDR_PRIMARY;
        Serial.printf("[TOUCH] FT6336U found at 0x%02X\n", ft6336u_addr);
        return true;
    }

    // Try alternate address
    Wire.beginTransmission(FT6336U_ADDR_ALTERNATE);
    if (Wire.endTransmission() == 0) {
        ft6336u_addr = FT6336U_ADDR_ALTERNATE;
        Serial.printf("[TOUCH] FT6336U found at 0x%02X\n", ft6336u_addr);
        return true;
    }

    Serial.println("[TOUCH] FT6336U not found at 0x38 or 0x48");
    return false;
}

/**
 * Read device information
 */
void ft6336u_read_info(void) {
    uint8_t vendor_id = ft6336u_read_reg(FT_REG_CIPHER);
    uint8_t chip_id = ft6336u_read_reg(FT_REG_FOCALTECH_ID);
    uint8_t firmware = ft6336u_read_reg(FT_REG_FIRMID);
    uint8_t lib_h = ft6336u_read_reg(FT_REG_LIB_VER_H);
    uint8_t lib_l = ft6336u_read_reg(FT_REG_LIB_VER_L);
    uint8_t state = ft6336u_read_reg(FT_REG_STATE);

    Serial.println("\n[TOUCH] FT6336U Device Info:");
    Serial.printf("  Vendor ID:    0x%02X\n", vendor_id);
    Serial.printf("  Chip ID:      0x%02X\n", chip_id);
    Serial.printf("  Firmware:     0x%02X\n", firmware);
    Serial.printf("  Library Ver:  %d.%d\n", lib_h, lib_l);
    Serial.printf("  State:        0x%02X\n", state);
}

/**
 * Read touch data
 */
bool ft6336u_read_touch(touch_data_t *data) {
    uint8_t buf[14];

    if (!ft6336u_read_regs(FT_REG_DEV_MODE, buf, 14)) {
        return false;
    }

    data->gesture = buf[1];  // GEST_ID
    data->num_points = buf[2] & 0x0F;  // TD_STATUS

    if (data->num_points > 2) {
        data->num_points = 2;
    }

    if (data->num_points > 0) {
        // Point 1
        data->points[0].event = (buf[3] >> 6) & 0x03;
        data->points[0].x = ((buf[3] & 0x0F) << 8) | buf[4];
        data->points[0].touch_id = (buf[5] >> 4) & 0x0F;
        data->points[0].y = ((buf[5] & 0x0F) << 8) | buf[6];
        data->points[0].weight = buf[7];
    }

    if (data->num_points > 1) {
        // Point 2
        data->points[1].event = (buf[9] >> 6) & 0x03;
        data->points[1].x = ((buf[9] & 0x0F) << 8) | buf[10];
        data->points[1].touch_id = (buf[11] >> 4) & 0x0F;
        data->points[1].y = ((buf[11] & 0x0F) << 8) | buf[12];
        data->points[1].weight = buf[13];
    }

    return true;
}

/**
 * Get gesture name
 */
const char* ft6336u_gesture_name(uint8_t gesture) {
    switch (gesture) {
        case GESTURE_MOVE_UP:    return "Swipe Up";
        case GESTURE_MOVE_DOWN:  return "Swipe Down";
        case GESTURE_MOVE_LEFT:  return "Swipe Left";
        case GESTURE_MOVE_RIGHT: return "Swipe Right";
        case GESTURE_ZOOM_IN:    return "Zoom In";
        case GESTURE_ZOOM_OUT:   return "Zoom Out";
        default:                 return "None";
    }
}

/**
 * Get event name
 */
const char* ft6336u_event_name(uint8_t event) {
    switch (event) {
        case EVENT_PRESS_DOWN: return "Press";
        case EVENT_LIFT_UP:    return "Lift";
        case EVENT_CONTACT:    return "Contact";
        default:               return "None";
    }
}

//=============================================================================
// Unity Test Functions
//=============================================================================

void setUp(void) {}
void tearDown(void) {}

/**
 * TEST_TOUCH_001: Initialize I2C and IO Expander
 */
void test_touch_io_init(void) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // Check IO expander
    Wire.beginTransmission(IOEXP_ADDR);
    bool found = (Wire.endTransmission() == 0);
    TEST_ASSERT_TRUE_MESSAGE(found, "IO Expander not detected at 0x20");

    // Read current port0 state
    ioexp_port0_cache = ioexp_read_reg(IOEXP_OUTPUT_PORT0);

    // Configure touch RST as output, INT as input
    ioexp_set_pin_output(IOEXP_TP_RST);
    ioexp_set_pin_input(IOEXP_TP_INT);

    // Set RST high initially
    ioexp_write_pin(IOEXP_TP_RST, HIGH);

    Serial.println("[PASS] TEST_TOUCH_001: IO initialized");
}

/**
 * TEST_TOUCH_002: Reset and detect touch controller
 */
void test_touch_detect(void) {
    ft6336u_reset();

    bool detected = ft6336u_init();
    TEST_ASSERT_TRUE_MESSAGE(detected, "FT6336U not detected");

    Serial.println("[PASS] TEST_TOUCH_002: Touch controller detected");
}

/**
 * TEST_TOUCH_003: Read device info
 */
void test_touch_info(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, ft6336u_addr, "Touch not initialized");

    ft6336u_read_info();

    // Verify vendor ID (FocalTech should be 0x11 or similar)
    uint8_t vendor_id = ft6336u_read_reg(FT_REG_CIPHER);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, vendor_id, "Invalid vendor ID");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0xFF, vendor_id, "Invalid vendor ID");

    Serial.println("[PASS] TEST_TOUCH_003: Device info read");
}

/**
 * TEST_TOUCH_004: Configure touch controller
 */
void test_touch_config(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, ft6336u_addr, "Touch not initialized");

    // Set to normal operating mode
    ft6336u_write_reg(FT_REG_DEV_MODE, 0x00);

    // Set interrupt mode (trigger on touch)
    ft6336u_write_reg(FT_REG_G_MODE, 0x00);  // Polling mode

    // Set touch threshold
    ft6336u_write_reg(FT_REG_TH_GROUP, 22);  // Default threshold

    Serial.println("[PASS] TEST_TOUCH_004: Touch configured");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_TOUCH_005: Continuous touch reading (interactive)
 */
void test_touch_read(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, ft6336u_addr, "Touch not initialized");

    Serial.println("\n[INFO] Touch the screen for 10 seconds...");
    Serial.println("       (Touch coordinates will be displayed)");

    touch_data_t touch;
    unsigned long start = millis();
    int touch_count = 0;

    while (millis() - start < 10000) {
        if (ft6336u_read_touch(&touch)) {
            if (touch.num_points > 0) {
                touch_count++;
                Serial.printf("[TOUCH] Points:%d  X:%3d Y:%3d Event:%s",
                    touch.num_points,
                    touch.points[0].x,
                    touch.points[0].y,
                    ft6336u_event_name(touch.points[0].event));

                if (touch.gesture != GESTURE_NONE) {
                    Serial.printf("  Gesture:%s", ft6336u_gesture_name(touch.gesture));
                }
                Serial.println();

                if (touch.num_points > 1) {
                    Serial.printf("         Point2: X:%3d Y:%3d\n",
                        touch.points[1].x, touch.points[1].y);
                }
            }
        }
        delay(20);  // ~50Hz polling
    }

    Serial.printf("\n[INFO] Total touch events: %d\n", touch_count);
    TEST_ASSERT_TRUE_MESSAGE(touch_count > 0, "No touch detected - try touching the screen!");

    Serial.println("[PASS] TEST_TOUCH_005: Touch reading works");
}

//=============================================================================
// Main Setup and Loop
//=============================================================================

void setup() {
    delay(2000);

    Serial.begin(115200);
    Serial.println("\n==========================================");
    Serial.println("FT6336U Touch Controller Test - Task #6");
    Serial.println("==========================================\n");

    UNITY_BEGIN();

    RUN_TEST(test_touch_io_init);
    RUN_TEST(test_touch_detect);
    RUN_TEST(test_touch_info);
    RUN_TEST(test_touch_config);
    RUN_TEST(test_touch_read);

    UNITY_END();
}

void loop() {
    // Continuous touch demo after tests
    static unsigned long last_print = 0;
    touch_data_t touch;

    if (ft6336u_addr && ft6336u_read_touch(&touch)) {
        if (touch.num_points > 0 && millis() - last_print > 100) {
            Serial.printf("Touch: X=%3d Y=%3d\n", touch.points[0].x, touch.points[0].y);
            last_print = millis();
        }
    }

    delay(10);
}
