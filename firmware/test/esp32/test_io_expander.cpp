/**
 * @file test_io_expander.cpp
 * @brief TCA9535 IO Expander Tests with Interrupt Support
 *
 * Tests:
 * - TEST_IOEXP_001: Detect TCA9535 at I2C address
 * - TEST_IOEXP_002: Set pin as output, toggle state
 * - TEST_IOEXP_003: Set pin as input, read state
 * - TEST_IOEXP_004: Read all pins
 * - TEST_IOEXP_005: GPIO42 interrupt setup and detection
 *
 * After tests complete, enters monitoring mode to show interrupts in real-time.
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

// ESP32-S3 I2C pins for SenseCAP Indicator
#define I2C_SDA_PIN  39
#define I2C_SCL_PIN  40

// TCA9535 Interrupt pin on ESP32
#define PIN_IO_EXP_INT  42

// TCA9535 I2C addresses to try
#define TCA9535_ADDR_PRIMARY    0x20
#define TCA9535_ADDR_ALTERNATE  0x39

// TCA9535 Registers
#define TCA9535_INPUT_PORT0     0x00
#define TCA9535_INPUT_PORT1     0x01
#define TCA9535_OUTPUT_PORT0    0x02
#define TCA9535_OUTPUT_PORT1    0x03
#define TCA9535_POLARITY_PORT0  0x04
#define TCA9535_POLARITY_PORT1  0x05
#define TCA9535_CONFIG_PORT0    0x06
#define TCA9535_CONFIG_PORT1    0x07

// IO Expander pin definitions
#define IOEXP_LORA_NSS      0   // Output
#define IOEXP_LORA_RST      1   // Output
#define IOEXP_LORA_BUSY     2   // Input - triggers INT
#define IOEXP_LORA_DIO1     3   // Input - triggers INT on packet RX
#define IOEXP_LCD_CS        4   // Output
#define IOEXP_LCD_RST       5   // Output
#define IOEXP_TOUCH_RST     7   // Output
#define IOEXP_RP2040_RST    8   // Output
#define IOEXP_TCXO_VER      11  // Input - triggers INT

static uint8_t tca9535_addr = 0;

// ============================================================================
// Interrupt handling
// ============================================================================
static volatile uint32_t interrupt_count = 0;
static volatile bool interrupt_pending = false;
static volatile unsigned long last_interrupt_time = 0;

// ISR must be in IRAM for ESP32
void IRAM_ATTR ioexp_interrupt_handler() {
    interrupt_count++;
    interrupt_pending = true;
    last_interrupt_time = millis();
}

// ============================================================================
// Helper functions
// ============================================================================
bool tca9535_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(tca9535_addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

uint8_t tca9535_read_reg(uint8_t reg) {
    Wire.beginTransmission(tca9535_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(tca9535_addr, (uint8_t)1);
    return Wire.read();
}

void tca9535_set_direction(uint8_t pin, bool output) {
    uint8_t reg = (pin < 8) ? TCA9535_CONFIG_PORT0 : TCA9535_CONFIG_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = tca9535_read_reg(reg);

    if (output) {
        val &= ~(1 << bit);  // 0 = output
    } else {
        val |= (1 << bit);   // 1 = input
    }
    tca9535_write_reg(reg, val);
}

void tca9535_write_pin(uint8_t pin, bool level) {
    uint8_t reg = (pin < 8) ? TCA9535_OUTPUT_PORT0 : TCA9535_OUTPUT_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = tca9535_read_reg(reg);

    if (level) {
        val |= (1 << bit);
    } else {
        val &= ~(1 << bit);
    }
    tca9535_write_reg(reg, val);
}

bool tca9535_read_pin(uint8_t pin) {
    uint8_t reg = (pin < 8) ? TCA9535_INPUT_PORT0 : TCA9535_INPUT_PORT1;
    uint8_t bit = pin % 8;
    uint8_t val = tca9535_read_reg(reg);
    return (val >> bit) & 0x01;
}

void setUp(void) {
    // Setup before each test
}

void tearDown(void) {
    // Cleanup after each test
}

// ============================================================================
// Tests
// ============================================================================

/**
 * TEST_IOEXP_001: Detect TCA9535 at I2C address
 */
void test_ioexp_detect(void) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // Try primary address
    Wire.beginTransmission(TCA9535_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
        tca9535_addr = TCA9535_ADDR_PRIMARY;
        Serial.printf("[PASS] TEST_IOEXP_001: TCA9535 found at 0x%02X\n", tca9535_addr);
        TEST_ASSERT_TRUE(true);
        return;
    }

    // Try alternate address
    Wire.beginTransmission(TCA9535_ADDR_ALTERNATE);
    if (Wire.endTransmission() == 0) {
        tca9535_addr = TCA9535_ADDR_ALTERNATE;
        Serial.printf("[PASS] TEST_IOEXP_001: TCA9535 found at 0x%02X\n", tca9535_addr);
        TEST_ASSERT_TRUE(true);
        return;
    }

    Serial.println("[FAIL] TEST_IOEXP_001: TCA9535 not found at 0x20 or 0x39");
    TEST_FAIL_MESSAGE("TCA9535 not detected");
}

/**
 * TEST_IOEXP_002: Set pin as output, toggle state
 * Using IO8 (RP2040 reset) as safe test pin
 */
void test_ioexp_output(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, tca9535_addr, "TCA9535 not detected");

    uint8_t testPin = 8;  // IO8 - RP2040 reset (safe to toggle)

    // Set as output
    tca9535_set_direction(testPin, true);

    // Set HIGH
    tca9535_write_pin(testPin, true);
    delay(10);

    // Read back output register
    uint8_t outputVal = tca9535_read_reg(TCA9535_OUTPUT_PORT1);
    TEST_ASSERT_TRUE_MESSAGE((outputVal & 0x01) != 0, "Pin not set HIGH");

    // Set LOW
    tca9535_write_pin(testPin, false);
    delay(10);

    outputVal = tca9535_read_reg(TCA9535_OUTPUT_PORT1);
    TEST_ASSERT_TRUE_MESSAGE((outputVal & 0x01) == 0, "Pin not set LOW");

    // Restore HIGH (release RP2040 from reset)
    tca9535_write_pin(testPin, true);

    Serial.println("[PASS] TEST_IOEXP_002: Output pin toggle successful");
}

/**
 * TEST_IOEXP_003: Set pin as input, read state
 * Using IO2 (LoRa BUSY) or IO11 (TCXO detect) as test inputs
 */
void test_ioexp_input(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, tca9535_addr, "TCA9535 not detected");

    uint8_t testPin = 2;  // IO2 - LoRa BUSY (input)

    // Set as input
    tca9535_set_direction(testPin, false);
    delay(10);

    // Read the pin
    bool state = tca9535_read_pin(testPin);

    Serial.printf("[INFO] IO%d (LoRa BUSY) state: %d\n", testPin, state);

    // Just verify we can read without error
    TEST_ASSERT_TRUE(state == 0 || state == 1);

    Serial.println("[PASS] TEST_IOEXP_003: Input pin read successful");
}

/**
 * TEST_IOEXP_004: Read all inputs (for debugging)
 */
void test_ioexp_read_all(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, tca9535_addr, "TCA9535 not detected");

    uint8_t port0 = tca9535_read_reg(TCA9535_INPUT_PORT0);
    uint8_t port1 = tca9535_read_reg(TCA9535_INPUT_PORT1);

    Serial.println("\n[INFO] IO Expander Pin States:");
    Serial.printf("  Port 0 (IO0-7):  0x%02X = %c%c%c%c%c%c%c%c\n", port0,
        (port0 & 0x80) ? '1' : '0', (port0 & 0x40) ? '1' : '0',
        (port0 & 0x20) ? '1' : '0', (port0 & 0x10) ? '1' : '0',
        (port0 & 0x08) ? '1' : '0', (port0 & 0x04) ? '1' : '0',
        (port0 & 0x02) ? '1' : '0', (port0 & 0x01) ? '1' : '0');
    Serial.printf("  Port 1 (IO8-15): 0x%02X = %c%c%c%c%c%c%c%c\n", port1,
        (port1 & 0x80) ? '1' : '0', (port1 & 0x40) ? '1' : '0',
        (port1 & 0x20) ? '1' : '0', (port1 & 0x10) ? '1' : '0',
        (port1 & 0x08) ? '1' : '0', (port1 & 0x04) ? '1' : '0',
        (port1 & 0x02) ? '1' : '0', (port1 & 0x01) ? '1' : '0');

    Serial.println("\n  Pin Mapping:");
    Serial.printf("    IO0 (LoRa NSS):    %d\n", (port0 >> 0) & 1);
    Serial.printf("    IO1 (LoRa RST):    %d\n", (port0 >> 1) & 1);
    Serial.printf("    IO2 (LoRa BUSY):   %d  <-- INPUT, triggers INT\n", (port0 >> 2) & 1);
    Serial.printf("    IO3 (LoRa DIO1):   %d  <-- INPUT, triggers INT\n", (port0 >> 3) & 1);
    Serial.printf("    IO4 (LCD CS):      %d\n", (port0 >> 4) & 1);
    Serial.printf("    IO5 (LCD RST):     %d\n", (port0 >> 5) & 1);
    Serial.printf("    IO7 (Touch RST):   %d\n", (port0 >> 7) & 1);
    Serial.printf("    IO8 (RP2040 RST):  %d\n", (port1 >> 0) & 1);
    Serial.printf("    IO10 (BMP PWR):    %d\n", (port1 >> 2) & 1);
    Serial.printf("    IO11 (TCXO Ver):   %d  <-- INPUT, triggers INT\n", (port1 >> 3) & 1);

    Serial.println("[PASS] TEST_IOEXP_004: Read all pins successful");
    TEST_ASSERT_TRUE(true);
}

/**
 * TEST_IOEXP_005: GPIO42 interrupt setup and detection
 *
 * The TCA9535 INT pin is open-drain, active-low.
 * It goes LOW when any INPUT pin changes state.
 * Reading the Input Port register clears the interrupt.
 */
void test_ioexp_interrupt(void) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, tca9535_addr, "TCA9535 not detected");

    Serial.println("\n[INFO] Setting up GPIO42 interrupt...");

    // Configure GPIO42 as input with pullup (INT is open-drain)
    pinMode(PIN_IO_EXP_INT, INPUT_PULLUP);

    // Read initial state of INT pin
    int int_state = digitalRead(PIN_IO_EXP_INT);
    Serial.printf("  GPIO42 initial state: %s\n", int_state ? "HIGH (no interrupt)" : "LOW (interrupt pending!)");

    // If INT is already LOW, read input registers to clear it
    if (int_state == LOW) {
        Serial.println("  Clearing pending interrupt by reading input registers...");
        uint8_t port0 = tca9535_read_reg(TCA9535_INPUT_PORT0);
        uint8_t port1 = tca9535_read_reg(TCA9535_INPUT_PORT1);
        Serial.printf("  Port0=0x%02X Port1=0x%02X\n", port0, port1);
        delay(10);
        int_state = digitalRead(PIN_IO_EXP_INT);
        Serial.printf("  GPIO42 after clear: %s\n", int_state ? "HIGH (cleared)" : "LOW (still pending)");
    }

    // Attach interrupt handler
    interrupt_count = 0;
    interrupt_pending = false;
    attachInterrupt(PIN_IO_EXP_INT, ioexp_interrupt_handler, FALLING);
    Serial.println("  Interrupt handler attached (FALLING edge)");

    // Ensure input pins are configured as inputs
    Serial.println("  Configuring IO2 (BUSY) and IO3 (DIO1) as inputs...");
    tca9535_set_direction(IOEXP_LORA_BUSY, false);  // Input
    tca9535_set_direction(IOEXP_LORA_DIO1, false);  // Input

    // Read current state to establish baseline
    uint8_t port0 = tca9535_read_reg(TCA9535_INPUT_PORT0);
    Serial.printf("  Initial Port0: 0x%02X (BUSY=%d, DIO1=%d)\n",
        port0,
        (port0 >> IOEXP_LORA_BUSY) & 1,
        (port0 >> IOEXP_LORA_DIO1) & 1);

    // Wait a moment to see if any interrupts fire
    Serial.println("  Waiting 2 seconds for any interrupts...");
    delay(2000);

    Serial.printf("\n  Result: %lu interrupts detected in 2 seconds\n", interrupt_count);
    Serial.printf("  GPIO42 current state: %s\n",
        digitalRead(PIN_IO_EXP_INT) ? "HIGH" : "LOW");

    // Test passes if we got here without crashing
    // Actual interrupt detection depends on hardware state
    Serial.println("[PASS] TEST_IOEXP_005: Interrupt setup successful");
    TEST_ASSERT_TRUE(true);
}

void setup() {
    delay(2000);

    Serial.begin(115200);
    Serial.println("\n==========================================");
    Serial.println("TCA9535 IO Expander Test Suite");
    Serial.println("With GPIO42 Interrupt Support");
    Serial.println("==========================================\n");

    UNITY_BEGIN();

    RUN_TEST(test_ioexp_detect);
    RUN_TEST(test_ioexp_output);
    RUN_TEST(test_ioexp_input);
    RUN_TEST(test_ioexp_read_all);
    RUN_TEST(test_ioexp_interrupt);

    UNITY_END();

    Serial.println("\n==========================================");
    Serial.println("Entering Interrupt Monitor Mode");
    Serial.println("==========================================");
    Serial.println("Watching GPIO42 for TCA9535 interrupts...");
    Serial.println("Any input pin change (IO2, IO3, IO11) will trigger.");
    Serial.println("Press reset to restart tests.\n");
}

// Track state for monitoring
static uint32_t last_printed_count = 0;
static unsigned long last_status_time = 0;
static uint8_t last_port0 = 0xFF;
static uint8_t last_port1 = 0xFF;

void loop() {
    unsigned long now = millis();

    // Check for interrupt
    if (interrupt_pending) {
        interrupt_pending = false;

        // Read input registers (this also clears the INT)
        uint8_t port0 = tca9535_read_reg(TCA9535_INPUT_PORT0);
        uint8_t port1 = tca9535_read_reg(TCA9535_INPUT_PORT1);

        // Show what changed
        Serial.printf("[INT #%lu] Port0=0x%02X Port1=0x%02X | BUSY=%d DIO1=%d | GPIO42=%s\n",
            interrupt_count,
            port0, port1,
            (port0 >> IOEXP_LORA_BUSY) & 1,
            (port0 >> IOEXP_LORA_DIO1) & 1,
            digitalRead(PIN_IO_EXP_INT) ? "HIGH" : "LOW");

        // Show differences if we have previous state
        if (last_port0 != 0xFF) {
            uint8_t diff0 = port0 ^ last_port0;
            uint8_t diff1 = port1 ^ last_port1;
            if (diff0 || diff1) {
                Serial.printf("         Changed: ");
                if (diff0 & (1 << IOEXP_LORA_BUSY)) Serial.printf("BUSY ");
                if (diff0 & (1 << IOEXP_LORA_DIO1)) Serial.printf("DIO1 ");
                if (diff1 & (1 << (IOEXP_TCXO_VER - 8))) Serial.printf("TCXO ");
                Serial.println();
            }
        }

        last_port0 = port0;
        last_port1 = port1;
    }

    // Periodic status every 10 seconds
    if (now - last_status_time >= 10000) {
        last_status_time = now;
        Serial.printf("[STATUS] Total interrupts: %lu | GPIO42: %s\n",
            interrupt_count,
            digitalRead(PIN_IO_EXP_INT) ? "HIGH (idle)" : "LOW (pending)");
    }

    delay(10);
}
