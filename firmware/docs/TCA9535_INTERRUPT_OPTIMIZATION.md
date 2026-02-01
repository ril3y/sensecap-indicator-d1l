# TCA9535 IO Expander Interrupt Optimization

## Overview

The SenseCAP Indicator D1L uses a TCA9535 IO expander (I2C address 0x20) to provide additional GPIO pins for LoRa and LCD control. The TCA9535 has an **interrupt output (INT)** connected to **ESP32 GPIO42** that eliminates the need for constant I2C polling.

## The Problem: Inefficient Polling

### Current Implementation

The firmware currently polls the IO expander over I2C to check input pin states:

```cpp
// In loop() - called every 5ms
if (ioexp_read_pin(LORA_IOEXP_DIO1)) {  // I2C transaction!
    // Handle LoRa packet
}
```

**Performance Impact:**
| Metric | Current (Polling) | With Interrupt |
|--------|-------------------|----------------|
| I2C transactions/sec | 200+ | ~1 (only on events) |
| CPU overhead | ~25% on I2C | <1% |
| LoRa latency | 0-5ms | <1ms |
| Power consumption | Higher | Lower (can sleep) |

## The Solution: Interrupt-Driven I/O

### TCA9535 INT Pin Behavior

From the [TCA9535 datasheet](https://www.ti.com/lit/ds/symlink/tca9535.pdf):

- **INT is open-drain, active-low** - goes LOW when any input changes
- **Triggered by any edge** (rising or falling) on input pins
- **Clears automatically** when the Input Port register is read
- Requires external pull-up resistor (already present on PCB)

### Hardware Connection

```
TCA9535 (0x20)           ESP32-S3
+-------------+          +--------+
| INT (pin 1) |--------->| GPIO42 |  (active-low, needs INPUT_PULLUP)
+-------------+          +--------+
```

### Input Pins That Trigger INT

| IO Pin | Function | Triggers INT When |
|--------|----------|-------------------|
| IO2 | LoRa BUSY | Radio starts/finishes TX/RX |
| IO3 | LoRa DIO1 | Packet received (goes HIGH) |
| IO11 | TCXO Detect | Only at startup (static) |

## Implementation

### 1. Define the Interrupt Pin

Already in `sensecap_pins.h`:
```cpp
#define PIN_IO_EXP_INT  42
```

### 2. Create Volatile Flag and ISR

```cpp
// Global volatile flag - modified by ISR, read by main loop
static volatile bool ioexp_interrupt_pending = false;

// ISR must be in IRAM for ESP32
void IRAM_ATTR ioexp_isr() {
    ioexp_interrupt_pending = true;
}
```

### 3. Attach Interrupt in setup()

```cpp
void setup() {
    // ... existing init code ...

    // Configure GPIO42 as input with pullup (INT is open-drain)
    pinMode(PIN_IO_EXP_INT, INPUT_PULLUP);

    // Attach interrupt on falling edge (INT goes LOW on change)
    attachInterrupt(PIN_IO_EXP_INT, ioexp_isr, FALLING);
}
```

### 4. Handle Interrupt in loop()

```cpp
void loop() {
    // ... LVGL tick handling ...

    // Check if IO expander interrupt fired
    if (ioexp_interrupt_pending) {
        ioexp_interrupt_pending = false;

        // Read input register (this also clears the INT)
        uint8_t port0 = ioexp_read_reg(IOEXP_INPUT_PORT0);

        // Check LoRa DIO1 (bit 3) - packet received
        if (port0 & (1 << LORA_IOEXP_DIO1)) {
            handle_lora_packet();
        }

        // Check LoRa BUSY (bit 2) - for state machine
        bool busy = port0 & (1 << LORA_IOEXP_BUSY);
        // ... update radio state ...
    }

    lv_timer_handler();
}
```

### 5. Update RadioLib HAL (Optional)

The `SenseCapHal` class can be updated to use the interrupt:

```cpp
void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
    if (interruptNum >= 200) {
        // Virtual pin on IO expander - route through GPIO42
        // Store callback for later invocation from main loop
        stored_dio1_callback = interruptCb;
        Serial.println("[HAL] DIO1 interrupt routed through GPIO42");
    } else {
        ::attachInterrupt(interruptNum, interruptCb, mode);
    }
}
```

## Complete Example

```cpp
#include <Arduino.h>
#include <Wire.h>

#define PIN_IO_EXP_INT    42
#define IOEXP_ADDR        0x20
#define IOEXP_INPUT_PORT0 0x00
#define LORA_IOEXP_DIO1   3
#define LORA_IOEXP_BUSY   2

static volatile bool ioexp_int_flag = false;

void IRAM_ATTR ioexp_isr() {
    ioexp_int_flag = true;
}

uint8_t ioexp_read_reg(uint8_t reg) {
    Wire.beginTransmission(IOEXP_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IOEXP_ADDR, (uint8_t)1);
    return Wire.read();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(39, 40);  // SDA=39, SCL=40
    Wire.setClock(400000);

    // Configure IO expander interrupt
    pinMode(PIN_IO_EXP_INT, INPUT_PULLUP);
    attachInterrupt(PIN_IO_EXP_INT, ioexp_isr, FALLING);

    Serial.println("IO Expander interrupt enabled on GPIO42");
}

void loop() {
    if (ioexp_int_flag) {
        ioexp_int_flag = false;

        // Read clears the interrupt
        uint8_t port0 = ioexp_read_reg(IOEXP_INPUT_PORT0);

        Serial.printf("INT! PORT0=0x%02X DIO1=%d BUSY=%d\n",
            port0,
            (port0 >> LORA_IOEXP_DIO1) & 1,
            (port0 >> LORA_IOEXP_BUSY) & 1);

        if (port0 & (1 << LORA_IOEXP_DIO1)) {
            Serial.println("LoRa packet received!");
            // radio->readData(...);
        }
    }

    // No need for delay - we wait for interrupt
    yield();
}
```

## Benefits

1. **95%+ reduction in I2C traffic** - Only read on actual events
2. **Sub-millisecond latency** - Detect packets immediately
3. **Lower power consumption** - CPU can idle between events
4. **Improved LVGL responsiveness** - Less I2C bus contention
5. **Scalable** - Adding more inputs doesn't increase polling load

## Caveats

1. **I2C in ISR**: The example above sets a flag; don't do I2C reads in the ISR itself on ESP32 (Wire is not ISR-safe)
2. **Multiple sources**: If both DIO1 and BUSY change simultaneously, you get one interrupt - read the full register to check all pins
3. **Clearing INT**: Reading the input port register automatically clears the INT - don't read it twice
4. **Edge vs Level**: INT indicates "something changed" not "current state" - always read the register to get actual values

## Files to Modify

| File | Changes Needed |
|------|----------------|
| `test_lora.cpp` | Add ISR, replace polling with flag check |
| `test_lvgl.cpp` | Add loop throttling or touch interrupt |
| `test_touch.cpp` | Consider routing FT6336U INT through IO expander |
| `sensecap_pins.h` | Already has `PIN_IO_EXP_INT 42` defined |

## References

- [TCA9535 Datasheet](https://www.ti.com/lit/ds/symlink/tca9535.pdf) - Section 7.3.3 "Interrupt Output"
- [ESP32-S3 GPIO Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html)
- [RadioLib SX1262 Documentation](https://jgromes.github.io/RadioLib/class_s_x1262.html)
