/**
 * @file test_rp2040_full.cpp
 * @brief RP2040 Full Peripheral Test with CLI
 *
 * Comprehensive test firmware for all RP2040 peripherals with
 * command-line interface. Works over USB (direct) or UART (to ESP32).
 *
 * Peripherals:
 * - AHT20 (Temp/Humidity) - I2C 0x38
 * - SGP40 (TVOC) - I2C 0x59
 * - SCD41 (CO2) - I2C 0x62
 * - Buzzer - GP19 PWM
 * - Sensor Power - GP18
 * - SD Card - SPI1 (GP10-13)
 * - Grove ADC - GP26/27
 * - UART to ESP32 - GP16/17
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Sensor libraries
#include <Adafruit_AHTX0.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CScd4x.h>
#include <VOCGasIndexAlgorithm.h>

//=============================================================================
// Pin Definitions
//=============================================================================

// UART to ESP32
#define UART_TX_ESP32   16
#define UART_RX_ESP32   17
#define UART_BAUD       115200

// I2C (sensors)
#define I2C_SDA         20
#define I2C_SCL         21

// SPI1 (SD card)
#define SD_SCK          10
#define SD_MOSI         11
#define SD_MISO         12
#define SD_CS           13

// Other
#define SENSOR_POWER_PIN 18
#define BUZZER_PIN       19
#define GROVE_ADC0       26
#define GROVE_ADC1       27
#define LED_PIN          25

//=============================================================================
// Global Objects
//=============================================================================

// Sensors
Adafruit_AHTX0 aht;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm vocAlgorithm;

// Sensor state
bool aht_ok = false;
bool sgp40_ok = false;
bool scd4x_ok = false;
bool sd_ok = false;
bool sensor_power_on = false;

// CLI buffer
#define CLI_BUFFER_SIZE 128
char cli_buffer[CLI_BUFFER_SIZE];
int cli_pos = 0;

// Which serial port to use for CLI output
Stream* cli_out = &Serial;

//=============================================================================
// Buzzer Functions
//=============================================================================

void buzzer_tone(uint32_t freq, uint32_t duration_ms) {
    if (freq == 0) {
        noTone(BUZZER_PIN);
        return;
    }
    tone(BUZZER_PIN, freq, duration_ms);
}

void buzzer_off() {
    noTone(BUZZER_PIN);
}

void buzzer_beep(int count, int freq, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        buzzer_tone(freq, on_ms);
        delay(on_ms + off_ms);
    }
}

void buzzer_success() {
    buzzer_tone(1000, 100);
    delay(150);
    buzzer_tone(1500, 100);
    delay(150);
    buzzer_tone(2000, 150);
}

void buzzer_error() {
    buzzer_tone(400, 200);
    delay(250);
    buzzer_tone(300, 400);
}

void buzzer_sweep(int start_freq, int end_freq, int duration_ms) {
    int steps = duration_ms / 10;
    float freq_step = (float)(end_freq - start_freq) / steps;
    float freq = start_freq;
    for (int i = 0; i < steps; i++) {
        tone(BUZZER_PIN, (int)freq);
        freq += freq_step;
        delay(10);
    }
    noTone(BUZZER_PIN);
}

//=============================================================================
// Sensor Power Control
//=============================================================================

void sensor_power(bool on) {
    digitalWrite(SENSOR_POWER_PIN, on ? HIGH : LOW);
    sensor_power_on = on;
    if (on) {
        delay(100); // Let sensors power up
    }
}

//=============================================================================
// Sensor Init Functions
//=============================================================================

bool init_aht() {
    aht_ok = aht.begin(&Wire);
    return aht_ok;
}

bool init_sgp40() {
    sgp40.begin(Wire);
    uint16_t serialNumber[3];
    uint16_t error = sgp40.getSerialNumber(serialNumber, 3);
    sgp40_ok = (error == 0);
    return sgp40_ok;
}

bool init_scd4x() {
    scd4x.begin(Wire, 0x62);  // SCD41 I2C address
    scd4x.stopPeriodicMeasurement();
    delay(500);
    uint16_t error = scd4x.startPeriodicMeasurement();
    scd4x_ok = (error == 0);
    return scd4x_ok;
}

bool init_sd() {
    SPI1.setRX(SD_MISO);
    SPI1.setTX(SD_MOSI);
    SPI1.setSCK(SD_SCK);
    sd_ok = SD.begin(SD_CS, SPI1);
    return sd_ok;
}

void init_sensors() {
    cli_out->println("Initializing sensors...");

    sensor_power(true);
    delay(500);

    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(100000); // 100kHz for sensors

    cli_out->printf("  AHT20:  %s\n", init_aht() ? "OK" : "FAIL");
    cli_out->printf("  SGP40:  %s\n", init_sgp40() ? "OK" : "FAIL");
    cli_out->printf("  SCD41:  %s\n", init_scd4x() ? "OK" : "FAIL");
    cli_out->printf("  SD:     %s\n", init_sd() ? "OK" : "FAIL");
}

//=============================================================================
// Sensor Read Functions
//=============================================================================

void read_aht() {
    if (!aht_ok) {
        cli_out->println("[AHT20] Not initialized");
        return;
    }

    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp)) {
        cli_out->printf("[AHT20] Temp: %.2f C, Humidity: %.2f %%\n",
                       temp.temperature, humidity.relative_humidity);
    } else {
        cli_out->println("[AHT20] Read failed");
    }
}

void read_sgp40() {
    if (!sgp40_ok) {
        cli_out->println("[SGP40] Not initialized");
        return;
    }

    uint16_t srawVoc = 0;
    float humidity = 50.0;  // Default if no AHT20
    float temperature = 25.0;

    // Get humidity/temp from AHT20 if available
    if (aht_ok) {
        sensors_event_t h, t;
        if (aht.getEvent(&h, &t)) {
            humidity = h.relative_humidity;
            temperature = t.temperature;
        }
    }

    uint16_t compensationRh = (uint16_t)(humidity * 65535 / 100);
    uint16_t compensationT = (uint16_t)((temperature + 45) * 65535 / 175);

    uint16_t error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
    if (error == 0) {
        int32_t vocIndex = vocAlgorithm.process(srawVoc);
        cli_out->printf("[SGP40] Raw: %u, VOC Index: %ld\n", srawVoc, vocIndex);
    } else {
        cli_out->println("[SGP40] Read failed");
    }
}

void read_scd4x() {
    if (!scd4x_ok) {
        cli_out->println("[SCD41] Not initialized");
        return;
    }

    uint16_t co2;
    float temperature, humidity;
    bool dataReady = false;

    scd4x.getDataReadyStatus(dataReady);
    if (!dataReady) {
        cli_out->println("[SCD41] Data not ready (wait 5s after start)");
        return;
    }

    uint16_t error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error == 0) {
        cli_out->printf("[SCD41] CO2: %u ppm, Temp: %.2f C, Humidity: %.2f %%\n",
                       co2, temperature, humidity);
    } else {
        cli_out->println("[SCD41] Read failed");
    }
}

void read_all_sensors() {
    read_aht();
    read_sgp40();
    read_scd4x();
}

//=============================================================================
// ADC Functions
//=============================================================================

void read_adc(int channel) {
    int pin = (channel == 0) ? GROVE_ADC0 : GROVE_ADC1;
    int raw = analogRead(pin);
    float voltage = raw * 3.3 / 4095.0;
    cli_out->printf("[ADC%d] Raw: %d, Voltage: %.3f V\n", channel, raw, voltage);
}

//=============================================================================
// SD Card Functions
//=============================================================================

void sd_list(const char* path) {
    if (!sd_ok) {
        cli_out->println("[SD] Not initialized");
        return;
    }

    File dir = SD.open(path);
    if (!dir) {
        cli_out->printf("[SD] Cannot open: %s\n", path);
        return;
    }

    cli_out->printf("[SD] Listing: %s\n", path);
    while (File entry = dir.openNextFile()) {
        cli_out->printf("  %s%s  %lu bytes\n",
                       entry.name(),
                       entry.isDirectory() ? "/" : "",
                       entry.size());
        entry.close();
    }
    dir.close();
}

void sd_read(const char* filename) {
    if (!sd_ok) {
        cli_out->println("[SD] Not initialized");
        return;
    }

    File f = SD.open(filename, FILE_READ);
    if (!f) {
        cli_out->printf("[SD] Cannot open: %s\n", filename);
        return;
    }

    cli_out->printf("[SD] Contents of %s:\n", filename);
    while (f.available()) {
        cli_out->write(f.read());
    }
    cli_out->println();
    f.close();
}

void sd_write(const char* filename, const char* content) {
    if (!sd_ok) {
        cli_out->println("[SD] Not initialized");
        return;
    }

    File f = SD.open(filename, FILE_WRITE);
    if (!f) {
        cli_out->printf("[SD] Cannot create: %s\n", filename);
        return;
    }

    f.print(content);
    f.close();
    cli_out->printf("[SD] Wrote %d bytes to %s\n", strlen(content), filename);
}

//=============================================================================
// I2C Scanner
//=============================================================================

void i2c_scan() {
    cli_out->println("[I2C] Scanning...");
    int found = 0;
    for (int addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            cli_out->printf("  0x%02X", addr);
            // Identify known devices
            switch (addr) {
                case 0x38: cli_out->print(" (AHT20)"); break;
                case 0x59: cli_out->print(" (SGP40)"); break;
                case 0x62: cli_out->print(" (SCD41)"); break;
            }
            cli_out->println();
            found++;
        }
    }
    cli_out->printf("[I2C] Found %d device(s)\n", found);
}

//=============================================================================
// CLI Command Parser
//=============================================================================

void print_help() {
    cli_out->println();
    cli_out->println("=== RP2040 Sensor Hub CLI ===");
    cli_out->println();
    cli_out->println("Sensors:");
    cli_out->println("  init          - Initialize all sensors");
    cli_out->println("  read          - Read all sensors");
    cli_out->println("  aht           - Read AHT20 (temp/humidity)");
    cli_out->println("  sgp           - Read SGP40 (VOC)");
    cli_out->println("  scd           - Read SCD41 (CO2)");
    cli_out->println("  adc <0|1>     - Read Grove ADC channel");
    cli_out->println("  i2c           - Scan I2C bus");
    cli_out->println();
    cli_out->println("Power:");
    cli_out->println("  power <on|off> - Sensor power control");
    cli_out->println();
    cli_out->println("Buzzer:");
    cli_out->println("  beep          - Single beep");
    cli_out->println("  tone <freq> <ms> - Play tone");
    cli_out->println("  sweep <f1> <f2> <ms> - Frequency sweep");
    cli_out->println("  success       - Success melody");
    cli_out->println("  error         - Error melody");
    cli_out->println();
    cli_out->println("SD Card:");
    cli_out->println("  ls [path]     - List directory");
    cli_out->println("  cat <file>    - Read file");
    cli_out->println("  write <file> <text> - Write file");
    cli_out->println();
    cli_out->println("System:");
    cli_out->println("  status        - Show system status");
    cli_out->println("  boot          - Reboot to bootloader");
    cli_out->println("  help          - Show this help");
    cli_out->println();
}

void print_status() {
    cli_out->println();
    cli_out->println("=== System Status ===");
    cli_out->printf("Sensor Power: %s\n", sensor_power_on ? "ON" : "OFF");
    cli_out->printf("AHT20:  %s\n", aht_ok ? "OK" : "Not initialized");
    cli_out->printf("SGP40:  %s\n", sgp40_ok ? "OK" : "Not initialized");
    cli_out->printf("SCD41:  %s\n", scd4x_ok ? "OK" : "Not initialized");
    cli_out->printf("SD:     %s\n", sd_ok ? "OK" : "Not initialized");
    cli_out->printf("Uptime: %lu ms\n", millis());
    cli_out->println();
}

void process_command(char* cmd) {
    // Trim whitespace
    while (*cmd == ' ') cmd++;
    char* end = cmd + strlen(cmd) - 1;
    while (end > cmd && (*end == ' ' || *end == '\r' || *end == '\n')) {
        *end-- = '\0';
    }

    if (strlen(cmd) == 0) return;

    // Parse command and arguments
    char* args = strchr(cmd, ' ');
    if (args) {
        *args++ = '\0';
        while (*args == ' ') args++;
    }

    // Command dispatch
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_help();
    }
    else if (strcmp(cmd, "init") == 0) {
        init_sensors();
    }
    else if (strcmp(cmd, "read") == 0) {
        read_all_sensors();
    }
    else if (strcmp(cmd, "aht") == 0) {
        read_aht();
    }
    else if (strcmp(cmd, "sgp") == 0) {
        read_sgp40();
    }
    else if (strcmp(cmd, "scd") == 0) {
        read_scd4x();
    }
    else if (strcmp(cmd, "adc") == 0) {
        int ch = args ? atoi(args) : 0;
        read_adc(ch);
    }
    else if (strcmp(cmd, "i2c") == 0) {
        i2c_scan();
    }
    else if (strcmp(cmd, "power") == 0) {
        if (args && strcmp(args, "on") == 0) {
            sensor_power(true);
            cli_out->println("[POWER] Sensors ON");
        } else if (args && strcmp(args, "off") == 0) {
            sensor_power(false);
            cli_out->println("[POWER] Sensors OFF");
        } else {
            cli_out->printf("[POWER] Currently %s\n", sensor_power_on ? "ON" : "OFF");
        }
    }
    else if (strcmp(cmd, "beep") == 0) {
        buzzer_beep(1, 1000, 100, 0);
        cli_out->println("[BUZZER] Beep");
    }
    else if (strcmp(cmd, "tone") == 0) {
        if (args) {
            int freq = atoi(args);
            char* dur_str = strchr(args, ' ');
            int dur = dur_str ? atoi(dur_str) : 500;
            buzzer_tone(freq, dur);
            cli_out->printf("[BUZZER] Tone %d Hz for %d ms\n", freq, dur);
        } else {
            cli_out->println("Usage: tone <freq> <ms>");
        }
    }
    else if (strcmp(cmd, "sweep") == 0) {
        if (args) {
            int f1 = atoi(args);
            char* f2_str = strchr(args, ' ');
            int f2 = f2_str ? atoi(f2_str + 1) : f1 * 2;
            char* dur_str = f2_str ? strchr(f2_str + 1, ' ') : NULL;
            int dur = dur_str ? atoi(dur_str) : 1000;
            buzzer_sweep(f1, f2, dur);
            cli_out->printf("[BUZZER] Sweep %d-%d Hz over %d ms\n", f1, f2, dur);
        } else {
            cli_out->println("Usage: sweep <f1> <f2> <ms>");
        }
    }
    else if (strcmp(cmd, "success") == 0) {
        buzzer_success();
        cli_out->println("[BUZZER] Success melody");
    }
    else if (strcmp(cmd, "error") == 0) {
        buzzer_error();
        cli_out->println("[BUZZER] Error melody");
    }
    else if (strcmp(cmd, "ls") == 0) {
        sd_list(args ? args : "/");
    }
    else if (strcmp(cmd, "cat") == 0) {
        if (args) {
            sd_read(args);
        } else {
            cli_out->println("Usage: cat <filename>");
        }
    }
    else if (strcmp(cmd, "write") == 0) {
        if (args) {
            char* content = strchr(args, ' ');
            if (content) {
                *content++ = '\0';
                sd_write(args, content);
            } else {
                cli_out->println("Usage: write <filename> <text>");
            }
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        print_status();
    }
    else if (strcmp(cmd, "boot") == 0) {
        cli_out->println("[BOOT] Rebooting to bootloader...");
        cli_out->flush();
        delay(100);
        rp2040.rebootToBootloader();
    }
    else if (strcmp(cmd, "led") == 0) {
        if (args && strcmp(args, "on") == 0) {
            digitalWrite(LED_PIN, HIGH);
            cli_out->println("[LED] ON");
        } else if (args && strcmp(args, "off") == 0) {
            digitalWrite(LED_PIN, LOW);
            cli_out->println("[LED] OFF");
        } else {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            cli_out->printf("[LED] Toggled %s\n", digitalRead(LED_PIN) ? "ON" : "OFF");
        }
    }
    else {
        cli_out->printf("Unknown command: %s\n", cmd);
        cli_out->println("Type 'help' for available commands");
    }
}

void cli_process_char(char c) {
    if (c == '\r' || c == '\n') {
        if (cli_pos > 0) {
            cli_buffer[cli_pos] = '\0';
            cli_out->println();
            process_command(cli_buffer);
            cli_pos = 0;
            cli_out->print("> ");
        }
    }
    else if (c == '\b' || c == 127) { // Backspace
        if (cli_pos > 0) {
            cli_pos--;
            cli_out->print("\b \b");
        }
    }
    else if (cli_pos < CLI_BUFFER_SIZE - 1 && c >= 32) {
        cli_buffer[cli_pos++] = c;
        cli_out->print(c); // Echo
    }
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    // Sensor power (start OFF)
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, LOW);

    // ADC
    analogReadResolution(12);

    // USB Serial (CLI)
    Serial.begin(115200);
    delay(1000);

    // UART to ESP32
    Serial1.setTX(UART_TX_ESP32);
    Serial1.setRX(UART_RX_ESP32);
    Serial1.begin(UART_BAUD);

    // Startup message
    Serial.println();
    Serial.println("========================================");
    Serial.println("RP2040 Sensor Hub - CLI Ready");
    Serial.println("========================================");
    Serial.println("Type 'help' for commands");
    Serial.println();
    Serial.print("> ");

    // Also send to ESP32 UART
    Serial1.println();
    Serial1.println("[RP2040] Sensor Hub Ready");

    // Ready beep
    buzzer_beep(2, 1000, 50, 50);

    digitalWrite(LED_PIN, LOW);
}

void loop() {
    // Process USB Serial CLI
    while (Serial.available()) {
        cli_out = &Serial;
        cli_process_char(Serial.read());
    }

    // Process UART CLI (from ESP32)
    while (Serial1.available()) {
        cli_out = &Serial1;
        cli_process_char(Serial1.read());
    }
}
