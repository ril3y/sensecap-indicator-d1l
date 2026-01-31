/**
 * @file rp2040_d1l.cpp
 * @brief RP2040 Firmware for SenseCAP Indicator D1L
 *
 * Handles SD card storage and buzzer via UART commands from ESP32.
 * D1L has NO onboard sensors - only SD, buzzer, and Grove expansion.
 *
 * UART Protocol (text-based, works from ESP32 or USB terminal):
 *   Commands are newline-terminated, responses prefixed with OK/ERR
 *
 * Peripherals:
 * - SD Card: SPI1 (GP10-13) - Store node database, message logs
 * - Buzzer: GP19 PWM - Notification sounds
 * - Grove I2C: GP20/21 - External sensor expansion
 * - Grove ADC: GP26/27 - Analog inputs
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

//=============================================================================
// Pin Definitions (D1L)
//=============================================================================

// UART to ESP32
#define UART_TX_ESP32   16
#define UART_RX_ESP32   17
#define UART_BAUD       115200

// SPI1 (SD card)
#define SD_SCK          10
#define SD_MOSI         11
#define SD_MISO         12
#define SD_CS           13
#define SD_DET          7   // Card detect (active low when card inserted)

// I2C (Grove expansion)
#define I2C_SDA         20
#define I2C_SCL         21

// Other
#define BUZZER_PIN      19
#define GROVE_ADC0      26
#define GROVE_ADC1      27
#define LED_PIN         25

//=============================================================================
// Global State
//=============================================================================

bool sd_ok = false;
int buzzer_volume = 100;  // 0-100%, controls PWM duty cycle
bool sd_card_last_state = false;  // Track SD card insert/remove

// Note frequencies (octave 4)
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_REST 0

//=============================================================================
// EEPROM Settings
//=============================================================================

#define EEPROM_SIZE     64
#define EEPROM_MAGIC    0xD1  // Magic byte to detect valid config
#define ADDR_MAGIC      0
#define ADDR_VOLUME     1

void settings_load() {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(ADDR_MAGIC) == EEPROM_MAGIC) {
        buzzer_volume = EEPROM.read(ADDR_VOLUME);
        if (buzzer_volume > 100) buzzer_volume = 100;
    }
}

void settings_save() {
    EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.write(ADDR_VOLUME, buzzer_volume);
    EEPROM.commit();
}

// CLI buffer
#define CLI_BUF_SIZE 256
char cli_buf[CLI_BUF_SIZE];
int cli_pos = 0;
Stream* out = &Serial;  // Current output stream

//=============================================================================
// Buzzer with Volume Control
//=============================================================================

// Custom tone with volume (duty cycle) control
void buzz_tone(int freq, int ms) {
    if (freq <= 0 || buzzer_volume == 0) {
        noTone(BUZZER_PIN);
        return;
    }

    // Use PWM for volume control
    // duty cycle: 0-255, where 128 = 50% = loudest
    // Scale volume: 100% -> 128 (50% duty), 50% -> 64 (25% duty), etc.
    int duty = (128 * buzzer_volume) / 100;

    analogWriteFreq(freq);
    analogWrite(BUZZER_PIN, duty);
    delay(ms);
    analogWrite(BUZZER_PIN, 0);
}

void buzz_beep(int n, int freq, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        buzz_tone(freq, on_ms);
        delay(off_ms);
    }
}

void buzz_off() {
    analogWrite(BUZZER_PIN, 0);
    noTone(BUZZER_PIN);
}

// Parse note name to frequency (e.g., "C4", "A#5", "Gb4")
int note_to_freq(const char* note) {
    if (!note || !*note) return 0;

    // Base frequencies for octave 4
    int base_freqs[] = {262, 294, 330, 349, 392, 440, 494}; // C D E F G A B

    char n = toupper(note[0]);
    if (n == 'R' || n == '-') return 0;  // Rest

    int idx = -1;
    if (n >= 'A' && n <= 'G') {
        int map[] = {5, 6, 0, 1, 2, 3, 4};  // A=5, B=6, C=0, D=1, E=2, F=3, G=4
        idx = map[n - 'A'];
    }
    if (idx < 0) return 0;

    int freq = base_freqs[idx];
    int pos = 1;

    // Check for sharp/flat
    if (note[pos] == '#' || note[pos] == 's') {
        freq = freq * 1059 / 1000;  // ~semitone up
        pos++;
    } else if (note[pos] == 'b') {
        freq = freq * 944 / 1000;   // ~semitone down
        pos++;
    }

    // Check for octave (default 4)
    int octave = 4;
    if (note[pos] >= '1' && note[pos] <= '8') {
        octave = note[pos] - '0';
    }

    // Adjust for octave
    while (octave < 4) { freq /= 2; octave++; }
    while (octave > 4) { freq *= 2; octave--; }

    return freq;
}

// Play a sequence of notes: "C4:100 D4:100 E4:200" (note:duration_ms)
void play_notes(const char* sequence) {
    char buf[128];
    strncpy(buf, sequence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char* token = strtok(buf, " ");
    while (token) {
        char* colon = strchr(token, ':');
        int duration = 100;  // Default 100ms
        if (colon) {
            *colon = '\0';
            duration = atoi(colon + 1);
        }

        int freq = note_to_freq(token);
        if (freq > 0) {
            buzz_tone(freq, duration);
        } else {
            delay(duration);  // Rest
        }
        delay(10);  // Small gap between notes

        token = strtok(NULL, " ");
    }
}

void buzz_melody(const char* name) {
    // === Keyboard/UI Feedback ===
    if (strcmp(name, "tick") == 0) {
        // Key press tick - very short click
        buzz_tone(4000, 2);
    }
    else if (strcmp(name, "click") == 0) {
        // Button click - slightly longer
        buzz_tone(2500, 8);
    }
    else if (strcmp(name, "key") == 0) {
        // Keyboard key - soft tick
        buzz_tone(3000, 5);
    }
    else if (strcmp(name, "enter") == 0) {
        // Enter/submit key
        buzz_tone(1500, 15); delay(20);
        buzz_tone(2000, 25);
    }
    else if (strcmp(name, "back") == 0) {
        // Backspace/cancel
        buzz_tone(1000, 20);
    }
    // === Short Notifications ===
    else if (strcmp(name, "pip") == 0) {
        // Single short pip
        buzz_tone(2000, 20);
    }
    else if (strcmp(name, "blip") == 0) {
        // Quick blip
        buzz_tone(1800, 30);
    }
    else if (strcmp(name, "ack") == 0) {
        // Acknowledgment
        buzz_tone(1200, 40);
    }
    else if (strcmp(name, "ok") == 0) {
        // Success - two quick rising tones
        buzz_tone(1000, 40); delay(50);
        buzz_tone(1500, 60);
    }
    else if (strcmp(name, "no") == 0) {
        // Reject/invalid - low buzz
        buzz_tone(300, 80);
    }
    // === Message Notifications ===
    else if (strcmp(name, "msg") == 0) {
        // New message - ascending
        buzz_tone(1200, 80); delay(100);
        buzz_tone(1600, 80); delay(100);
        buzz_tone(2000, 120);
    }
    else if (strcmp(name, "msg2") == 0) {
        // Alt message - two tone
        buzz_tone(1800, 60); delay(80);
        buzz_tone(2200, 100);
    }
    else if (strcmp(name, "dm") == 0) {
        // Direct message - softer
        buzz_tone(1500, 50); delay(70);
        buzz_tone(1800, 70);
    }
    else if (strcmp(name, "sent") == 0) {
        // Message sent confirmation
        buzz_tone(1600, 30); delay(40);
        buzz_tone(2000, 50);
    }
    // === Status Sounds ===
    else if (strcmp(name, "conn") == 0) {
        // Connected
        buzz_tone(800, 50); delay(60);
        buzz_tone(1200, 50); delay(60);
        buzz_tone(1600, 80);
    }
    else if (strcmp(name, "disc") == 0) {
        // Disconnected
        buzz_tone(1200, 50); delay(60);
        buzz_tone(800, 50); delay(60);
        buzz_tone(500, 100);
    }
    else if (strcmp(name, "boot") == 0) {
        // Boot complete
        buzz_tone(800, 80); delay(100);
        buzz_tone(1000, 80); delay(100);
        buzz_tone(1200, 150);
    }
    else if (strcmp(name, "low") == 0) {
        // Low battery warning
        buzz_tone(600, 200); delay(150);
        buzz_tone(600, 200);
    }
    // === SD Card Events ===
    else if (strcmp(name, "sdin") == 0) {
        // SD card inserted - pleasant rising tone
        buzz_tone(800, 50); delay(40);
        buzz_tone(1200, 50); delay(40);
        buzz_tone(1600, 80);
    }
    else if (strcmp(name, "sdout") == 0) {
        // SD card removed - falling tone
        buzz_tone(1200, 50); delay(40);
        buzz_tone(800, 50); delay(40);
        buzz_tone(500, 80);
    }
    // === Errors/Alerts ===
    else if (strcmp(name, "err") == 0) {
        // Error
        buzz_tone(400, 150); delay(200);
        buzz_tone(300, 300);
    }
    else if (strcmp(name, "warn") == 0) {
        // Warning - two beeps
        buzz_tone(1000, 100); delay(120);
        buzz_tone(1000, 100);
    }
    else if (strcmp(name, "alert") == 0) {
        // Urgent alert
        for (int i = 0; i < 3; i++) {
            buzz_tone(2000, 100); delay(150);
            buzz_tone(1500, 100); delay(150);
        }
    }
    else if (strcmp(name, "sos") == 0) {
        // SOS pattern
        for (int i = 0; i < 3; i++) { buzz_tone(1500, 100); delay(150); }
        delay(200);
        for (int i = 0; i < 3; i++) { buzz_tone(1500, 250); delay(300); }
        delay(200);
        for (int i = 0; i < 3; i++) { buzz_tone(1500, 100); delay(150); }
    }
    else {
        out->println("ERR unknown melody");
        return;
    }
    out->printf("OK %s\n", name);
}

//=============================================================================
// SD Card
//=============================================================================

bool sd_card_inserted() {
    return digitalRead(SD_DET) == LOW;  // Active low
}

bool sd_init() {
    if (!sd_card_inserted()) {
        sd_ok = false;
        return false;
    }
    SPI1.setRX(SD_MISO);
    SPI1.setTX(SD_MOSI);
    SPI1.setSCK(SD_SCK);
    sd_ok = SD.begin(SD_CS, SPI1);
    return sd_ok;
}

void sd_ls(const char* path) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    File dir = SD.open(path);
    if (!dir) { out->printf("ERR cannot open %s\n", path); return; }
    if (!dir.isDirectory()) { out->println("ERR not a directory"); dir.close(); return; }

    out->printf("OK ls %s\n", path);
    while (File f = dir.openNextFile()) {
        out->printf("%c %8lu %s\n",
            f.isDirectory() ? 'd' : '-',
            f.size(),
            f.name());
        f.close();
    }
    dir.close();
}

void sd_read(const char* path) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    File f = SD.open(path, FILE_READ);
    if (!f) { out->printf("ERR cannot open %s\n", path); return; }

    size_t sz = f.size();
    out->printf("OK %lu bytes\n", sz);
    while (f.available()) {
        out->write(f.read());
    }
    out->println();
    f.close();
}

void sd_write(const char* path, const char* data) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    File f = SD.open(path, FILE_WRITE);
    if (!f) { out->printf("ERR cannot create %s\n", path); return; }

    size_t written = f.print(data);
    f.close();
    out->printf("OK wrote %lu bytes\n", written);
}

void sd_append(const char* path, const char* data) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    File f = SD.open(path, FILE_WRITE);
    if (!f) { out->printf("ERR cannot open %s\n", path); return; }

    f.seek(f.size());  // Seek to end
    size_t written = f.print(data);
    f.close();
    out->printf("OK appended %lu bytes\n", written);
}

void sd_rm(const char* path) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    if (SD.remove(path)) {
        out->printf("OK removed %s\n", path);
    } else {
        out->printf("ERR cannot remove %s\n", path);
    }
}

void sd_mkdir(const char* path) {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    if (SD.mkdir(path)) {
        out->printf("OK created %s\n", path);
    } else {
        out->printf("ERR cannot create %s\n", path);
    }
}

void sd_info() {
    if (!sd_ok) { out->println("ERR sd not ready"); return; }

    // Note: SD library doesn't expose card size easily on RP2040
    out->println("OK sd info");
    out->println("SD card mounted");
}

//=============================================================================
// Grove I2C
//=============================================================================

void i2c_scan() {
    out->println("OK i2c scan");
    int found = 0;
    for (int addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            out->printf("0x%02X\n", addr);
            found++;
        }
    }
    out->printf("found %d\n", found);
}

void i2c_read(int addr, int reg, int len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        out->println("ERR i2c write failed");
        return;
    }

    int received = Wire.requestFrom(addr, len);
    if (received == 0) {
        out->println("ERR i2c read failed");
        return;
    }

    out->printf("OK %d bytes:", received);
    while (Wire.available()) {
        out->printf(" %02X", Wire.read());
    }
    out->println();
}

void i2c_write(int addr, int reg, uint8_t* data, int len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    if (Wire.endTransmission() == 0) {
        out->printf("OK wrote %d bytes\n", len);
    } else {
        out->println("ERR i2c write failed");
    }
}

//=============================================================================
// ADC
//=============================================================================

void adc_read(int ch) {
    int pin = (ch == 0) ? GROVE_ADC0 : GROVE_ADC1;
    int raw = analogRead(pin);
    float volts = raw * 3.3f / 4095.0f;
    out->printf("OK adc%d %d %.3fV\n", ch, raw, volts);
}

//=============================================================================
// Command Parser
//=============================================================================

void cmd_help() {
    out->println("OK help");
    out->println("=== RP2040 D1L Commands ===");
    out->println();
    out->println("Buzzer:");
    out->println("  beep [n] [freq] [ms]  - Beep n times");
    out->println("  tone <freq> <ms>      - Play tone");
    out->println("  melody <name>         - Play sound (see list below)");
    out->println("  note <N> [ms]         - Play note (e.g., C4, A#5, Gb3)");
    out->println("  play <notes>          - Play sequence: C4:100 D4:100 E4:200");
    out->println("  vol [0-100]           - Get/set volume %");
    out->println("  quiet                 - Stop sound");
    out->println("  Melodies: tick,click,key,enter,back (UI feedback)");
    out->println("            pip,blip,ack,ok,no (short)");
    out->println("            msg,msg2,dm,sent (messages)");
    out->println("            conn,disc,boot,low,sdin,sdout (status)");
    out->println("            err,warn,alert,sos (alerts)");
    out->println();
    out->println("SD Card:");
    out->println("  sd init               - Initialize SD");
    out->println("  sd info               - Card info");
    out->println("  ls [path]             - List directory");
    out->println("  cat <file>            - Read file");
    out->println("  write <file> <data>   - Write file");
    out->println("  append <file> <data>  - Append to file");
    out->println("  rm <file>             - Delete file");
    out->println("  mkdir <path>          - Create directory");
    out->println();
    out->println("Grove I2C:");
    out->println("  i2c scan              - Scan bus");
    out->println("  i2c read <addr> <reg> <len>");
    out->println("  i2c write <addr> <reg> <hex...>");
    out->println();
    out->println("ADC:");
    out->println("  adc <0|1>             - Read Grove ADC");
    out->println();
    out->println("System:");
    out->println("  status                - System status");
    out->println("  boot                  - Reboot to UF2 bootloader");
    out->println("  led <on|off|toggle>   - Control LED");
    out->println();
}

void process_cmd(char* cmd) {
    // Trim
    while (*cmd == ' ') cmd++;
    char* end = cmd + strlen(cmd) - 1;
    while (end > cmd && (*end == ' ' || *end == '\r' || *end == '\n')) *end-- = '\0';
    if (!*cmd) return;

    // Parse command and args
    char* arg1 = strchr(cmd, ' ');
    if (arg1) { *arg1++ = '\0'; while (*arg1 == ' ') arg1++; }

    char* arg2 = arg1 ? strchr(arg1, ' ') : NULL;
    if (arg2) { *arg2++ = '\0'; while (*arg2 == ' ') arg2++; }

    char* arg3 = arg2 ? strchr(arg2, ' ') : NULL;
    if (arg3) { *arg3++ = '\0'; while (*arg3 == ' ') arg3++; }

    // Dispatch
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    }
    // Buzzer
    else if (strcmp(cmd, "beep") == 0) {
        int n = arg1 ? atoi(arg1) : 1;
        int freq = arg2 ? atoi(arg2) : 1000;
        int ms = arg3 ? atoi(arg3) : 100;
        buzz_beep(n, freq, ms, 50);
        out->println("OK beep");
    }
    else if (strcmp(cmd, "tone") == 0) {
        if (!arg1 || !arg2) { out->println("ERR usage: tone <freq> <ms>"); return; }
        buzz_tone(atoi(arg1), atoi(arg2));
        out->println("OK tone");
    }
    else if (strcmp(cmd, "melody") == 0) {
        if (!arg1) { out->println("ERR usage: melody <name>"); return; }
        buzz_melody(arg1);
    }
    else if (strcmp(cmd, "quiet") == 0) {
        buzz_off();
        out->println("OK quiet");
    }
    else if (strcmp(cmd, "note") == 0) {
        if (!arg1) { out->println("ERR usage: note <N> [ms] (e.g., note C4 200)"); return; }
        int freq = note_to_freq(arg1);
        int ms = arg2 ? atoi(arg2) : 100;
        if (freq > 0) {
            buzz_tone(freq, ms);
            out->printf("OK note %s %dHz %dms\n", arg1, freq, ms);
        } else {
            delay(ms);  // Rest
            out->printf("OK rest %dms\n", ms);
        }
    }
    else if (strcmp(cmd, "play") == 0) {
        if (!arg1) { out->println("ERR usage: play <notes> (e.g., play C4:100 D4:100 E4:200)"); return; }
        // Reconstruct the full sequence (arg1 + arg2 + arg3 were split)
        char sequence[128];
        strncpy(sequence, arg1, sizeof(sequence)-1);
        sequence[sizeof(sequence)-1] = '\0';
        // The original args were split by spaces, need to find rest from original buffer
        // Just use arg1 position to end of line
        char* rest = arg1;
        while (*rest) rest++;  // Find end of arg1
        rest++;  // Skip null
        // Rebuild from the CLI buffer position
        if (arg2) {
            strncat(sequence, " ", sizeof(sequence)-strlen(sequence)-1);
            strncat(sequence, arg2, sizeof(sequence)-strlen(sequence)-1);
        }
        if (arg3) {
            strncat(sequence, " ", sizeof(sequence)-strlen(sequence)-1);
            strncat(sequence, arg3, sizeof(sequence)-strlen(sequence)-1);
        }
        play_notes(sequence);
        out->println("OK play");
    }
    else if (strcmp(cmd, "vol") == 0) {
        if (arg1) {
            int v = atoi(arg1);
            if (v < 0) v = 0;
            if (v > 100) v = 100;
            buzzer_volume = v;
            settings_save();  // Persist to flash
        }
        out->printf("OK vol %d\n", buzzer_volume);
    }
    // SD Card
    else if (strcmp(cmd, "sd") == 0) {
        if (!arg1 || strcmp(arg1, "init") == 0) {
            out->printf("%s sd init\n", sd_init() ? "OK" : "ERR");
        } else if (strcmp(arg1, "info") == 0) {
            sd_info();
        }
    }
    else if (strcmp(cmd, "ls") == 0) {
        sd_ls(arg1 ? arg1 : "/");
    }
    else if (strcmp(cmd, "cat") == 0) {
        if (!arg1) { out->println("ERR usage: cat <file>"); return; }
        sd_read(arg1);
    }
    else if (strcmp(cmd, "write") == 0) {
        if (!arg1 || !arg2) { out->println("ERR usage: write <file> <data>"); return; }
        sd_write(arg1, arg2);
    }
    else if (strcmp(cmd, "append") == 0) {
        if (!arg1 || !arg2) { out->println("ERR usage: append <file> <data>"); return; }
        sd_append(arg1, arg2);
    }
    else if (strcmp(cmd, "rm") == 0) {
        if (!arg1) { out->println("ERR usage: rm <file>"); return; }
        sd_rm(arg1);
    }
    else if (strcmp(cmd, "mkdir") == 0) {
        if (!arg1) { out->println("ERR usage: mkdir <path>"); return; }
        sd_mkdir(arg1);
    }
    // I2C
    else if (strcmp(cmd, "i2c") == 0) {
        if (!arg1 || strcmp(arg1, "scan") == 0) {
            i2c_scan();
        }
        else if (strcmp(arg1, "read") == 0) {
            if (!arg2 || !arg3) { out->println("ERR usage: i2c read <addr> <reg> <len>"); return; }
            char* arg4 = strchr(arg3, ' ');
            int len = arg4 ? atoi(arg4) : 1;
            i2c_read(strtol(arg2, NULL, 0), strtol(arg3, NULL, 0), len);
        }
        else if (strcmp(arg1, "write") == 0) {
            if (!arg2 || !arg3) { out->println("ERR usage: i2c write <addr> <reg> <bytes>"); return; }
            int addr = strtol(arg2, NULL, 0);
            int reg = strtol(arg3, NULL, 0);
            // Parse remaining bytes
            uint8_t data[16];
            int n = 0;
            char* p = strchr(arg3, ' ');
            while (p && n < 16) {
                while (*p == ' ') p++;
                if (!*p) break;
                data[n++] = strtol(p, &p, 16);
            }
            i2c_write(addr, reg, data, n);
        }
    }
    // ADC
    else if (strcmp(cmd, "adc") == 0) {
        int ch = arg1 ? atoi(arg1) : 0;
        adc_read(ch);
    }
    // System
    else if (strcmp(cmd, "status") == 0) {
        out->println("OK status");
        out->printf("sd_card: %s\n", sd_card_inserted() ? "inserted" : "not inserted");
        out->printf("sd: %s\n", sd_ok ? "mounted" : "not mounted");
        out->printf("volume: %d%%\n", buzzer_volume);
        out->printf("uptime: %lu ms\n", millis());
    }
    else if (strcmp(cmd, "boot") == 0) {
        out->println("OK rebooting to bootloader");
        out->flush();
        delay(100);
        rp2040.rebootToBootloader();
    }
    else if (strcmp(cmd, "led") == 0) {
        if (arg1 && strcmp(arg1, "on") == 0) digitalWrite(LED_PIN, HIGH);
        else if (arg1 && strcmp(arg1, "off") == 0) digitalWrite(LED_PIN, LOW);
        else digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        out->printf("OK led %s\n", digitalRead(LED_PIN) ? "on" : "off");
    }
    else {
        out->printf("ERR unknown: %s\n", cmd);
    }
}

void cli_char(char c) {
    if (c == '\r' || c == '\n') {
        if (cli_pos > 0) {
            cli_buf[cli_pos] = '\0';
            process_cmd(cli_buf);
            cli_pos = 0;
        }
    }
    else if (c == '\b' || c == 127) {
        if (cli_pos > 0) cli_pos--;
    }
    else if (cli_pos < CLI_BUF_SIZE - 1 && c >= 32) {
        cli_buf[cli_pos++] = c;
    }
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    // GPIO
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SD_DET, INPUT_PULLUP);  // SD card detect
    analogReadResolution(12);

    // Load saved settings from EEPROM
    settings_load();

    // I2C for Grove
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();

    // USB Serial
    Serial.begin(115200);
    delay(500);

    // UART to ESP32
    Serial1.setTX(UART_TX_ESP32);
    Serial1.setRX(UART_RX_ESP32);
    Serial1.begin(UART_BAUD);

    // Try SD card
    sd_card_last_state = sd_card_inserted();  // Initialize state for detection
    sd_init();

    // Boot notification
    Serial.println("OK RP2040 D1L ready");
    Serial1.println("OK RP2040 D1L ready");
    buzz_melody("boot");

    digitalWrite(LED_PIN, LOW);
}

void loop() {
    // USB CLI
    while (Serial.available()) {
        out = &Serial;
        cli_char(Serial.read());
    }

    // ESP32 UART CLI
    while (Serial1.available()) {
        out = &Serial1;
        cli_char(Serial1.read());
    }

    // SD card insert/remove detection
    static unsigned long last_sd_check = 0;
    if (millis() - last_sd_check > 200) {  // Check every 200ms
        last_sd_check = millis();
        bool inserted = sd_card_inserted();
        if (inserted != sd_card_last_state) {
            sd_card_last_state = inserted;
            if (inserted) {
                Serial.println("SD card inserted");
                Serial1.println("SD card inserted");
                buzz_melody("sdin");
                // Try to mount the card
                if (sd_init()) {
                    Serial.println("SD card mounted");
                    Serial1.println("SD card mounted");
                }
            } else {
                Serial.println("SD card removed");
                Serial1.println("SD card removed");
                sd_ok = false;
                buzz_melody("sdout");
            }
        }
    }
}
