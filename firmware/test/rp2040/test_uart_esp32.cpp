/**
 * @file test_uart_esp32.cpp
 * @brief RP2040 <-> ESP32 UART Communication Test
 *
 * Tests bidirectional UART communication with the ESP32.
 * Uses PacketSerial with COBS encoding for reliable framing.
 *
 * RP2040 pins: TX=GP16, RX=GP17
 * ESP32 pins: TX=GPIO43, RX=GPIO44
 */

#include <Arduino.h>
#include <PacketSerial.h>

// UART pins to ESP32
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_BAUD   115200

// Use Serial1 for ESP32 communication
PacketSerial_<COBS, 0, 256> pktSerial;

// Statistics
static uint32_t packets_sent = 0;
static uint32_t packets_received = 0;
static uint32_t packets_errors = 0;
static uint32_t last_send_time = 0;
static uint32_t last_rtt = 0;

// Test packet structure (must match ESP32)
struct TestPacket {
    uint8_t type;       // 0=ping, 1=pong, 2=data
    uint32_t seq;       // Sequence number
    uint32_t timestamp; // Sender timestamp
    uint8_t data[8];    // Payload
};

#define PKT_PING 0
#define PKT_PONG 1
#define PKT_DATA 2

// Current sequence number
static uint32_t seq_num = 0;

// LED for activity indication
#define LED_PIN 25

//=============================================================================
// Packet Handler
//=============================================================================

void onPacketReceived(const uint8_t* buffer, size_t size) {
    digitalWrite(LED_PIN, HIGH);

    if (size < sizeof(TestPacket)) {
        packets_errors++;
        Serial.printf("[ERROR] Packet too small: %d bytes\n", size);
        digitalWrite(LED_PIN, LOW);
        return;
    }

    TestPacket* pkt = (TestPacket*)buffer;
    packets_received++;

    switch (pkt->type) {
        case PKT_PING:
            // Respond with pong
            {
                TestPacket resp;
                resp.type = PKT_PONG;
                resp.seq = pkt->seq;
                resp.timestamp = pkt->timestamp;
                memcpy(resp.data, pkt->data, 8);
                pktSerial.send((uint8_t*)&resp, sizeof(resp));
                Serial.printf("[RX] PING seq=%lu, sending PONG\n", pkt->seq);
            }
            break;

        case PKT_PONG:
            // Calculate round-trip time
            last_rtt = millis() - pkt->timestamp;
            Serial.printf("[RX] PONG seq=%lu RTT=%lu ms\n", pkt->seq, last_rtt);
            break;

        case PKT_DATA:
            Serial.printf("[RX] DATA seq=%lu: ", pkt->seq);
            for (int i = 0; i < 8; i++) {
                Serial.printf("%02X ", pkt->data[i]);
            }
            Serial.println();
            break;

        default:
            packets_errors++;
            Serial.printf("[ERROR] Unknown packet type: %d\n", pkt->type);
            break;
    }

    digitalWrite(LED_PIN, LOW);
}

//=============================================================================
// Send Functions
//=============================================================================

void sendPing() {
    TestPacket pkt;
    pkt.type = PKT_PING;
    pkt.seq = seq_num++;
    pkt.timestamp = millis();
    for (int i = 0; i < 8; i++) {
        pkt.data[i] = i;
    }

    pktSerial.send((uint8_t*)&pkt, sizeof(pkt));
    packets_sent++;
    last_send_time = millis();
    Serial.printf("[TX] PING seq=%lu\n", pkt.seq);
}

void sendData(uint8_t* data, size_t len) {
    TestPacket pkt;
    pkt.type = PKT_DATA;
    pkt.seq = seq_num++;
    pkt.timestamp = millis();
    memset(pkt.data, 0, 8);
    if (len > 8) len = 8;
    memcpy(pkt.data, data, len);

    pktSerial.send((uint8_t*)&pkt, sizeof(pkt));
    packets_sent++;
}

//=============================================================================
// Main
//=============================================================================

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(1000);
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("RP2040 <-> ESP32 UART Test");
    Serial.println("========================================\n");

    // Initialize Serial1 for ESP32 communication
    // RP2040 Arduino core uses SerialPIO or hardware UART
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.begin(UART_BAUD);

    // Setup PacketSerial
    pktSerial.setStream(&Serial1);
    pktSerial.setPacketHandler(&onPacketReceived);

    Serial.printf("UART initialized: TX=GP%d, RX=GP%d, Baud=%d\n",
                  UART_TX_PIN, UART_RX_PIN, UART_BAUD);
    Serial.println("\nCommands:");
    Serial.println("  p - Send PING");
    Serial.println("  t - Run 100-packet test");
    Serial.println("  s - Show statistics");
    Serial.println("  r - Reset statistics");
    Serial.println("  b - Reboot to bootloader (for firmware update)");
    Serial.println("\nWaiting for ESP32...\n");

    // Blink LED to show ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void loop() {
    // Process incoming packets
    pktSerial.update();

    // Handle serial commands
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'p':
            case 'P':
                sendPing();
                break;

            case 't':
            case 'T':
                Serial.println("\n[TEST] Sending 100 packets...");
                {
                    uint32_t start = millis();
                    uint32_t start_rx = packets_received;
                    for (int i = 0; i < 100; i++) {
                        sendPing();
                        // Wait for response with timeout
                        uint32_t wait_start = millis();
                        while (millis() - wait_start < 50) {
                            pktSerial.update();
                            delay(1);
                        }
                    }
                    uint32_t elapsed = millis() - start;
                    uint32_t received = packets_received - start_rx;
                    Serial.printf("[TEST] Complete: %lu/%d received (%.1f%%) in %lu ms\n",
                                  received, 100, received * 100.0 / 100, elapsed);
                }
                break;

            case 's':
            case 'S':
                Serial.println("\n--- Statistics ---");
                Serial.printf("Packets sent:     %lu\n", packets_sent);
                Serial.printf("Packets received: %lu\n", packets_received);
                Serial.printf("Packet errors:    %lu\n", packets_errors);
                Serial.printf("Last RTT:         %lu ms\n", last_rtt);
                Serial.println("------------------\n");
                break;

            case 'r':
            case 'R':
                packets_sent = 0;
                packets_received = 0;
                packets_errors = 0;
                last_rtt = 0;
                seq_num = 0;
                Serial.println("[OK] Statistics reset");
                break;

            case 'b':
            case 'B':
                Serial.println("[BOOT] Rebooting to bootloader...");
                Serial.flush();
                delay(100);
                rp2040.rebootToBootloader();
                break;
        }
    }

    delay(1);
}
