/**
 * @file sensecap_protocol.h
 * @brief Inter-MCU communication protocol for SenseCAP Indicator
 *
 * This header defines the packet types and structures used for
 * communication between ESP32-S3 and RP2040 over UART.
 * Protocol uses PacketSerial with COBS encoding.
 */

#ifndef SENSECAP_PROTOCOL_H
#define SENSECAP_PROTOCOL_H

#include <stdint.h>

// ============================================================================
// Packet Type Definitions
// ============================================================================

// Sensor Data Packets (RP2040 -> ESP32)
#define PKT_TYPE_SENSOR_CO2         0xB2    // CO2 in ppm (float)
#define PKT_TYPE_SENSOR_TEMP        0xB3    // Temperature in C (float)
#define PKT_TYPE_SENSOR_HUMIDITY    0xB4    // Relative humidity % (float)
#define PKT_TYPE_SENSOR_TVOC        0xB5    // TVOC index (float)
#define PKT_TYPE_SENSOR_PRESSURE    0xB6    // Pressure in hPa (float) [extension]
#define PKT_TYPE_SENSOR_ADC0        0xB7    // Grove ADC0 value (uint16)
#define PKT_TYPE_SENSOR_ADC1        0xB8    // Grove ADC1 value (uint16)

// Command Packets (ESP32 -> RP2040)
#define PKT_TYPE_CMD_COLLECT_INTERVAL   0xA0    // Set polling interval (uint32 ms)
#define PKT_TYPE_CMD_BEEP               0xA1    // Trigger buzzer (uint16 duration_ms)
#define PKT_TYPE_CMD_BEEP_PATTERN       0xA2    // Buzzer pattern (uint8 pattern_id)
#define PKT_TYPE_CMD_SHUTDOWN           0xA3    // Power off sensors
#define PKT_TYPE_CMD_POWER_ON           0xA4    // Power on sensors
#define PKT_TYPE_CMD_READ_SENSORS       0xA5    // Request immediate sensor read
#define PKT_TYPE_CMD_SD_WRITE           0xA6    // Write data to SD card
#define PKT_TYPE_CMD_SD_READ            0xA7    // Read data from SD card

// Response/Status Packets (bidirectional)
#define PKT_TYPE_ACK                    0x00    // Acknowledgment
#define PKT_TYPE_NACK                   0x01    // Negative acknowledgment
#define PKT_TYPE_STATUS                 0x02    // Status report
#define PKT_TYPE_ERROR                  0x03    // Error report
#define PKT_TYPE_HEARTBEAT              0x04    // Keep-alive heartbeat
#define PKT_TYPE_VERSION                0x05    // Firmware version info

// ============================================================================
// Packet Structures
// ============================================================================

#pragma pack(push, 1)

// Generic packet header
typedef struct {
    uint8_t type;       // Packet type (PKT_TYPE_*)
    uint8_t length;     // Payload length
} PacketHeader;

// Sensor data packet (float payload)
typedef struct {
    uint8_t type;
    float value;
} SensorPacket;

// Sensor data packet (uint16 payload)
typedef struct {
    uint8_t type;
    uint16_t value;
} SensorPacket16;

// Command packet with uint32 parameter
typedef struct {
    uint8_t type;
    uint32_t param;
} CommandPacket32;

// Command packet with uint16 parameter
typedef struct {
    uint8_t type;
    uint16_t param;
} CommandPacket16;

// Status/Error packet
typedef struct {
    uint8_t type;
    uint8_t code;
    char message[32];
} StatusPacket;

// Version packet
typedef struct {
    uint8_t type;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    char build[16];
} VersionPacket;

// Heartbeat packet
typedef struct {
    uint8_t type;
    uint32_t uptime_ms;
    uint8_t status_flags;
} HeartbeatPacket;

// All sensor data combined
typedef struct {
    float temperature;      // C
    float humidity;         // %
    uint16_t co2;           // ppm
    uint16_t tvoc_index;    // 0-500
    float pressure;         // hPa (if available)
    uint16_t adc0;          // Raw ADC value
    uint16_t adc1;          // Raw ADC value
    uint32_t timestamp;     // ms since boot
} SensorData;

#pragma pack(pop)

// ============================================================================
// Status Flags (for HeartbeatPacket)
// ============================================================================

#define STATUS_FLAG_SENSORS_OK      (1 << 0)
#define STATUS_FLAG_SD_CARD_OK      (1 << 1)
#define STATUS_FLAG_I2C_OK          (1 << 2)
#define STATUS_FLAG_POWER_ON        (1 << 3)
#define STATUS_FLAG_ERROR           (1 << 7)

// ============================================================================
// Error Codes
// ============================================================================

#define ERR_NONE                    0x00
#define ERR_SENSOR_AHT20            0x01
#define ERR_SENSOR_SGP40            0x02
#define ERR_SENSOR_SCD41            0x03
#define ERR_SD_CARD_INIT            0x10
#define ERR_SD_CARD_WRITE           0x11
#define ERR_SD_CARD_READ            0x12
#define ERR_I2C_BUS                 0x20
#define ERR_INVALID_COMMAND         0x30
#define ERR_BUFFER_OVERFLOW         0x31

// ============================================================================
// Buzzer Patterns
// ============================================================================

#define BEEP_PATTERN_SINGLE         0x00    // Single short beep
#define BEEP_PATTERN_DOUBLE         0x01    // Two short beeps
#define BEEP_PATTERN_TRIPLE         0x02    // Three short beeps
#define BEEP_PATTERN_LONG           0x03    // One long beep
#define BEEP_PATTERN_ALARM          0x04    // Continuous alarm
#define BEEP_PATTERN_SUCCESS        0x05    // Rising tone
#define BEEP_PATTERN_ERROR          0x06    // Falling tone

// ============================================================================
// Helper Macros
// ============================================================================

#define PACKET_MAX_SIZE             64
#define SENSOR_PACKET_SIZE          (sizeof(SensorPacket))
#define COMMAND_PACKET_SIZE         (sizeof(CommandPacket32))

// ============================================================================
// Firmware Version
// ============================================================================

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0

#ifdef ESP32_MAIN_MCU
#define FW_BUILD_STRING     "esp32-dev"
#endif

#ifdef RP2040_SENSOR_HUB
#define FW_BUILD_STRING     "rp2040-dev"
#endif

#endif // SENSECAP_PROTOCOL_H
