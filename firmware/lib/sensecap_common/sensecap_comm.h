/**
 * @file sensecap_comm.h
 * @brief Communication library for SenseCAP Indicator inter-MCU protocol
 *
 * Provides high-level functions for sending/receiving data between
 * ESP32-S3 and RP2040 using PacketSerial.
 */

#ifndef SENSECAP_COMM_H
#define SENSECAP_COMM_H

#include <Arduino.h>
#include <PacketSerial.h>
#include "sensecap_protocol.h"

class SenseCapComm {
public:
    /**
     * @brief Initialize communication
     * @param serial Hardware serial port to use
     * @param baud Baud rate (default 115200)
     */
    void begin(HardwareSerial &serial, uint32_t baud = 115200);

    /**
     * @brief Process incoming packets (call in loop)
     */
    void update();

    /**
     * @brief Set callback for received packets
     * @param callback Function to call when packet received
     */
    void onPacketReceived(void (*callback)(const uint8_t *buffer, size_t size));

    // ========================================================================
    // Sensor Data Transmission (RP2040 -> ESP32)
    // ========================================================================

    /**
     * @brief Send temperature reading
     * @param temp Temperature in Celsius
     */
    void sendTemperature(float temp);

    /**
     * @brief Send humidity reading
     * @param humidity Relative humidity percentage
     */
    void sendHumidity(float humidity);

    /**
     * @brief Send CO2 reading
     * @param co2 CO2 concentration in ppm
     */
    void sendCO2(float co2);

    /**
     * @brief Send TVOC index
     * @param tvoc TVOC index (0-500)
     */
    void sendTVOC(float tvoc);

    /**
     * @brief Send ADC reading
     * @param channel ADC channel (0 or 1)
     * @param value Raw ADC value
     */
    void sendADC(uint8_t channel, uint16_t value);

    /**
     * @brief Send all sensor data at once
     * @param data SensorData structure
     */
    void sendAllSensors(const SensorData &data);

    // ========================================================================
    // Command Transmission (ESP32 -> RP2040)
    // ========================================================================

    /**
     * @brief Request buzzer beep
     * @param duration_ms Beep duration in milliseconds
     */
    void sendBeep(uint16_t duration_ms);

    /**
     * @brief Request buzzer pattern
     * @param pattern Pattern ID (BEEP_PATTERN_*)
     */
    void sendBeepPattern(uint8_t pattern);

    /**
     * @brief Set sensor polling interval
     * @param interval_ms Interval in milliseconds
     */
    void sendCollectInterval(uint32_t interval_ms);

    /**
     * @brief Request sensor power on
     */
    void sendPowerOn();

    /**
     * @brief Request sensor power off (shutdown)
     */
    void sendShutdown();

    /**
     * @brief Request immediate sensor reading
     */
    void sendReadSensors();

    // ========================================================================
    // Status/Control
    // ========================================================================

    /**
     * @brief Send acknowledgment
     * @param message Optional message
     */
    void sendAck(const char *message = nullptr);

    /**
     * @brief Send error
     * @param code Error code
     * @param message Error message
     */
    void sendError(uint8_t code, const char *message);

    /**
     * @brief Send heartbeat
     * @param uptime_ms Uptime in milliseconds
     * @param status_flags Status flags
     */
    void sendHeartbeat(uint32_t uptime_ms, uint8_t status_flags);

    /**
     * @brief Send version info
     */
    void sendVersion();

    // ========================================================================
    // Raw packet access
    // ========================================================================

    /**
     * @brief Send raw packet
     * @param buffer Packet data
     * @param size Packet size
     */
    void sendPacket(const uint8_t *buffer, size_t size);

private:
    PacketSerial _packetSerial;
    void (*_userCallback)(const uint8_t *buffer, size_t size);

    void sendFloat(uint8_t type, float value);
    void sendUint16(uint8_t type, uint16_t value);
    void sendUint32(uint8_t type, uint32_t value);

    static void internalCallback(const uint8_t *buffer, size_t size);
    static SenseCapComm *_instance;
};

// Global instance
extern SenseCapComm sensecapComm;

#endif // SENSECAP_COMM_H
