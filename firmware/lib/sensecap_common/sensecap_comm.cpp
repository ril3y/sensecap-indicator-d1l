/**
 * @file sensecap_comm.cpp
 * @brief Implementation of SenseCAP communication library
 */

#include "sensecap_comm.h"
#include <string.h>

// Static instance pointer for callback
SenseCapComm *SenseCapComm::_instance = nullptr;

// Global instance
SenseCapComm sensecapComm;

void SenseCapComm::begin(HardwareSerial &serial, uint32_t baud) {
    _instance = this;
    _userCallback = nullptr;

    serial.begin(baud);
    _packetSerial.setStream(&serial);
    _packetSerial.setPacketHandler(&SenseCapComm::internalCallback);
}

void SenseCapComm::update() {
    _packetSerial.update();
}

void SenseCapComm::onPacketReceived(void (*callback)(const uint8_t *buffer, size_t size)) {
    _userCallback = callback;
}

void SenseCapComm::internalCallback(const uint8_t *buffer, size_t size) {
    if (_instance && _instance->_userCallback) {
        _instance->_userCallback(buffer, size);
    }
}

// ============================================================================
// Low-level send functions
// ============================================================================

void SenseCapComm::sendPacket(const uint8_t *buffer, size_t size) {
    _packetSerial.send(buffer, size);
}

void SenseCapComm::sendFloat(uint8_t type, float value) {
    SensorPacket pkt;
    pkt.type = type;
    pkt.value = value;
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendUint16(uint8_t type, uint16_t value) {
    SensorPacket16 pkt;
    pkt.type = type;
    pkt.value = value;
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendUint32(uint8_t type, uint32_t value) {
    CommandPacket32 pkt;
    pkt.type = type;
    pkt.param = value;
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

// ============================================================================
// Sensor Data (RP2040 -> ESP32)
// ============================================================================

void SenseCapComm::sendTemperature(float temp) {
    sendFloat(PKT_TYPE_SENSOR_TEMP, temp);
}

void SenseCapComm::sendHumidity(float humidity) {
    sendFloat(PKT_TYPE_SENSOR_HUMIDITY, humidity);
}

void SenseCapComm::sendCO2(float co2) {
    sendFloat(PKT_TYPE_SENSOR_CO2, co2);
}

void SenseCapComm::sendTVOC(float tvoc) {
    sendFloat(PKT_TYPE_SENSOR_TVOC, tvoc);
}

void SenseCapComm::sendADC(uint8_t channel, uint16_t value) {
    uint8_t type = (channel == 0) ? PKT_TYPE_SENSOR_ADC0 : PKT_TYPE_SENSOR_ADC1;
    sendUint16(type, value);
}

void SenseCapComm::sendAllSensors(const SensorData &data) {
    sendTemperature(data.temperature);
    sendHumidity(data.humidity);
    sendCO2((float)data.co2);
    sendTVOC((float)data.tvoc_index);
    sendADC(0, data.adc0);
    sendADC(1, data.adc1);
}

// ============================================================================
// Commands (ESP32 -> RP2040)
// ============================================================================

void SenseCapComm::sendBeep(uint16_t duration_ms) {
    CommandPacket16 pkt;
    pkt.type = PKT_TYPE_CMD_BEEP;
    pkt.param = duration_ms;
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendBeepPattern(uint8_t pattern) {
    uint8_t pkt[2];
    pkt[0] = PKT_TYPE_CMD_BEEP_PATTERN;
    pkt[1] = pattern;
    sendPacket(pkt, 2);
}

void SenseCapComm::sendCollectInterval(uint32_t interval_ms) {
    sendUint32(PKT_TYPE_CMD_COLLECT_INTERVAL, interval_ms);
}

void SenseCapComm::sendPowerOn() {
    uint8_t pkt = PKT_TYPE_CMD_POWER_ON;
    sendPacket(&pkt, 1);
}

void SenseCapComm::sendShutdown() {
    uint8_t pkt = PKT_TYPE_CMD_SHUTDOWN;
    sendPacket(&pkt, 1);
}

void SenseCapComm::sendReadSensors() {
    uint8_t pkt = PKT_TYPE_CMD_READ_SENSORS;
    sendPacket(&pkt, 1);
}

// ============================================================================
// Status/Control
// ============================================================================

void SenseCapComm::sendAck(const char *message) {
    StatusPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = PKT_TYPE_ACK;
    pkt.code = 0;
    if (message) {
        strncpy(pkt.message, message, sizeof(pkt.message) - 1);
    }
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendError(uint8_t code, const char *message) {
    StatusPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = PKT_TYPE_ERROR;
    pkt.code = code;
    if (message) {
        strncpy(pkt.message, message, sizeof(pkt.message) - 1);
    }
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendHeartbeat(uint32_t uptime_ms, uint8_t status_flags) {
    HeartbeatPacket pkt;
    pkt.type = PKT_TYPE_HEARTBEAT;
    pkt.uptime_ms = uptime_ms;
    pkt.status_flags = status_flags;
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

void SenseCapComm::sendVersion() {
    VersionPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = PKT_TYPE_VERSION;
    pkt.major = FW_VERSION_MAJOR;
    pkt.minor = FW_VERSION_MINOR;
    pkt.patch = FW_VERSION_PATCH;
    strncpy(pkt.build, FW_BUILD_STRING, sizeof(pkt.build) - 1);
    sendPacket((uint8_t *)&pkt, sizeof(pkt));
}
