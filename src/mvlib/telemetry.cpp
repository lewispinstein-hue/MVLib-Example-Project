#include "mvlib/telemetry.hpp"
#include "pros/rtos.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace mvlib {

Telemetry& Telemetry::getInstance() {
  static Telemetry instance;
  return instance;
}

void Telemetry::setMinLevel(LogLevel level) {
  m_minLevel = level;
}

bool Telemetry::shouldLog(LogLevel level) const {
  // Casting to uint8_t ensures numeric comparison works for LogLevel enum
  return static_cast<uint8_t>(level) >= static_cast<uint8_t>(m_minLevel) && 
         !(m_minLevel == LogLevel::OFF || m_minLevel == LogLevel::NONE);
}

// --- High-Level Senders ---

void Telemetry::sendPose(const PosePacket& pkt) {
  transmit(MsgType::POSE, reinterpret_cast<const uint8_t*>(&pkt), sizeof(PosePacket));
}

void Telemetry::sendWaypoint(const WaypointPacket& pkt) {
  transmit(MsgType::WPOINT, reinterpret_cast<const uint8_t*>(&pkt), sizeof(WaypointPacket));
}

void Telemetry::sendWatch(const WatchPacket& pkt) {
  if (shouldLog(static_cast<LogLevel>(pkt.level))) {
    transmit(MsgType::WATCH, reinterpret_cast<const uint8_t*>(&pkt), sizeof(WatchPacket));
  }
}

void Telemetry::sendRoster(uint64_t id, const std::string& name) {
  RosterPacket pkt;
  pkt.id = id;
  std::memset(pkt.name, 0, sizeof(pkt.name));
  std::strncpy(pkt.name, name.c_str(), sizeof(pkt.name) - 1);
  
  transmit(MsgType::ROSTER, reinterpret_cast<const uint8_t*>(&pkt), sizeof(RosterPacket));
}

void Telemetry::sendText(LogLevel level, const char* fmt, ...) {
  if (!shouldLog(level)) return;

  // 1. Format the string
  char textBuf[512];
  va_list args;
  va_start(args, fmt);
  int textLen = std::vsnprintf(textBuf, sizeof(textBuf), fmt, args);
  va_end(args);

  if (textLen < 0) return;
  if (textLen >= (int)sizeof(textBuf)) textLen = sizeof(textBuf) - 1;

  // 2. Prepare the binary frame (Header + String)
  LogPacketHeader header;
  header.timestamp = pros::millis();
  header.level = static_cast<uint8_t>(level);

  size_t totalPayloadSize = sizeof(LogPacketHeader) + textLen;
  uint8_t payload[totalPayloadSize];

  std::memcpy(payload, &header, sizeof(LogPacketHeader));
  std::memcpy(payload + sizeof(LogPacketHeader), textBuf, textLen);

  transmit(MsgType::LOG, payload, totalPayloadSize);
}

// --- Internal Engine: COBS Encoder & UART Writer ---

/**
 * Consistent Overhead Byte Stuffing (COBS)
 * Prevents 0x00 bytes in the data stream by using them as frame delimiters.
 */
void Telemetry::transmit(MsgType type, const uint8_t* data, size_t len) {
  // Buffer for [TypeByte + Data]
  size_t rawLen = len + 1;
  uint8_t rawBuf[rawLen];
  rawBuf[0] = static_cast<uint8_t>(type);
  std::memcpy(rawBuf + 1, data, len);

  /**
   * COBS encoding overhead: 
   * - 1 byte for every 254 bytes of data
   * - 1 byte for the final 0x00 delimiter
   * - 1 byte for the initial code
   */
  uint8_t encodedBuf[rawLen + (rawLen / 254) + 2];
  
  size_t writePos = 1;
  size_t codePos = 0;
  uint8_t code = 1;

  for (size_t i = 0; i < rawLen; ++i) {
    if (rawBuf[i] == 0) {
      encodedBuf[codePos] = code;
      codePos = writePos++;
      code = 1;
    } else {
      encodedBuf[writePos++] = rawBuf[i];
      code++;
      if (code == 0xFF) {
      encodedBuf[codePos] = code;
      codePos = writePos++;
      code = 1;
      }
    }
  }
  encodedBuf[codePos] = code;
  encodedBuf[writePos++] = 0x00; // Final Delimiter

  // Lock the mutex for the duration of the physical write to prevent packet collision
  m_terminalMutex.take();
  write(1, encodedBuf, writePos); 
  m_terminalMutex.give();
}
} // namespace mvlib
