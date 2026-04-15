#include "mvlib/telemetry.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace mvlib {
namespace {
constexpr std::size_t kTelemetryMaxPayloadBytes =
  std::max(sizeof(LogPacketHeader), sizeof(WatchTextPacketHeader)) + kTelemetryMaxTextBytes;
constexpr std::size_t kTelemetryMaxRawFrameBytes = kTelemetryMaxPayloadBytes + 1;
constexpr std::size_t kTelemetryMaxEncodedFrameBytes =
  kTelemetryMaxRawFrameBytes + (kTelemetryMaxRawFrameBytes / 254) + 2;
} // namespace

Telemetry& Telemetry::getInstance() {
  static Telemetry instance;
  return instance;
}

void Telemetry::setMinLevel(LogLevel level) {
  m_minLevel = level;
}

bool Telemetry::shouldLog(LogLevel level) const {
  if (level == LogLevel::OVERRIDE) return true;
  if (m_minLevel == LogLevel::OFF || m_minLevel == LogLevel::NONE) return false;

  // Casting to uint8_t ensures numeric comparison works for LogLevel enum
  return static_cast<uint8_t>(level) >= static_cast<uint8_t>(m_minLevel);
}

// --- High-Level Senders ---

void Telemetry::sendPose(const PosePacket& pkt) {
  // Pose is high-priority system data, so it gets OVERRIDE level
  transmit(encodeMsgAll(LogLevel::OVERRIDE, MsgType::POSE),
           reinterpret_cast<const uint8_t*>(&pkt), sizeof(PosePacket));
}

void Telemetry::sendWaypointCreated(const WaypointCreatedPacket& pkt) {
  // SubType 1 = Created
  transmit(encodeMsgAll(LogLevel::OVERRIDE, MsgType::WPOINT, 1), 
           reinterpret_cast<const uint8_t*>(&pkt), sizeof(WaypointCreatedPacket));
}

void Telemetry::sendWaypointStatus(WPId id, uint8_t subType) {
  if (subType < 2 || subType > 3) return;

  // Construct minimal packet for Reached (2) or TimedOut (3)
  WaypointStatusPacket pkt;
  pkt.timestamp = static_cast<uint16_t>(pros::millis());
  pkt.id = id;
  
  transmit(encodeMsgAll(LogLevel::OVERRIDE, MsgType::WPOINT, subType), 
           reinterpret_cast<const uint8_t*>(&pkt), sizeof(WaypointStatusPacket));
}

void Telemetry::sendWatch(WatchId id, LogLevel lvl, float val, bool tripped) {
  if (!shouldLog(lvl)) return;

  WatchPacket pkt;
  pkt.timestamp = static_cast<uint16_t>(pros::millis());
  pkt.id = id;
  pkt.value = val;
  
  // SubType 1 indicates the watch predicate was tripped (elevated)
  transmit(encodeMsgAll(lvl, MsgType::WATCH, tripped ? 1 : 0), 
           reinterpret_cast<const uint8_t*>(&pkt), sizeof(WatchPacket));
}

void Telemetry::sendWatchText(WatchId id, LogLevel lvl, const std::string& text, bool tripped) {
  if (!shouldLog(lvl)) return;

  WatchTextPacketHeader header;
  header.timestamp = static_cast<uint16_t>(pros::millis());
  header.id = id;

  const std::size_t textLen = std::min(text.size(), kTelemetryMaxTextBytes);
  const std::size_t totalPayloadSize = sizeof(WatchTextPacketHeader) + textLen;
  std::array<uint8_t, kTelemetryMaxPayloadBytes> payload{};

  std::memcpy(payload.data(), &header, sizeof(WatchTextPacketHeader));
  std::memcpy(payload.data() + sizeof(WatchTextPacketHeader), text.data(), textLen);

  // SubType 2 = textual watch sample, SubType 3 = textual watch sample with tripped predicate.
  transmit(encodeMsgAll(lvl, MsgType::WATCH, tripped ? 3 : 2), payload.data(), totalPayloadSize);
}

void Telemetry::sendRoster(uint16_t id, bool isElevated) {
  auto name = Logger::getInstance().m_getRosterNameUnlocked(id, isElevated);
  if (!name.has_value()) return;

  RosterPacket pkt;
  pkt.id = id;
  std::memset(pkt.name, 0, sizeof(pkt.name));
  std::strncpy(pkt.name, name->c_str(), sizeof(pkt.name) - 1);
  
  // SubType 1 indicates this is the secondary/elevated label for the ID
  transmit(encodeMsgAll(LogLevel::OVERRIDE, MsgType::ROSTER, isElevated ? 1 : 0), 
           reinterpret_cast<const uint8_t*>(&pkt), sizeof(RosterPacket));
}

void Telemetry::sendText(LogLevel level, const char* fmt, ...) {
  if (!shouldLog(level)) return;

  // 1. Format the string
  std::array<char, kTelemetryMaxTextBytes + 1> textBuf{};
  va_list args;
  va_start(args, fmt);
  int textLen = std::vsnprintf(textBuf.data(), textBuf.size(), fmt, args);
  va_end(args);

  if (textLen < 0) return;
  if (textLen >= static_cast<int>(textBuf.size())) {
    textLen = static_cast<int>(textBuf.size()) - 1;
  }

  // 2. Prepare the binary frame (16-bit Timestamp Header + String)
  LogPacketHeader header;
  header.timestamp = static_cast<uint16_t>(pros::millis());

  const std::size_t totalPayloadSize = sizeof(LogPacketHeader) + static_cast<std::size_t>(textLen);
  std::array<uint8_t, kTelemetryMaxPayloadBytes> payload{};

  std::memcpy(payload.data(), &header, sizeof(LogPacketHeader));
  std::memcpy(payload.data() + sizeof(LogPacketHeader), textBuf.data(), static_cast<std::size_t>(textLen));

  transmit(encodeMsgAll(level, MsgType::LOG), payload.data(), totalPayloadSize);
}

// --- Internal Engine: COBS Encoder & UART Writer ---

/**
 * Consistent Overhead Byte Stuffing (COBS)
 * Prevents 0x00 bytes in the data stream by using them as frame delimiters.
 */
void Telemetry::transmit(uint8_t header, const uint8_t* data, size_t len) {
  if (data == nullptr || len > kTelemetryMaxPayloadBytes) return;

  // Buffer for [Packed Header Byte + Payload Data]
  const std::size_t rawLen = len + 1;
  std::array<uint8_t, kTelemetryMaxRawFrameBytes> rawBuf{};
  rawBuf[0] = header;
  std::memcpy(rawBuf.data() + 1, data, len);

  /**
   * COBS encoding overhead: 
   * - 1 byte for every 254 bytes of data
   * - 1 byte for the final 0x00 delimiter
   * - 1 byte for the initial code
   */
  std::array<uint8_t, kTelemetryMaxEncodedFrameBytes> encodedBuf{};
  
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
  std::size_t totalWritten = 0;
  while (totalWritten < writePos) {
    const auto result = write(1, encodedBuf.data() + totalWritten, writePos - totalWritten);
    if (result <= 0) break;
    totalWritten += static_cast<std::size_t>(result);
  }
  m_terminalMutex.give();
}

} // namespace mvlib
