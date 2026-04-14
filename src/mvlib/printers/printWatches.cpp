#include "mvlib/core.hpp"
#include "mvlib/telemetry.hpp"
#include <inttypes.h>
#include <string>

namespace mvlib {

void Logger::printWatches() {
  unique_lock lock(m_mutex);
  if (!lock.isLocked()) return;

  uint32_t nowMs = pros::millis();

  // --- 1. Watch Roster Beacon ---
  // Periodically sync watch IDs to labels so the frontend can resolve them
  static uint32_t lastWatchRosterMs = 0;
  if (nowMs - lastWatchRosterMs > 5000) {
    for (auto& w : m_watches) {
      Telemetry::getInstance().sendRoster(w.id, w.label);
    }
    lastWatchRosterMs = nowMs;
  }

  for (auto& w : m_watches) {
    // Frequency gating
    if (!w.onChange && w.lastPrintMs != 0 && (nowMs - w.lastPrintMs) < w.intervalMs) {
      continue;
    }

    if (!w.eval) continue;

    auto [lvl, valueStr, label] = w.eval();

    // Change detection
    if (w.onChange) {
      if (w.lastValue && *w.lastValue == valueStr) continue;
      w.lastValue = valueStr;
    } else {
      w.lastPrintMs = nowMs;
    }

    // --- 2. Terminal Dispatch (Binary Hex) ---
    if (m_config.logToTerminal.load()) {
      bool sentAsBinary = false;
      
      // Optimization: If the string looks like a number, send it as a raw float packet
      if (!valueStr.empty() && (isdigit(valueStr[0]) || valueStr[0] == '-')) {
        try {
          float numericVal = std::stof(valueStr);
          WatchPacket pkt;
          pkt.timestamp = nowMs;
          pkt.id = w.id;
          pkt.level = static_cast<uint8_t>(lvl);
          pkt.value = numericVal;
          
          Telemetry::getInstance().sendWatch(pkt);
          sentAsBinary = true;
        } catch (...) {
          sentAsBinary = false;
        }
      }

      // Fallback: If not numeric (e.g. "true"), send as framed binary text
      if (!sentAsBinary) {
        Telemetry::getInstance().sendText(lvl, "[WATCH],%u,%" PRIu64 ",%s", 
                                          nowMs, w.id, valueStr.c_str());
      }
    }

    // --- 3. SD Dispatch (Human Readable) ---
    if (m_config.logToSD.load() && !m_sdLocked && m_sdFile) {
      logToSD(lvl, "[WATCH],%d,%s,%" PRIu64 ",%s,%s", 
              nowMs, m_levelToString(lvl), w.id, label.c_str(), valueStr.c_str());
    }
  }
}
} // namespace mvlib
