#include "mvlib/core.hpp"
#include "mvlib/telemetry.hpp"
#include <inttypes.h>
#include <cmath>

namespace mvlib {

void Logger::printWaypoints() {
  unique_lock lock(m_mutex);
  if (!lock.isLocked()) return;

  uint32_t nowMs = pros::millis();

  // --- 1. Roster Beacon (Late-Joiner Support) ---
  static uint32_t lastRosterMs = 0;
  if (nowMs - lastRosterMs > 2000) {
    for (auto& wp : m_waypoints) {
      Telemetry::getInstance().sendRoster(wp.id, wp.name);
    }
    lastRosterMs = nowMs;
  }

  auto pose = m_getPose ? m_getPose() : std::nullopt;

  for (auto& wp : m_waypoints) {
    if (!wp.active) continue;

    WaypointOffset off{};
    if (pose) {
      off.offX = wp.params.tarX - pose->x;
      off.offY = wp.params.tarY - pose->y;
      off.totalOffset = std::sqrt(off.offX * off.offX + off.offY * off.offY);

      if (wp.params.tarT.has_value()) {
        double error = wp.params.tarT.value() - pose->theta;
        error = std::fmod(error + 180.0, 360.0);
        if (error < 0) error += 360.0;
        off.offT = error - 180.0;
      }

      bool linearReached = off.totalOffset <= wp.params.linearTol;
      bool angularReached = !wp.params.thetaTol.has_value() ||
                            (off.offT.has_value() && std::abs(off.offT.value()) <= wp.params.thetaTol.value());
      off.reached = (linearReached && angularReached);
    }

    if (wp.params.timeoutMs.has_value()) {
      uint32_t elapsed = nowMs - wp.startTimeMs;
      if (elapsed >= wp.params.timeoutMs.value()) {
        off.timedOut = true;
        off.remainingTimeout = 0;
      } else {
        off.remainingTimeout = wp.params.timeoutMs.value() - elapsed;
        off.timedOut = false;
      }
    }

    // Determine State for Binary Packet
    uint8_t binaryState = 0; // OFFSET
    bool shouldTrigger = false;
    const char* statusStr = "OFFSET";

    if (off.timedOut.value_or(false)) {
      binaryState = 2; // TIMEDOUT
      statusStr = "TIMEDOUT";
      shouldTrigger = true;
      wp.active = false;
    } else if (off.reached) {
      binaryState = 1; // REACHED
      statusStr = "REACHED";
      if (!wp.prevReached || !wp.params.retriggerable) {
        shouldTrigger = true;
        wp.prevReached = true;
        wp.active = wp.params.retriggerable;
      }
    } else {
      wp.prevReached = false;
      // Normal periodic offset log
      if (wp.params.logOffsetEveryMs.has_value() && 
          (nowMs - wp.lastPrintMs >= wp.params.logOffsetEveryMs.value())) {
        shouldTrigger = true;
        wp.lastPrintMs = nowMs;
      }
    }

    if (!shouldTrigger) continue;

    // --- 2. Binary Dispatch (Terminal) ---
    if (m_config.logToTerminal.load()) {
      WaypointPacket pkt;
      pkt.timestamp = nowMs;
      pkt.id = wp.id;
      pkt.state = binaryState;
      pkt.offX = (float)off.offX;
      pkt.offY = (float)off.offY;
      pkt.offT = (float)off.offT.value_or(NAN);
      pkt.totalOff = (float)off.totalOffset;
      pkt.remainingTimeout = (int32_t)off.remainingTimeout.value_or(-1);
      Telemetry::getInstance().sendWaypoint(pkt);
    }

    // --- 3. ASCII Dispatch (SD Card) ---
    if (m_config.logToSD.load() && !m_sdLocked && m_sdFile) {
      logToSD(LogLevel::OVERRIDE, "[WPOINT],%u,%s,%" PRIu64 ",%s,%.2f,%.2f,%s,%s",
              nowMs, statusStr, wp.id, wp.name.c_str(), 
              off.offX, off.offY,
              off.offT.has_value() ? std::to_string(off.offT.value()).c_str() : "NA",
              off.remainingTimeout.has_value() ? std::to_string(off.remainingTimeout.value()).c_str() : "NA");
    }
  }
}
} // namespace mvlib
