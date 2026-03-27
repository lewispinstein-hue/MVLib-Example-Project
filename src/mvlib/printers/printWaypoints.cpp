#include "mvlib/core.hpp"
#include "mvlib/logMacros.h"
#include <inttypes.h>

#ifdef MVLIB_LOGS_REDEFINED
#undef LOG_INFO

#define LOG_INFO MVLIB_LOG_INFO
#endif

namespace mvlib {
void Logger::printWaypoints() {
  unique_lock lock(m_mutex);
  if (!lock.isLocked()) return;

  uint32_t nowMs = pros::millis();
  char buffer[256];
  
  for (auto& cp : m_waypoints) {
    if (!cp.active) continue; 

    WaypointOffset off = getWaypointOffset(cp.id);
    std::optional<uint32_t> printEveryMs = cp.params.logOffsetEveryMs;

    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%d",
          off.offX, off.offY,
          off.offT.value_or(0.0), off.remainingTimeout.value_or(0));

    if (off.reached) {
      LOG_INFO("[WPOINT],%u,REACHED,%" PRIu64 ",%s,%s", nowMs, cp.id, cp.name.c_str(), buffer);
      cp.active = false; 
    } else if (off.timedOut.value_or(false)) {
      LOG_INFO("[WPOINT],%u,TIMEDOUT,%" PRIu64 ",%s,%s", nowMs, cp.id, cp.name.c_str(), buffer);
      cp.active = false;
    } else if (printEveryMs.has_value() && nowMs - cp.lastPrintMs >= printEveryMs.value()) {
      LOG_INFO("[WPOINT],%u,OFFSET,%" PRIu64 ",%s,%s", nowMs, cp.id, cp.name.c_str(), buffer);      
      cp.lastPrintMs = nowMs;
    }
  }
}
} // namespace mvlib