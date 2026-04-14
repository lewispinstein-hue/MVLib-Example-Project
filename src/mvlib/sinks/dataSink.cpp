#include "pros/rtos.hpp"
#include "mvlib/core.hpp"
#include "mvlib/telemetry.hpp"
#include <cstdarg>

namespace mvlib {

void Logger::logMessage(const LogLevel& level, const char *fmt, ...) {
  // Check global filter first
  if (!Telemetry::getInstance().shouldLog(level)) return;

  // Format once for both sinks
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  // Terminal Sink: High-speed binary wrapping
  if (m_config.logToTerminal.load()) {
    // We use the "%s" pattern to pass the already-formatted buffer safely
    Telemetry::getInstance().sendText(level, "%s", buffer);
  }

  // SD Sink: Standard ASCII text
  if (m_config.logToSD.load() && !m_sdLocked && m_sdFile) {
    logToSD(level, "%s", buffer);
  }
}

} // namespace mvlib
