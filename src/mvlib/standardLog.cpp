#include "mvlib/core.hpp"
#include "mvlib/telemetry.hpp"
#include <cstdarg>

namespace mvlib {

void Logger::debug(const char *fmt, ...) {
  if (!Telemetry::getInstance().shouldLog(LogLevel::DEBUG)) return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  logMessage(LogLevel::DEBUG, "%s", buffer);
}

void Logger::info(const char *fmt, ...) {
  if (!Telemetry::getInstance().shouldLog(LogLevel::INFO)) return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  logMessage(LogLevel::INFO, "%s", buffer);
}

void Logger::warn(const char *fmt, ...) {
  if (!Telemetry::getInstance().shouldLog(LogLevel::WARN)) return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  logMessage(LogLevel::WARN, "%s", buffer);
}

void Logger::error(const char *fmt, ...) {
  if (!Telemetry::getInstance().shouldLog(LogLevel::ERROR)) return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  logMessage(LogLevel::ERROR, "%s", buffer);
}

void Logger::fatal(const char *fmt, ...) {
  if (!Telemetry::getInstance().shouldLog(LogLevel::FATAL)) return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  logMessage(LogLevel::FATAL, "%s", buffer);
}
} // namespace mvlib
