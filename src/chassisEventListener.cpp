#include "chassisEventListener.hpp"
#define MVLIB_USE_SIMPLES
#include "mvlib/api.hpp"
#include <sstream>

namespace c {

namespace {
pros::Mutex g_eventMutex;
std::string g_chassisEvent;
uint64_t g_eventSeq = 0;
bool g_watchRegistered = false;

std::string directionToString(lemlib::AngularDirection direction) {
  switch (direction) {
  case lemlib::AngularDirection::CW_CLOCKWISE: return "CW";
  case lemlib::AngularDirection::CCW_COUNTERCLOCKWISE: return "CCW";
  case lemlib::AngularDirection::AUTO:
  default: return "AUTO";
  }
  __builtin_unreachable();
}

std::string moveToPointParamsToString(const lemlib::MoveToPointParams& params) {
  std::ostringstream out;
  out << "{for=" << (params.forwards ? "true" : "false")
      << ", maxS=" << params.maxSpeed
      << ", minS=" << params.minSpeed
      << ", EAR=" << params.earlyExitRange
      << "}";
  return out.str();
}

std::string moveToPoseParamsToString(const lemlib::MoveToPoseParams& params) {
  std::ostringstream out;
  out << "{for=" << (params.forwards ? "true" : "false")
      << ", hD=" << params.horizontalDrift
      << ", lead=" << params.lead
      << ", maxS=" << params.maxSpeed
      << ", minS=" << params.minSpeed
      << ", EAR=" << params.earlyExitRange
      << "}";
  return out.str();
}

std::string turnToPointParamsToString(const lemlib::TurnToPointParams& params) {
  std::ostringstream out;
  out << "{for=" << (params.forwards ? "true" : "false")
      << ", dir=" << directionToString(params.direction)
      << ", maxS=" << params.maxSpeed
      << ", minS=" << params.minSpeed
      << ", EAR=" << params.earlyExitRange
      << "}";
  return out.str();
}

std::string turnToHeadingParamsToString(const lemlib::TurnToHeadingParams& params) {
  std::ostringstream out;
  out << "{dir=" << directionToString(params.direction)
      << ", maxS=" << params.maxSpeed
      << ", minS=" << params.minSpeed
      << ", EAR=" << params.earlyExitRange
      << "}";
  return out.str();
}

void ensureWatchRegistered() {
  if (g_watchRegistered) return;
  logger.watch("Chassis Event:", mvlib::LogLevel::INFO, true,
               []() {
                 g_eventMutex.take();
                 std::string snapshot = g_chassisEvent;
                 g_eventMutex.give();
                 return snapshot;
               },
               mvlib::LevelOverride<std::string>{}, "%s");
  g_watchRegistered = true;
}

void setEvent(std::string event) {
  g_eventMutex.take();
  g_chassisEvent = std::move(event);
  g_eventMutex.give();
}

std::string nextSeqString() {
  g_eventMutex.take();
  const uint64_t seq = ++g_eventSeq;
  g_eventMutex.give();
  std::ostringstream out;
  out << "#" << seq;
  return out.str();
}
} // namespace

void moveToPoint(float x, float y, int timeout,
                 lemlib::MoveToPointParams params, bool async) {
  ensureWatchRegistered();
  std::ostringstream out;
  out << nextSeqString()
      << " moveToPoint: "
      << x << ", "
      << y << ", "
      << timeout << ", "
      << moveToPointParamsToString(params) << ", "
      << (async ? "true" : "false");
  setEvent(out.str());
  chassis.moveToPoint(x, y, timeout, params, async);
}

void moveToPose(float x, float y, float theta, int timeout,
                lemlib::MoveToPoseParams params, bool async) {
  ensureWatchRegistered();
  std::ostringstream out;
  out << nextSeqString()
      << " moveToPose: "
      << x << ", "
      << y << ", "
      << theta << ", "
      << timeout << ", "
      << moveToPoseParamsToString(params) << ", "
      << (async ? "true" : "false");
  setEvent(out.str());
  chassis.moveToPose(x, y, theta, timeout, params, async);
}

void turnToHeading(float theta, int timeout,
                   lemlib::TurnToHeadingParams params, bool async) {
  ensureWatchRegistered();
  std::ostringstream out;
  out << nextSeqString()
      << " turnToHeading: "
      << theta << ", "
      << timeout << ", "
      << turnToHeadingParamsToString(params) << ", "
      << (async ? "true" : "false");
  setEvent(out.str());
  chassis.turnToHeading(theta, timeout, params, async);
}

void turnToPoint(float x, float y, int timeout,
                 lemlib::TurnToPointParams params, bool async) {
  ensureWatchRegistered();
  std::ostringstream out;
  out << nextSeqString()
      << " turnToPoint: "
      << x << ", "
      << y << ", "
      << timeout << ", "
      << turnToPointParamsToString(params) << ", "
      << (async ? "true" : "false");
  setEvent(out.str());
  chassis.turnToPoint(x, y, timeout, params, async);
}

void waitUntilDone() {
  ensureWatchRegistered();
  std::ostringstream out;
  out << nextSeqString() << " waitUntilDone";
  setEvent(out.str());
  chassis.waitUntilDone();
}
} // namespace c