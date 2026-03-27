#include "pros/rtos.hpp"
#include "mvlib/core.hpp"
#include "mvlib/logMacros.h"
#include "mvlib/config.hpp"
#include <cmath>

#ifdef MVLIB_LOGS_REDEFINED
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL

#define LOG_DEBUG MVLIB_LOG_DEBUG
#define LOG_INFO MVLIB_LOG_INFO
#define LOG_WARN MVLIB_LOG_WARN
#define LOG_ERROR MVLIB_LOG_ERROR
#define LOG_FATAL MVLIB_LOG_FATAL
#endif

namespace mvlib {

Logger &Logger::getInstance() {
  static Logger instance;
  return instance;
}

void Logger::setLogToTerminal(bool v) {
  m_config.logToTerminal.store(v);
  LOG_DEBUG("logToTerminal set to: %d", v);
}

void Logger::setLogToSD(bool v) {
  if (m_started || m_sdLocked) {
    LOG_WARN("setLogToSD() called after logger start — ignored. Set value: %d", v);
    return;
  }
  m_config.logToSD.store(v);
  LOG_DEBUG("logToSD set to: %d", v);
}

void Logger::setPrintWatches(bool v) {
  m_config.printWatches.store(v);
  LOG_DEBUG("printWatches set to: %d", v);
}

void Logger::setPrintTelemetry(bool v) {
  m_config.printTelemetry.store(v);
  LOG_DEBUG("printTelemetry set to: %d", v);
}

void Logger::setPrintWaypoints(bool v) {
  m_config.printWaypoints.store(v);
  LOG_DEBUG("printWaypoints set to: %d", v);
}

void Logger::setLoggerMinLevel(LogLevel level) {
  LOG_DEBUG("SetLoggerMinLevel set to: %d", (int)level);
  m_minLogLevel = level;
}

void Logger::setPoseGetter(std::function<std::optional<Pose>()> getter) {
  unique_lock m(m_mutex, TIMEOUT_MAX);
  if (!m.isLocked() || !getter) return;
  m_getPose = std::move(getter);
}

bool Logger::setRobot(Drivetrain drivetrain) {
  if (m_configSet) {
    LOG_WARN("setRobot(Drivetrain) called twice!");
    return false;
  }
  m_configSet = true;

  if (!drivetrain.leftDrivetrain || !drivetrain.rightDrivetrain) {
    LOG_FATAL("setRobot(Drivetrain) called with nullptr drivetrain arguments!");
    return false;
  }

  m_pLeftDrivetrain = drivetrain.leftDrivetrain;
  m_pRightDrivetrain = drivetrain.rightDrivetrain;

  LOG_INFO("setRobot() successfully set variables!");
  return true;
}

bool Logger::m_checkRobotConfig() {
  unique_lock m(m_mutex, TIMEOUT_MAX);

  bool allValid = true;

  if (!m_pLeftDrivetrain) {
    LOG_FATAL("Left Drivetrain pointer is NULL!");
    allValid = false;
  }
  if (!m_pRightDrivetrain) {
    LOG_FATAL("Right Drivetrain pointer is NULL!");
    allValid = false;
  }

  return allValid;
}

uint32_t Logger::status() const {
  if (!m_task) return pros::E_TASK_STATE_INVALID;
  return m_task->get_state();
}

void Logger::pause() {
  uint32_t st = status();
  if (st != pros::E_TASK_STATE_DELETED && st != pros::E_TASK_STATE_INVALID &&
      st != pros::E_TASK_STATE_SUSPENDED) {
    m_task->suspend();
    LOG_INFO("Logger paused.");
  } else LOG_INFO("Logger cannot be paused as it is not in a running state.");
}

void Logger::resume() {
  uint32_t st = status();
  if (st != pros::E_TASK_STATE_DELETED && st != pros::E_TASK_STATE_INVALID &&
      st == pros::E_TASK_STATE_SUSPENDED) {
    m_task->resume();
    LOG_INFO("Logger resumed.");
  } else LOG_INFO("Logger cannot be resumed as it is not paused.");
}

void Logger::start() {
  if (m_started) {
    LOG_WARN("start() called more than once. Aborted!");
    return;
  }
  m_started = true;

  // SD init happens here.
  if (m_config.logToSD.load() && m_sdFile == nullptr) {
    bool success = m_initSDLogger();
    if (!success) {
      m_config.logToSD.store(false);
      m_sdLocked = true;
      LOG_FATAL("initSDCard failed! Unable to initialize SD card.");
    } else LOG_INFO("Successfully initialized SD card with filename: %s", m_currentFilename);
  }
    
  // Check config
  m_configValid = m_checkRobotConfig();
  if (!m_configValid) LOG_ERROR("At least one pointer set by setRobot(Drivetrain) is nullptr. "
                                "Using speed estimation instead.");
  else LOG_INFO("All pointers set by setRobot(Drivetrain) seem to be valid.");

  // Create task that runs Update
  m_task = std::make_unique<pros::Task>([this]() mutable {
    pros::delay(200);
    // Wait for controller RX settle
    if (m_config.logToTerminal.load()) pros::delay(1000);
    while (true) {
      // Update loop
      try { this->Update(); }
      catch (std::exception &e) {
        LOG_FATAL("%s", e.what());
      }

      // Flush stdout buffer, and wait appropriate time
      if (m_config.logToTerminal.load()) {
        fflush(stdout); // Stdout is only flushed if logging to terminal
        pros::delay(TERMINAL_POLLING_RATE_MS);
      } else pros::delay(SD_POLLING_RATE_MS);
    }
  }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mvlib Logger");
}

void Logger::Update() {
  if (m_config.printWatches.load()) printWatches();
  if (m_config.printWaypoints.load()) printWaypoints();

  static double leftVelocity, rightVelocity;

  if (m_configValid) {
    static auto getGearsetValue = [&](pros::MotorGears gearset) {
      switch (gearset) {
        case pros::MotorGears::rpm_100: return 100.0; break;
        case pros::MotorGears::rpm_200: return 200.0; break;
        case pros::MotorGears::rpm_600: return 600.0; break;
        default:                        return 127.0; // If unknown, leave unmodified
      }
    };

    static auto norm = [&](double rpm, pros::MotorGears gearset) {
      double v = (rpm / getGearsetValue(gearset)) * 127.0;
      return std::clamp(v, -127.0, 127.0);
    };

    pros::MotorGears leftGearing = m_pLeftDrivetrain 
                     ? m_pLeftDrivetrain->get_gearing()
                     : pros::MotorGears::invalid;

    pros::MotorGears rightGearing = m_pRightDrivetrain
                     ? m_pRightDrivetrain->get_gearing()
                     : pros::MotorGears::invalid;

    // Update drivetrain speed
    leftVelocity = norm(m_pLeftDrivetrain->get_actual_velocity(), leftGearing);
    rightVelocity = norm(m_pRightDrivetrain->get_actual_velocity(), rightGearing);
  } else {
    // Because no drivetrain, we do speed approx with pose
    auto pose = m_getPose ? m_getPose() : std::nullopt;


    static Pose prevPose;
    static uint32_t prevMs = pros::millis();
    static double fallbackSpeed = 0.0;
    double avgSpeed = 0.0;
    if (pose.has_value()) {
      uint32_t nowMs = pros::millis();

      double dt = (nowMs - prevMs) / 1000.0; // delta time
      double vx = (dt > 0) ? (pose->x - prevPose.x) / dt : 0.0;
      double vy = (dt > 0) ? (pose->y - prevPose.y) / dt : 0.0;

      // Update drivetrain speed
      avgSpeed = std::sqrt(vx * vx + vy * vy);
      leftVelocity = avgSpeed;
      rightVelocity = avgSpeed;
      fallbackSpeed = avgSpeed;

      prevPose = *pose;
      prevMs = nowMs;
    } else {
      // Use last valid speed if no more pose information
      leftVelocity = fallbackSpeed;
      rightVelocity = fallbackSpeed;
    }
  }

  auto pose = m_getPose ? m_getPose() : std::nullopt;
  if (m_config.printTelemetry.load() && pose.has_value()) {
    double normalizedTheta = fmod(pose->theta, 360.0); // Normalize theta 
    if (normalizedTheta < 0) normalizedTheta += 360.0;
    
    // Print main telemetry
    Logger::getInstance().logMessage(LogLevel::OVERRIDE, 
              "[POSE],%u,%.2f,%.2f,%.2f,%.1f,%.1f", 
              pros::millis(), pose->x, pose->y, normalizedTheta,
              leftVelocity, rightVelocity);
  }
}
} // namespace mvlib
