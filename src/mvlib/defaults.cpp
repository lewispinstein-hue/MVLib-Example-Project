#include "mvlib/core.hpp"
#include "mvlib/logMacros.h"
#include "pros/misc.hpp"

namespace mvlib {

struct WatchInfo {
  int16_t currVal = -1;
  int16_t prevVal = -1;
  int value{};
};  

bool Logger::setDefaultWatches(const DefaultWatches& watches) {
  if (!m_configSet || !m_configValid) {
    LOG_WARN("Default watches could not be set because config is not set or invalid!");
    return false;
  }

  if (watches.leftDrivetrainWatchdog) {
    WatchInfo leftDT{}; 
    Logger::getInstance().watch("Left Drivetrain OK:", LogLevel::INFO, true,
      [&]() {
        uint temp = m_pLeftDrivetrain ? m_pLeftDrivetrain->get_temperature() : 0;
        leftDT.currVal = temp;

        // Case 1: not overheating -> overheating
        if (leftDT.currVal >= 50 && leftDT.prevVal < 50) leftDT.value = leftDT.currVal;
        // Case 2: overheating -> not overheating
        else if (leftDT.currVal <= 50 && leftDT.prevVal > 50) leftDT.value = leftDT.currVal;

        leftDT.prevVal = leftDT.currVal;
        return leftDT.value;
      }, 
        LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > 50),
        .label = "Left Drivetrain Overheating:"
      },
      "%d"
    );
    LOG_INFO("Created default Left Drivetrain watch.");
  }

  if (watches.rightDrivetrainWatchdog) {
    WatchInfo rightDT{};
    Logger::getInstance().watch("Right Drivetrain OK:", LogLevel::INFO, true,
      [&]() {
        uint temp = m_pRightDrivetrain ? m_pRightDrivetrain->get_temperature() : 0;
        rightDT.currVal = temp;

        // Case 1: not overheating -> overheating
        if (rightDT.currVal >= 50 && rightDT.prevVal < 50) rightDT.value = rightDT.currVal;
        // Case 2: overheating -> not overheating
        else if (rightDT.currVal <= 50 && rightDT.prevVal > 50) rightDT.value = rightDT.currVal;

        rightDT.prevVal = rightDT.currVal;
        return rightDT.value;
      }, LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > 50),
        .label = "Right Drivetrain Overheating:"
      },
      "%d"
    );
    LOG_INFO("Created default Right Drivetrain watch.");
  }

  if (watches.batteryWatchdog) {
    static bool forceBatteryPredicate = false;
    Logger::getInstance().watch("NULL", LogLevel::OFF, true,
      [&]() {
        uint temp = pros::battery::get_temperature();
        uint32_t voltage = pros::battery::get_voltage();
        uint32_t current = pros::battery::get_current();

        if (temp >= 50 ||
            voltage <= 3000 ||
            current >= 5000) {
          forceBatteryPredicate = true;
        } else {
          forceBatteryPredicate = false;
        }
        return std::format(",{},{},{},", temp, voltage, current);
      },
      LevelOverride<std::string>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = asPredicate<std::string>([&](const std::string&) -> bool {
          return forceBatteryPredicate;
        }),
        .label = "Battery Warning:"
      }, 
      "%s"
    );
  }

  return true;
}
} // namespace mvlib
