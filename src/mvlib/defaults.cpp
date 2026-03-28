#include "mvlib/core.hpp"
#include "mvlib/private/forwardLogMacros.h"
#include "pros/misc.hpp"
#include <sys/types.h>

namespace mvlib {

struct WatchInfo {
  int16_t currVal{0};
  int16_t prevVal{0};
  int displayValue{0};
};  

bool Logger::setDefaultWatches(const DefaultWatches& watches) {
  if (!m_configSet || !m_configValid) {
    _MVLIB_FORWARD_WARN("Default watches could not be set because config is not set or invalid!");
    return false;
  }

  constexpr uint TEMP_THRESHOLD = 50;
  if (watches.leftDrivetrainWatchdog) {
    Logger::getInstance().watch("Left Drivetrain OK:", LogLevel::INFO, true,
      [info = WatchInfo{}, this]() mutable {        uint temp = m_pLeftDrivetrain ? m_pLeftDrivetrain->get_temperature() : 0;
        info.currVal = temp;

        // Case 1: not overheating -> overheating
        if (info.currVal >= TEMP_THRESHOLD && info.prevVal < TEMP_THRESHOLD) info.displayValue = info.currVal;
        // Case 2: overheating -> not overheating
        else if (info.currVal <= TEMP_THRESHOLD && info.prevVal > TEMP_THRESHOLD) info.displayValue = info.currVal;

        info.prevVal = info.currVal;
        return info.displayValue;
      }, 
        LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > TEMP_THRESHOLD),
        .label = "Left Drivetrain Overheating:"
      },
      "%d"
    );
    _MVLIB_FORWARD_INFO("Created default Left Drivetrain watch.");
  }

  if (watches.rightDrivetrainWatchdog) {
    Logger::getInstance().watch("Right Drivetrain OK:", LogLevel::INFO, true,
      [info = WatchInfo{}, this]() mutable {       
        uint temp = m_pRightDrivetrain ? m_pRightDrivetrain->get_temperature() : 0;
        info.currVal = temp;

        // Case 1: not overheating -> overheating
        if (info.currVal >= TEMP_THRESHOLD && info.prevVal < TEMP_THRESHOLD) info.displayValue = info.currVal;
        // Case 2: overheating -> not overheating
        else if (info.currVal <= TEMP_THRESHOLD && info.prevVal > TEMP_THRESHOLD) info.displayValue = info.currVal;

        info.prevVal = info.currVal;
        return info.displayValue;
      }, LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > TEMP_THRESHOLD),
        .label = "Right Drivetrain Overheating:"
      },
      "%d"
    );
    _MVLIB_FORWARD_INFO("Created default Right Drivetrain watch.");
  }

  if (watches.batteryWatchdog) {
    Logger::getInstance().watch("Battery Temp OK:", LogLevel::INFO, true,
      [info = WatchInfo{}, this]() mutable {        uint temp = pros::battery::get_temperature();
        info.currVal = temp;

        if (info.currVal >= TEMP_THRESHOLD && info.prevVal < TEMP_THRESHOLD) info.displayValue = info.currVal;
        else if (info.currVal <= TEMP_THRESHOLD && info.prevVal > TEMP_THRESHOLD) info.displayValue = info.currVal;

        info.prevVal = info.currVal;
        return info.displayValue;
      },
      LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > 45),
        .label = "Battery Temp High:"
      }, 
      "%d"
    );
    _MVLIB_FORWARD_INFO("Created default Battery Temperature Watch");

    constexpr uint MIN_BATTERY_VOLTAGE = 11200;
    constexpr uint MAX_BATTERY_VOLTAGE = 13000;

    Logger::getInstance().watch("Battery Voltage OK:", LogLevel::INFO, true,
      [info = WatchInfo{}, this]() mutable {        uint curr = pros::battery::get_voltage();
        uint prev = info.prevVal;

        bool currBad = (curr < MIN_BATTERY_VOLTAGE || curr > MAX_BATTERY_VOLTAGE);
        bool prevBad = (prev < MIN_BATTERY_VOLTAGE || prev > MAX_BATTERY_VOLTAGE);

        // Only trigger on transitions between GOOD <-> BAD
        if (currBad != prevBad) {
          info.displayValue = curr;
        }

        info.prevVal = curr;
        return info.displayValue;
      },
      LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v < MIN_BATTERY_VOLTAGE || v > MAX_BATTERY_VOLTAGE),
        .label = "Battery Voltage Anomaly:"
      }, 
      "%d"
    );

    _MVLIB_FORWARD_INFO("Created default Battery Voltage Watch");
  }

  return true;
}
} // namespace mvlib
