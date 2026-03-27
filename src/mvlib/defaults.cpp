#include "mvlib/core.hpp"
#include "mvlib/logMacros.h"
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
    LOG_WARN("Default watches could not be set because config is not set or invalid!");
    return false;
  }

  constexpr uint TEMP_THRESHOLD = 50;
  if (watches.leftDrivetrainWatchdog) {
    WatchInfo leftDT{}; 
    leftDT.currVal = 0;
    leftDT.prevVal = 0;
    Logger::getInstance().watch("Left Drivetrain OK:", LogLevel::INFO, true,
      [&]() {
        uint temp = m_pLeftDrivetrain ? m_pLeftDrivetrain->get_temperature() : 0;
        leftDT.currVal = temp;

        // Case 1: not overheating -> overheating
        if (leftDT.currVal >= TEMP_THRESHOLD && leftDT.prevVal < TEMP_THRESHOLD) leftDT.displayValue = leftDT.currVal;
        // Case 2: overheating -> not overheating
        else if (leftDT.currVal <= TEMP_THRESHOLD && leftDT.prevVal > TEMP_THRESHOLD) leftDT.displayValue = leftDT.currVal;

        leftDT.prevVal = leftDT.currVal;
        return leftDT.displayValue;
      }, 
        LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > TEMP_THRESHOLD),
        .label = "Left Drivetrain Overheating:"
      },
      "%d"
    );
    LOG_INFO("Created default Left Drivetrain watch.");
  }

  if (watches.rightDrivetrainWatchdog) {
    WatchInfo rightDT{};
    rightDT.currVal = 0;
    rightDT.prevVal = 0;
    Logger::getInstance().watch("Right Drivetrain OK:", LogLevel::INFO, true,
      [&]() {
        uint temp = m_pRightDrivetrain ? m_pRightDrivetrain->get_temperature() : 0;
        rightDT.currVal = temp;

        // Case 1: not overheating -> overheating
        if (rightDT.currVal >= TEMP_THRESHOLD && rightDT.prevVal < TEMP_THRESHOLD) rightDT.displayValue = rightDT.currVal;
        // Case 2: overheating -> not overheating
        else if (rightDT.currVal <= TEMP_THRESHOLD && rightDT.prevVal > TEMP_THRESHOLD) rightDT.displayValue = rightDT.currVal;

        rightDT.prevVal = rightDT.currVal;
        return rightDT.displayValue;
      }, LevelOverride<int>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > TEMP_THRESHOLD),
        .label = "Right Drivetrain Overheating:"
      },
      "%d"
    );
    LOG_INFO("Created default Right Drivetrain watch.");
  }

  if (watches.batteryWatchdog) {
    WatchInfo batTemp{};
    batTemp.currVal = pros::battery::get_temperature();
    batTemp.prevVal = batTemp.currVal;
    Logger::getInstance().watch("Battery Temp OK:", LogLevel::INFO, true,
      [&]() -> uint {
        uint temp = pros::battery::get_temperature();
        batTemp.currVal = temp;

        if (batTemp.currVal >= TEMP_THRESHOLD && batTemp.prevVal < TEMP_THRESHOLD) batTemp.displayValue = batTemp.currVal;
        else if (batTemp.currVal <= TEMP_THRESHOLD && batTemp.prevVal > TEMP_THRESHOLD) batTemp.displayValue = batTemp.currVal;

        batTemp.prevVal = batTemp.currVal;
        return batTemp.displayValue;
      },
      LevelOverride<uint>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v > 45),
        .label = "Battery Temp High:"
      }, 
      "%d"
    );
    LOG_INFO("Created default Battery Temperature Watch");

    WatchInfo batVolt{};
    // Init
    batVolt.currVal = pros::battery::get_voltage();
    batVolt.prevVal = batVolt.currVal;
    constexpr uint MIN_BATTERY_VOLTAGE = 11200;
    constexpr uint MAX_BATTERY_VOLTAGE = 13000;

    Logger::getInstance().watch("Battery Voltage OK:", LogLevel::INFO, true,
      [&]() -> uint {
        uint curr = pros::battery::get_voltage();
        uint prev = batVolt.prevVal;

        bool currBad = (curr < MIN_BATTERY_VOLTAGE || curr > MAX_BATTERY_VOLTAGE);
        bool prevBad = (prev < MIN_BATTERY_VOLTAGE || prev > MAX_BATTERY_VOLTAGE);

        // Only trigger on transitions between GOOD <-> BAD
        if (currBad != prevBad) {
          batVolt.displayValue = curr;
        }

        batVolt.prevVal = curr;
        return batVolt.displayValue;
      },
      LevelOverride<uint>{
        .elevatedLevel = LogLevel::WARN,
        .predicate = PREDICATE(v < MIN_BATTERY_VOLTAGE || v > MAX_BATTERY_VOLTAGE),
        .label = "Battery Voltage Anomaly:"
      }, 
      "%d"
    );

    LOG_INFO("Created default Battery Voltage Watch");
  }

  return true;
}
} // namespace mvlib
