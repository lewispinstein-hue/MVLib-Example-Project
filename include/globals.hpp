#pragma once

// Global hardware and control instances shared across the robot code.
#define PROS_USE_SIMPLE_NAMES

// IWYU pragma: begin_keep
#include "api.h"
#include "lemlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"
#include "mvlib/core.hpp"
#include "screen.hpp"
// IWYU pragma: end_keep

extern pros::Controller controller;
extern pros::MotorGroup leftMg;
extern pros::MotorGroup rightMg;

extern float ROBOT_WIDTH;
extern float ROBOT_HEIGHT;

extern pros::Imu imu;

extern lemlib::Drivetrain drivetrain;
extern pros::Rotation horizontal;
extern pros::Rotation vertical;
extern lemlib::TrackingWheel horizontal1;
extern lemlib::TrackingWheel vertical1;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateralController;
extern lemlib::ControllerSettings angularController;
extern lemlib::Chassis chassis;

extern screen::Manager disp;
extern mvlib::Logger& logger;

extern pros::Distance rearDist;
extern pros::Distance frontDist;

void handleController();
void setupWatches();

/**
 * @brief Returns the average value of the vector.
 * 
 * @tparam T The type of the vector (can usually be deduced)
 * @tparam rtn The return type (defaults to double)
 * @param v 
 * @return rtn casted average of the vector, skipping non-finite values
*/
template<class T, class rtn = double>
requires (std::is_arithmetic_v<T> && std::is_arithmetic_v<rtn>)
rtn avg(const std::vector<T>& v) {
  if (v.empty()) return 0;
  rtn num = 0;
  size_t iter = 0;
  for (auto& e : v) {
    if (!std::isfinite(e)) continue; 
    num += e;
    iter++;
  }
  return num /= iter;
}