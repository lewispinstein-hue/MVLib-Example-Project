#pragma once

// Global hardware and control instances shared across the robot code.
#include "pros/adi.hpp"
#define PROS_USE_SIMPLE_NAMES

// IWYU pragma: begin_keep
#include "api.h"
#include "lemlib/api.hpp"
#include "mvlib/Optional/logger_optional_lemlib.hpp"
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

template<class T, class rtn = double>
requires (std::is_arithmetic_v<T> && std::is_arithmetic_v<rtn>)
rtn avg(const std::vector<T>& v) {
  rtn num = 0;
  for (auto& e : v) {
    num += e;
  }
  return v.empty() ? 0 : num /= v.size();
}