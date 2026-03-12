#include "main.h"
#include "globals.hpp"
#include "mvlib/core.hpp"
#include <cstdint>

void initialize() {
  mvlib::setOdom(logger, &chassis);
  logger.setRobot({
    .leftDrivetrain = &left_mg,
    .rightDrivetrain = &right_mg
  });
  logger.setLoggerMinLevel(mvlib::LogLevel::DEBUG);
  chassis.calibrate();
  chassis.setPose(50, 6, 270);
  
  setupWatches();
  logger.start();
}

double expoForward(double input, double expoForwards, double deadband) {
  if (fabs(input) < deadband) return 0;
  double norm = input / 127.0;
  double curved = pow(fabs(norm), expoForwards);
  curved = curved * (norm >= 0 ? 1 : -1); 
  if (fabs(curved) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return curved * 127.0;
}

double expoTurn(double input, double expoTurn, double deadband) {
  if (fabs(input) < deadband) return 0;
  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = 0.38 * linear + 0.42 * exponential;
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * 90.0;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  const uint8_t deadband = 10;
  while (true) {
    handleController();
    
    float LEFT_Y = controller.get_analog(ANALOG_LEFT_Y);
    float RIGHT_X = controller.get_analog(ANALOG_RIGHT_X);
    
    LEFT_Y = expoForward(LEFT_Y, 1.9, deadband);
    RIGHT_X = expoTurn(RIGHT_X, 2.8, deadband);
    
    chassis.arcade(LEFT_Y, RIGHT_X, false, 0.48);
    pros::delay(20);
  }
}