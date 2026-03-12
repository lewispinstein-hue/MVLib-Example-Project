#include "main.h"
#include "globals.hpp"
#include "mvlib/core.hpp"
#include <cstdint>

template<class T>
requires std::is_arithmetic_v<T>
constexpr float toFloat(T v) { return static_cast<float>(std::floor(v + 0.5)); }

struct simPose {
  double x;
  double y;
  double theta;
}; simPose simpose{0, 0, 0};

inline const simPose getSimPose() { return simpose; }

void updateSimPose() { 
  chassis.setPose(toFloat(simpose.x), 
                  toFloat(simpose.y), 
                  toFloat(simpose.theta)); 
}

void setSimPose(const simPose pose) {
  simpose.x = pose.x;
  simpose.y = pose.y;
  simpose.theta = pose.theta;
}

simPose userPose{0, 0, 0};
static uint32_t lastSimUpdate = 0;
constexpr uint16_t SIM_DEBOUNCE_MS = 150;
bool simMode = false;
void moveSimPose() {
  if (pros::millis() - lastSimUpdate < SIM_DEBOUNCE_MS) return;
  lastSimUpdate = pros::millis();
  // use arrow keys to move, B to update, and L1/L2 to turn
  if (controller.get_digital(DIGITAL_LEFT)) userPose.x -= 1;
  else if (controller.get_digital(DIGITAL_RIGHT)) userPose.x += 1;
  if (controller.get_digital(DIGITAL_UP)) userPose.y += 1;
  else if (controller.get_digital(DIGITAL_DOWN)) userPose.y -= 1;
  if (controller.get_digital(DIGITAL_L1)) userPose.theta += 1;
  else if (controller.get_digital(DIGITAL_L2)) userPose.theta -= 1;

  if (controller.get_digital_new_press(DIGITAL_B)) simMode = !simMode;

  if (simMode) {
    setSimPose({userPose.x, userPose.y, userPose.theta}); // Set the pose
    updateSimPose(); // Then update it
  }
}

void initialize() {
  mvlib::setOdom(logger, &chassis);
  logger.setRobot({
    .leftDrivetrain = &left_mg,
    .rightDrivetrain = &right_mg
  });
  chassis.calibrate();
  chassis.setPose(-60, -60, 0);
  
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
    handleMisc();
    
    double LEFT_Y = controller.get_analog(ANALOG_LEFT_Y);
    double RIGHT_X = controller.get_analog(ANALOG_RIGHT_X);

    LEFT_Y = expoForward(LEFT_Y, 1.9, deadband);
    RIGHT_X = expoTurn(RIGHT_X, 2.8, deadband);

    chassis.arcade(LEFT_Y, RIGHT_X, false, 0.4);
    pros::delay(20);
  }
}