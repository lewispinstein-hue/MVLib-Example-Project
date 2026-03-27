#include "main.h" 
#include "globals.hpp"

namespace control {

double normVel(double rpm) {
  double v = (rpm / 4.7244094488); // Locked at blue gearset
  return std::clamp(v, -127.0, 127.0);
}

/**
 * @struct Slew
 * @brief Slew rate limiter. Used with the slewLimit function
 * @note Set values to 0 to disable slew on that axis
*/
struct Slew {
  /** @brief Slew applied when increasing magnitude/speed (in turning mode) */
  float risingAngle; 

  /** @brief Slew applied when decreasing magnitude/speed (in turning mode) */
  float fallingAngle;

  // |---|<*>|---| //

  /** @brief Slew applied when increasing magnitude/speed (in forward mode) */
  float risingThrottle;

  /** @brief Slew applied when decreasing magnitude/speed (in forward mode) */
  float fallingThrottle;

  static constexpr float infinity = std::numeric_limits<float>::infinity();
  Slew(const float risingAngle, const float fallingAngle,
       const float risingThrottle, const float fallingThrottle)
      : risingAngle(risingAngle <= 0 ? infinity : risingAngle),
        fallingAngle(fallingAngle <= 0 ? infinity : fallingAngle),
        risingThrottle(risingThrottle <= 0 ? infinity : risingThrottle),
        fallingThrottle(fallingThrottle <= 0 ? infinity : fallingThrottle) {}
};

enum class MotionType {
  FORWARD, /// Forward motion, lateral movement
  TURN,    /// Turn motion, angular movement
};

/**
 * @brief Slew rate limiter
 * 
 * @param target target value
 * @param prev previous value
 * @param type forward or turn
 * @param slew slew struct
 * 
 * @return Limited @c target value with applied slew based on type, slew, and prev
*/
double slewLimit(double target, double prev, const MotionType& type, 
                 const Slew& slew) {
    
  if (target == prev) return target;
  double delta = target - prev;
  // Same direction?
  bool sameDirection = (prev * target > 0);

  double riseDelta = (type == MotionType::FORWARD) ? slew.risingThrottle : slew.risingAngle;
  double fallDelta = (type == MotionType::FORWARD) ? slew.fallingThrottle : slew.fallingAngle;

  // Increasing magnitude in same direction -> accelerating
  bool accelerating = sameDirection && (fabs(target) > fabs(prev));
  double maxDelta = accelerating ? riseDelta : fallDelta;
  if (delta > maxDelta) delta = maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  return prev + delta;
}

double expoThrottle(const double& input, const double& expoThrottle, 
                   const double& deadband) {
  if (fabs(input) < deadband) return 0;
  double norm = input / 127.0;
  double curved = pow(fabs(norm), expoThrottle);
  curved = curved * (norm >= 0 ? 1 : -1); 
  if (fabs(curved) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return curved * 127.0;
}

double expoTurn(const double& input, const double& expoTurn, 
                   const double& deadband) {  
  if (fabs(input) < deadband) return 0;
  
  constexpr uint16_t TURN_JOYSTICK_CUTTOFF = 129; // Out of 127
  bool CONTROL_OVERRIDE = false;
  if (abs(controller.get_analog(ANALOG_RIGHT_X)) > TURN_JOYSTICK_CUTTOFF) {
    CONTROL_OVERRIDE = true;
  } else {
    CONTROL_OVERRIDE = false;
  }
  
  float speed = normVel(
                      (avg<double, float>(leftMg.get_actual_velocity_all()) + 
                       avg<double, float>(rightMg.get_actual_velocity_all())) / 2);

  constexpr uint8_t MAX_SPEED_OVERRIDE = 126;
  constexpr uint8_t OVERRIDE_SPEED = 127;
  constexpr uint8_t DEFAULT_SPEED = 95;

  uint8_t retExpo = (speed > MAX_SPEED_OVERRIDE || CONTROL_OVERRIDE) 
                     ? OVERRIDE_SPEED : DEFAULT_SPEED;

  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = 0.38 * linear + 0.42 * exponential;
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * retExpo;
}

float getDesaturateBias() {
  constexpr uint8_t CUTTOFF = 100;
  constexpr double OVERRIDE_BIAS = 0.65; // When driving faster than CUTTOFF
  constexpr double DEFAULT_BIAS = 0.48; // When not driving faster than CUTTOFF

  // If driving slow, we want high desat bias. If driving fast, we want lower bias
  float speed = normVel((avg<double, float>(leftMg.get_actual_velocity_all()) + 
                 avg<double, float>(rightMg.get_actual_velocity_all())) / 2);
  return (speed >= CUTTOFF) ? OVERRIDE_BIAS : DEFAULT_BIAS;
}
} // namespace control

void initialize() {
  rightMg.set_brake_mode_all(pros::MotorBrake::brake);
  leftMg.set_brake_mode_all(pros::MotorBrake::brake);
  pros::screen::erase();

  mvlib::setOdom(&chassis);
  logger.setRobot({
    .leftDrivetrain = &leftMg,
    .rightDrivetrain = &rightMg
  });
  logger.setLoggerMinLevel(mvlib::LogLevel::DEBUG);
  logger.setLogToSD(false);
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  
  logger.start();
  setupWatches();
}

void screenTask() {
  pros::screen::erase_rect(0, 5, 270, 65);
  // Pose
  lemlib::Pose pose = chassis.getPose();
  pros::screen::print(pros::E_TEXT_SMALL, 0, 5, 
                "X: %.2f | Y: %.2f | T: %.2f", pose.x, pose.y, pose.theta);
  pros::screen::print(pros::E_TEXT_SMALL, 0, 25, 
                "Back: %d | Front: %d", rearDist.get_distance(), frontDist.get_distance());

  pros::screen::print(pros::E_TEXT_SMALL, 0, 45, 
                      "Waypoint Distances (lin, ang): PZ: %.2f, null | ML: %.2f, %.1f | HG: %.2f, %.1f", 
                      PZ.getOffset().totalOffset,
                      ML.getOffset().totalOffset, ML.getOffset().offT.value_or(0),
                      HG.getOffset().totalOffset, HG.getOffset().offT.value_or(0));

  
  pros::delay(50);
}

// Value of 0 disables slew on that axis
control::Slew slew(
  25, 0, 
  18, 0
);

void opcontrol() {
  constexpr uint8_t deadband = 10;
  static float prevThrottle = 0;
  static float prevTurn = 0;
  pros::Task telemetry(screenTask);
  while (true) {

    handleController();
    screenTask();
    const float rawLeftY = controller.get_analog(ANALOG_LEFT_Y);
    const float rawRightX = controller.get_analog(ANALOG_RIGHT_X);

    float LEFT_Y = control::slewLimit(
        rawLeftY,
        prevThrottle,
        control::MotionType::FORWARD,
        slew
    );
    prevThrottle = LEFT_Y;

    float RIGHT_X = control::slewLimit(
        rawRightX,
        prevTurn,
        control::MotionType::TURN,
        slew
    );
    prevTurn = RIGHT_X;

    float throttle = control::expoThrottle(LEFT_Y, 1.9, deadband);
    float turn = control::expoTurn(RIGHT_X, 2.8, deadband);
    
    chassis.arcade(throttle, turn, false, control::getDesaturateBias());
    pros::delay(20);
  }
}