#include "main.h"
#include <unistd.h>

namespace control {
 
double normVel(double rpm) {
  double v = (rpm / 4.7244094488); // Locked at blue gearset
  // If your using a different gearset, change this to (maxRPM / 127)
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
  
  constexpr uint16_t TURN_JOYSTICK_CUTTOFF = 126; // Out of 127
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
  constexpr uint8_t DEFAULT_SPEED = 90;

  uint8_t retExpo = (speed > MAX_SPEED_OVERRIDE || CONTROL_OVERRIDE) 
                     ? OVERRIDE_SPEED : DEFAULT_SPEED;

  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = 0.38 * linear + 0.42 * exponential;
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * retExpo;
}
} // namespace control

std::atomic<bool> exportGraphWatches{false};

float rawRightX, rawLeftY;
float LEFT_Y, RIGHT_X;
float prevTurn, prevThrottle;

void initialize() {
  rightMg.set_brake_mode_all(pros::MotorBrake::brake);
  leftMg.set_brake_mode_all(pros::MotorBrake::brake);
  pros::screen::erase();

  // setupWatches();
  logger.setLogSystemInfo(true);
  logger.setLoggerMinLevel(mvlib::LogLevel::DEBUG);
  logger.setLogToSD(false);
  logger.setTimings({
    .terminal_polling_rate = 10
  });

  mvlib::Pose pose{};
  mvlib::setOdom([&]() -> std::optional<mvlib::Pose> {
    pose.x += 0.1;
    pose.y += 0.1;
    pose.theta += 0.1;
    return pose;
  });
  logger.setRobot({
    .leftDrivetrain = &leftMg,
    .rightDrivetrain = &rightMg
  });

  chassis.calibrate();
  chassis.setPose(-45, 5.5, 90);
  // chassis.setPose(0, 0, 0);

  // logger.watch("Throttle Wanted", mvlib::LogLevel::INFO, 150_mvMs, 
  //   []() { return prevThrottle; }, 
  //   mvlib::LevelOverride<float>{}, "%.2f");

    logger.addWaypoint("Blue Left High Goal", {
    .tarX = 24,
    .tarY = -47,
    .tarT = 90,
    .linearTol = 2.5,
    .thetaTol = 10,
    .retriggerable = true
  });

  logger.addWaypoint("Blue Right High Goal", {
    .tarX = 24, 
    .tarY = 47,
    .tarT = 90,
    .linearTol = 2.5,
    .thetaTol = 10,
    .retriggerable = true
  });

  logger.addWaypoint("Red Right High Goal", {
    .tarX = -24, 
    .tarY = 47,
    .tarT = 270,
    .linearTol = 2.5,
    .thetaTol = 10,
    .retriggerable = true
  });
  logger.addWaypoint("Red Left High Goal", {
    .tarX = -24, 
    .tarY = -47,
    .tarT = 270,
    .linearTol = 2.5,
    .thetaTol = 10,
    .retriggerable = true
  });
  logger.start();
}

void screenTask() {
  pros::screen::erase_rect(0, 5, 480, 65);

  lemlib::Pose pose = chassis.getPose();
  pros::screen::print(pros::E_TEXT_SMALL, 0, 5, 
                "X: %.2f | Y: %.2f | T: %.2f", pose.x, pose.y, pose.theta);
  pros::screen::print(pros::E_TEXT_SMALL, 0, 25, 
                "Back: %d | Front: %d", rearDist.get_distance(), frontDist.get_distance());

  // pros::screen::print(pros::E_TEXT_SMALL, 0, 45, 
  //                     "WP Dist: PZ: %.2f | ML: %.2f, %.1f | HG: %.2f, %.1f", 
  //                     PZ.getOffset().totalOffset,
  //                     ML.getOffset().totalOffset, ML.getOffset().offT.value_or(0),
  //                     HG.getOffset().totalOffset, HG.getOffset().offT.value_or(0));

  pros::delay(65);
}

// Value of 0 disables slew on that axis
control::Slew slew(
  25, 0, 
  18, 0
);

void auton() {
  logger.info("Starting auton"); 
  uint32_t startTime = pros::millis();
  chassis.setPose(50, 6, 270); // Blue parking zone start

  /* 
   * Route: move from start -> grouped 4 balls -> match loader ->
   * backwards into high goal -> parking zone
  */

  chassis.moveToPose(23, 12, 0, 1000);
  // Slower while intake ing
  chassis.moveToPoint(24, 31, 1100, {.maxSpeed = 80}); 
  chassis.moveToPose(47, 47, 90, 1300);
  // Dont eject balls when going into loader
  chassis.moveToPose(65, 47, 90, 700, {.maxSpeed = 80});
  pros::delay(1200);
  chassis.moveToPose(24, 47, 90, 1300, 
                     {.forwards = false, .maxSpeed = 100});
  pros::delay(1000); // While scoring
  chassis.setPose(25, 47, chassis.getPose().theta);
  chassis.moveToPose(44, 42, 130, 900, 
                     {.minSpeed = 90}); // Drift to turn
  chassis.waitUntil(18); // after moving 18in, cancel to drift
  chassis.moveToPose(60, -2, 180, 1200, 
                     {.minSpeed = 100}); // Slam into parking zone to clear


  logger.info("Auton took %d ms", pros::millis() - startTime);
}

void opcontrol() {

  float turn = control::expoTurn(RIGHT_X, 2, 10);

  logger.watch("Turn Raw", mvlib::LogLevel::INFO, 150_mvMs, []() {
    return controller.get_analog(ANALOG_RIGHT_X);
  }, mvlib::LevelOverride<int32_t>{}, "%d");

  logger.watch("Turn Post-processing", mvlib::LogLevel::INFO, 150_mvMs, 
    [&]() { return turn; },
    mvlib::LevelOverride<float>{}, "%.2f");

  pros::Task telemetry(screenTask);
  // disp.drawBottomButtons(false);
  // if (disp.waitForBottomButtonTap(0) == screen::ButtonId::MIDDLE) {
  //   auton();
  // }
  // disp.clearScreen();

  constexpr uint8_t deadband = 10;
  static float prevThrottle = 0;
  static float prevTurn = 0;
  while (true) {
    handleController();
     rawLeftY = controller.get_analog(ANALOG_LEFT_Y);
     rawRightX = controller.get_analog(ANALOG_RIGHT_X);

     LEFT_Y = control::slewLimit(
        rawLeftY,
        prevThrottle,
        control::MotionType::FORWARD,
        slew
    );
    prevThrottle = LEFT_Y;

     RIGHT_X = control::slewLimit(
        rawRightX,
        prevTurn,
        control::MotionType::TURN,
        slew
    );
    prevTurn = RIGHT_X;

    float throttle = control::expoThrottle(LEFT_Y, 1.9, deadband);
     turn = control::expoTurn(RIGHT_X, 2, deadband);

    chassis.arcade(throttle, turn, false, 0.55);
    logger.info("TESTING");
    pros::delay(15);
  }
}