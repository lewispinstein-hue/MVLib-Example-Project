#include "globals.hpp"
#include "chassisEventListener.hpp"
#include "mvlib/waypoint.hpp"
#include "pros/misc.hpp"

pros::Controller controller(CONTROLLER_MASTER);

pros::MotorGroup leftMg({-5, -10},
                         pros::MotorGearset::blue,
                         pros::v5::MotorUnits::degrees);

pros::MotorGroup rightMg({20, 16},
                          pros::MotorGearset::blue,
                          pros::v5::MotorUnits::degrees);

pros::Distance rearDist(1); 
pros::Distance frontDist(8);

float TRACK_WIDTH = 8.9;
float TRACK_LENGTH = 12.375f;

// 12.375 length
// 8.9 width
// 8 wheelbase
lemlib::Drivetrain drivetrain(&leftMg,
                              &rightMg,
                              TRACK_WIDTH,
                              lemlib::Omniwheel::NEW_4,
                              400,
                              2);

pros::Imu imu(17);
pros::Rotation horizontal(-11);
pros::Rotation vertical(14);

lemlib::TrackingWheel vertical1(&vertical, 
                                1.95, 
                                -1.0);

lemlib::TrackingWheel horizontal1(&horizontal,
                                  1.95,
                                  -3.4);

lemlib::OdomSensors sensors(&vertical1,
                            nullptr,
                            &horizontal1,
                            nullptr,
                            &imu);

lemlib::ControllerSettings lateralController(10,
                                             0,
                                             3,
                                             3,
                                             1,
                                             100,
                                             3,
                                             500,
                                             20);

lemlib::ControllerSettings angularController(2,
                                              0,
                                              10,
                                              3,
                                              1,
                                              100,
                                              3,
                                              500,
                                              0);

lemlib::Chassis chassis(drivetrain,
                        lateralController,
                        angularController,
                        sensors);

screen::Manager disp;
mvlib::Logger& logger = mvlib::Logger::getInstance();

// mvlib::WaypointHandle PZ = logger.addWaypoint("Parking Zone", {
//   .tarX = 60.5, 
//   .tarY = -2,
//   // .timeoutMs = 50_mvS,
//   .linearTol = 2,
//   .logOffsetEveryMs = 2_mvS,
//   // .retriggerable = true,
// });

// mvlib::WaypointHandle ML = logger.addWaypoint("Match Loader", {
//   .tarX = 64.5, 
//   .tarY = 47,
//   // .tarT = 90,
//   // .timeoutMs = 8_mvS,
//   .linearTol = 2,
//   .thetaTol = 10,
//   .logOffsetEveryMs = 2_mvS,
// });

// mvlib::WaypointHandle HG = logger.addWaypoint("High Goal", {
//   .tarX = 24, 
//   .tarY = 47,
//   // .tarT = 90,
//   // .timeoutMs = 15_mvS,
//   .linearTol = 1,
//   .thetaTol = 5,
//   .logOffsetEveryMs = 2_mvS,
// });



void handleController() {
  typedef enum class ControllerButton {
    BTN_NONE,
    BTN_L1,
    BTN_L2,
    BTN_R1,
    BTN_R2,
    BTN_A,
    BTN_B,
    BTN_X,
    BTN_Y,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
  } ControllerButton;

  struct ButtonBinding {
    ControllerButton action;
    pros::controller_digital_e_t button;
    bool onPress; // true = new_press, false = new_release
  };

  // Map every controller button you care about
  static constexpr ButtonBinding bindings[] = {
      {ControllerButton::BTN_L1,   DIGITAL_L1,   true},
      {ControllerButton::BTN_R1,   DIGITAL_R1,   true},
      {ControllerButton::BTN_R2,   DIGITAL_R2,   true},
      {ControllerButton::BTN_L2,   DIGITAL_L2,   true},
      {ControllerButton::BTN_B,    DIGITAL_B,    false},
      {ControllerButton::BTN_A,    DIGITAL_A,    true},
      {ControllerButton::BTN_Y,    DIGITAL_Y,    true},
      {ControllerButton::BTN_X,    DIGITAL_X,    true},
      {ControllerButton::BTN_UP,   DIGITAL_UP,   true},
      {ControllerButton::BTN_DOWN, DIGITAL_DOWN, false},
     {ControllerButton::BTN_LEFT, DIGITAL_LEFT, true},
     {ControllerButton::BTN_RIGHT,DIGITAL_RIGHT,true},
  };

  ControllerButton event = ControllerButton::BTN_NONE;

  // Find the first button that fired this cycle
  for (const auto& b : bindings) {
    bool triggered = b.onPress
                     ? controller.get_digital_new_press(b.button)
                     : controller.get_digital_new_release(b.button);
    if (triggered) {
      event = b.action;
      break;
    }
  }

  switch (event) {
  case ControllerButton::BTN_L1:
    c::turnToHeading(180, 5000);
  break;

  case ControllerButton::BTN_R1:
    chassis.setPose(-24, -35, 0);
  break;

  case ControllerButton::BTN_R2:
  chassis.setPose(0, 0, 0);
  break;

  case ControllerButton::BTN_L2:
  exportGraphWatches.store(!exportGraphWatches.load());
  break;

  case ControllerButton::BTN_B:
  logger.pause();
  break;

  case ControllerButton::BTN_A:
  {
    const float currTheta = chassis.getPose().theta;
    switch (getFieldQuadrant()) {
      case FieldQuadrant::BlueLeft:  chassis.setPose(25.5, -47, currTheta); break;
      case FieldQuadrant::BlueRight: chassis.setPose(25.5, 47, currTheta); break;
      case FieldQuadrant::RedLeft:   chassis.setPose(-25.5, 47, currTheta); break;
      case FieldQuadrant::RedRight:  chassis.setPose(-25.5, -47, currTheta); break;
      default: break;
    }
  }
  break;

  case ControllerButton::BTN_Y:
  break;

  case ControllerButton::BTN_X:
  break;

  case ControllerButton::BTN_DOWN:
  break;

  case ControllerButton::BTN_UP:
  break;

  case ControllerButton::BTN_LEFT:
  break;
    
  case ControllerButton::BTN_RIGHT:
  break;

  case ControllerButton::BTN_NONE:
  default: break;
  }
}

void setupWatches() {
  logger.setDefaultWatches({/* Defaults */});
  logger.watch("Left Drive Temp", mvlib::LogLevel::INFO, 5_mvS,
  []() { return avg<double, float>(leftMg.get_temperature_all()); },
  mvlib::LevelOverride<float>{
    .elevatedLevel = mvlib::LogLevel::WARN,
    .predicate = mvlib::asPredicate<float>([](float v) { return v > 50; }),
    .label = "High Left Drive Temp"
  }, "%.1f");

  logger.watch("Right Drive Temp", mvlib::LogLevel::INFO, 5_mvS,
  []() { return avg<double, float>(rightMg.get_temperature_all()); },
  mvlib::LevelOverride<float>{
    .elevatedLevel = mvlib::LogLevel::WARN,
    .predicate = mvlib::asPredicate<float>([](float v) { return v > 50; }),
    .label = "High Right Drive Temp"
  }, "%.1f");

  logger.watch("Battery %", mvlib::LogLevel::INFO, 30_mvS,
  []() { return (int)pros::battery::get_capacity(); },
  mvlib::LevelOverride<int>{
    .elevatedLevel = mvlib::LogLevel::WARN,
    .predicate = PREDICATE(v < 30),
    .label = "Low Battery"
  }, "%.0f");
}

constexpr std::pair<double, double> highGoalMappings[] = {
  // X, Y
  {25, 47},   // Blue Right
  {25, -47},  // Blue Left
  {-25, -47}, // Red Right
  {-25, 47}   // Red Left
};

FieldQuadrant getFieldQuadrant(double linTol, double angTol) {
  lemlib::Pose pose = chassis.getPose();

  // Iterate through the 4 defined quadrants
  for (int i = 0; i < 4; ++i) {
    double targetX = highGoalMappings[i].first;
    double targetY = highGoalMappings[i].second;

    double targetTheta = 0.0;
    bool restrictOnTheta = true;

    // Set the target theta based on the quadrant
    switch (static_cast<FieldQuadrant>(i)) {
      case FieldQuadrant::BlueLeft:
      case FieldQuadrant::BlueRight:
        targetTheta = 90.0; 
        break;

      case FieldQuadrant::RedLeft:
      case FieldQuadrant::RedRight:
        targetTheta = 270.0;
        break;

      default: break;
    }

    double distance = std::hypot(pose.x - targetX, pose.y - targetY);    
    // Calculate shortest angular difference (handles 360-degree wrap)
    double angleDiff = std::abs(std::remainder(pose.theta - targetTheta, 360.0));

    // Check if within tolerances
    if (distance < linTol && (!restrictOnTheta || angleDiff < angTol)) {
      return static_cast<FieldQuadrant>(i);
    }
  }
  // Return Unknown if the robot is not within the tolerance of any quadrant
  return FieldQuadrant::Unknown;
}