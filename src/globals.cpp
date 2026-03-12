#include "globals.hpp"
#include "chassisEventListener.hpp"
#include "pros/misc.hpp"

pros::Controller controller(CONTROLLER_MASTER);

pros::MotorGroup left_mg({-5, -10},
                         pros::MotorGearset::blue,
                         pros::v5::MotorUnits::degrees);

pros::MotorGroup right_mg({20, 16},
                          pros::MotorGearset::blue,
                          pros::v5::MotorUnits::degrees);

float TRACK_WIDTH = 8.9;
float ROBOT_HEIGHT = 12.40f;

pros::Imu imu(18);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              TRACK_WIDTH,
                              lemlib::Omniwheel::NEW_4,
                              400,
                              2);

pros::Rotation horizontal(-11);
pros::Rotation vertical(14);

lemlib::TrackingWheel vertical1(&vertical, 
                                1.95, 
                                0.5);

lemlib::TrackingWheel horizontal1(&horizontal,
                                  1.95,
                                  0);

lemlib::OdomSensors sensors(&vertical1,
                            nullptr,
                            &horizontal1,
                            nullptr,
                            &imu);

lemlib::ControllerSettings lateral_controller(10,
                                             0,
                                             3,
                                             3,
                                             1,
                                             100,
                                             3,
                                             500,
                                             20);

lemlib::ControllerSettings angular_controller(2,
                                              0,
                                              10,
                                              3,
                                              1,
                                              100,
                                              3,
                                              500,
                                              0);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors);

screen::Manager disp;
mvlib::Logger& logger = mvlib::Logger::getInstance();

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
  for (const auto &b : bindings) {
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
    break;

  case ControllerButton::BTN_B:
  logger.pause();
    break;

  case ControllerButton::BTN_A:
  logger.resume();
    break;

  case ControllerButton::BTN_Y:
  LOG_INFO("Status: %d", logger.status());
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
  logger.watch("Left Drive Temp", mvlib::LogLevel::INFO, 10_mvS,
  []() { return avg<double, float>(left_mg.get_temperature_all()); },
  mvlib::LevelOverride<float>{
    .elevatedLevel = mvlib::LogLevel::WARN,
    .predicate = mvlib::asPredicate<float>([](float v) { return v > 50; }),
    .label = "High Left Drive Temp"
  }, "%.1f");

  logger.watch("Right Drive Temp", mvlib::LogLevel::INFO, 10_mvS,
  []() { return avg<double, float>(right_mg.get_temperature_all()); },
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