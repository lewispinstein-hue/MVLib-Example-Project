#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "mvlib/Optional/logger_optional_lemlib.hpp" // IWYU pragma: keep
#include "mvlib/core.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "screen.hpp" // IWYU pragma: keep


// Creating motors and controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({16, 17},
                pros::MotorGearset::blue,
                pros::v5::MotorUnits::degrees); // Creates a motor group with forwards ports 1 & 3 and reversed port 2

pros::MotorGroup right_mg({-4, 5, -6},
                pros::MotorGearset::blue, 
                pros::v5::MotorUnits::degrees); // Creates a motor group with forwards port 5 and reversed ports 4 & 6

// Setup LemLib 

// create an imu on port 10
pros::Imu imu(10);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1, set to nullptr
                            nullptr, // horizontal tracking wheel 2, also set to nullptr
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// Finally, create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
screen::Manager disp;
auto& logger = mvlib::Logger::getInstance();

void initialize() {
  mvlib::setOdom(logger, &chassis);
  logger.setRobot({
    .LeftDrivetrain = mvlib::shared(left_mg),
    .RightDrivetrain = mvlib::shared(right_mg)
  });
  chassis.calibrate();
  logger.start();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  while (true) {
    if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
      left_mg.move(127);
      disp.printToScreen("Set to 127");
    } else if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B)) {
      left_mg.move(-127);
      disp.printToScreen("Set to -127");
    } else if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_X)) {
      left_mg.move(0);
      disp.printToScreen("Set to 0");
    }
    pros::delay(20);
  }
}