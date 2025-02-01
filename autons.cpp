#include "main.h"
// Removed using statements for std::make_tuple, std::tuple, and std::tie
using std::tie;

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void wait() {
  chassis.pid_wait();
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

enum class dir { fwd, rev };

//sets up function for motion chaining, used for chaining multiple motions together, can call function instead of writing everything out
void multPoints(std::tuple<double, double, double, int> point1, std::tuple<double, double, double, int> point2, dir direction) {
  double x, y, theta, x1, y1, theta1;
  int speed, speed1;
  std::tie(x, y, theta, speed) = point1;
  std::tie(x1, y1, theta1, speed1) = point2;
  chassis.pid_odom_set({{{x * 1_in, y * 1_in, theta * 1_deg}, direction == dir::fwd ? ez::fwd : ez::rev, speed},
                        {{x1 * 1_in, y1 * 1_in, theta1 * 1_deg}, direction == dir::fwd ? ez::fwd : ez::rev, speed1}}, true);
}

// sets up function for moving to a pose, used for moving to a specific point on the field at a specific angle
void moveToPose(double X, double Y, double theta, int speed, dir dir = dir::fwd) {
  chassis.pid_odom_set({{X * 1_in, Y * 1_in, theta * 1_deg}, dir == dir::fwd ? ez::fwd : ez::rev}, true);
}

//sets up function for moving to a point, used for moving to a specific point on the field
void moveToPoint(double X, double Y, int speed, dir dir = dir::fwd) {
  chassis.pid_odom_set({{X * 1_in, Y * 1_in}, dir == dir::fwd ? ez::fwd : ez::rev, speed}, true);
}

//sets up function for driving relative to the robot, used for driving a specific distance relative to the robot
void drive(double distance, double speed) {
  chassis.pid_odom_set(distance * 1_in,speed, true);
}

//sets up function for turning to a specific heading, used for turning to a specific angle
void turnToHeading (double theta, int speed) {
  chassis.pid_turn_set(theta * 1_deg, speed);
}

void skillsAuton() {
  //scores on Alliance Stake
  currState = 0;
  target = states[currState];
  hookIntake.move(127);
  pros::delay(500);
  hookIntake.brake();
  //////////////////////////////
  //moves to the first mogo
  moveToPoint(0,11,90);
  wait();
  moveToPoint(29,11,60,dir::rev);
  pros::delay(1173);
  mogoMech.set_value(true);
  pros::delay(200);
  //////////////////////////////
  //moves to score 2 rings
  moveToPoint(25,38,DRIVE_SPEED);
  intakeMove(127);
  wait();
  moveToPoint(56.5,37,110);
  wait();
  moveToPoint(41,37,110,dir::rev);
  wait();
  //////////////////////////////
  //moves to intake Ring and score on Wall Stake
  moveToPoint(41,59.4,100);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  wait();
  turnToHeading(90,TURN_SPEED);
  wait();
  moveToPoint(57.45,59.8,70);
  wait();
  pros::delay(1500);
  intakeBrake();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =2;
  target = states[currState];
  pros::delay(1100);
  currState = 0;
  target = states[currState];
  //////////////////////////////
  //moves to score 2 more rings on Mogo 
  intakeMove(127);
  moveToPoint(46.75,59.4,110,dir::rev);
  wait();
  moveToPoint(46.75,-3,75);
  wait();
  pros::delay(200);
  //////////////////////////////
  //moves to 5th ring on Mogo
  moveToPoint(30,7,80,dir::rev);
  wait();
  moveToPoint(59,7,110);
  wait();
  //////////////////////////////
  //puts mogo in corner
  moveToPoint(55,7,110,dir::rev);
  wait();
  moveToPoint(60,-3,110,dir::rev);
  wait();
  intakeBrake();
  pros::delay(200);
  mogoMech.set_value(false);
  pros::delay(200);
  //////////////////////////////
  //move to second mogo
  moveToPoint(48,5.35,110);
  wait();
  multPoints({-5,5.35,90,110},{-22,5.35,90,65},dir::rev);
  wait();
  mogoMech.set_value(true);
  pros::delay(150);
  //////////////////////////////
  //move to score 2 rings
  moveToPoint(-22,35,110);
  floatingIntake.move(127);
  wait();
  moveToPoint(-55,32,110);
  pros::delay(200);
  intakeMove(127);
  wait();
  //////////////////////////////
  //moves to intake and score on Wall Stake
  moveToPoint(-42.5,32,110,dir::rev);
  wait();
  moveToPoint(-42.5,54.6,110);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  wait();
  turnToHeading(-90,TURN_SPEED);
  wait();
  drive(17.5,70);
  wait();
  pros::delay(1500);
  intakeBrake();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =2;
  target = states[currState];
  pros::delay(1100);
  currState = 0;
  target = states[currState];
  //////////////////////////////
  //moves to score 2 more rings on Mogo
  intakeMove(127);
  moveToPoint(-49.5,54.6,110,dir::rev);
  wait();
  moveToPoint(-49.5,-3.25,75);
  wait();
  pros::delay(300);
  //////////////////////////////
  //moves to 5th ring on Mogo
  moveToPoint(-30,7,80,dir::rev);
  wait();
  moveToPoint(-60.25,7,110);
  wait();
  //////////////////////////////
  //puts mogo in corner
  moveToPoint(-55,7,110,dir::rev);
  wait();
  moveToPoint(-60,-3,110,dir::rev);
  wait();
  intakeBrake();
  pros::delay(200);
  mogoMech.set_value(false);
  pros::delay(200);
  moveToPoint(-46,15,110);
  //////////////////////////////
  //moves to intake two Rings
  multPoints({-46.75, 72, 0, 110}, {-46.75, 82, 0, 75}, dir::fwd);
  floatingIntake.move(127);
  wait();
  drive(-4,110);
  wait();
  turnToHeading(90,90);
  wait();
  floatingIntake.move(127);
  multPoints({-34, 78, 90, 110}, {-22, 78, 90, 65}, dir::fwd);
  hookIntake.move(127);
  pros::delay(300);
  hookIntake.brake();
  wait();
  //////////////////////////////
  //moves to center mogo
  moveToPoint(8, 115, 65, dir::rev);
  pros::delay(2000); //tune this value
  mogoMech.set_value(true);
  pros::delay(200);
  //////////////////////////////
  //moves to score 4 rings
  intakeMove(127);
  moveToPoint(25, 82, 110);
  wait();
  turnToHeading(180,90);
  wait();
  drive(3,110);
  wait();
  moveToPoint(57, 79, 110);
  wait();
  //////////////////////////////
  //moves to score 1 more ring
  moveToPoint(57, 110, 110);
  wait();
  moveToPoint(49, 122, 110);
  wait();
  //////////////////////////////
  //puts mogo in corner
  moveToPoint(58, 123, 110, dir::rev);
  intakeBrake();
  wait();
  mogoMech.set_value(false);
  pros::delay(200);
  //////////////////////////////
  //moves to last mogo
  multPoints({24, 106, -100, 110}, {0, 94, -100, 65}, dir::fwd);
  wait();
}

void rightSoloAWP () {
  //scores on Alliance Stake with Lady Brown
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState =1;
  target = states[currState];
  pros::delay(250);
  hookIntake.move(127);
  pros::delay(500);
  hookIntake.brake();
  pros::delay(10);
  turnToHeading(-135,90);
  wait();
  drive(5,110);
  wait();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =4;
  target = states[currState];
  pros::delay(1000);
  //////////////////////////////
  //moves to mogo
  drive(-10,110);
  pros::delay(100);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(12, 35, 65, dir::rev);
  pros::delay(1150);
  mogoMech.set_value(true);
  intakeMove(127);
  pros::delay(200);
  //////////////////////////////
  //moves to score a Ring
  moveToPoint(37.5,28,110);
  wait();
  //////////////////////////////
  //moves to center Ring, uses Doinker to  pull off Ring
  moveToPoint(1,12,110);
  wait();
  turnToHeading(-90,90);
  wait();
  doinker.set_value(true);
  pros::delay(100);
  moveToPoint(0, 23, DRIVE_SPEED,dir::rev);
  wait();
  doinker.set_value(false);
  pros::delay(250);
  //////////////////////////////
  //scores second Ring and contacts ladder
  moveToPoint(8,8,110);
  wait();
  moveToPoint(0,28,90);
  enableMogoMech = true;
  pros::delay(500);
  currState =2;
  target = states[currState];
  wait();
  //////////////////////////////
}

void rightAutonElim ()  {
   //scores on Alliance Stake with Lady Brown
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState =1;
  target = states[currState];
  pros::delay(250);
  hookIntake.move(127);
  pros::delay(500);
  hookIntake.brake();
  pros::delay(10);
  turnToHeading(-135.1,90);
  wait();
  drive(4.9,110);
  wait();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =4;
  target = states[currState];
  pros::delay(1000);
  //////////////////////////////
  //moves to mogo
  drive(-10,110);
  pros::delay(100);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(12, 35, 65, dir::rev);
  pros::delay(1150);
  mogoMech.set_value(true);
  intakeMove(127);
  pros::delay(200);
  //////////////////////////////
  //moves to score a Ring
  moveToPoint(37,28,110); //37.5 og value
  wait();
  //////////////////////////////
  //moves to center Ring, uses Doinker to  pull off Ring
  moveToPoint(0.75,11.5,110);
  wait();
  turnToHeading(-90,90);
  wait();
  doinker.set_value(true);
  pros::delay(100);
  moveToPoint(0, 23, DRIVE_SPEED,dir::rev);
  wait();
  doinker.set_value(false);
  pros::delay(250);
  //////////////////////////////
  //scores second Ring and contacts ladder
  moveToPoint(12,4,110);
  wait();
  hookIntake.move(127);
  moveToPoint(36,5,110);
  pros::delay(250);
  floatingIntake.move(-127);
  wait();
  turnToHeading(145,90);
  wait();
  enableMogoMech = true;
  doinker.set_value(true);
  pros::delay(200);
  drive(11,110);
  wait();
  turnToHeading(200,85);
  doinker.set_value(false);
  wait();
  turnToHeading(0,110);
  wait();
  moveToPoint(40,-5,110,dir::rev);
  wait();
  //////////////////////////////
}

void leftSoloAWP () {
  //scores on Alliance Stake with Lady Brown
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState =1;
  target = states[currState];
  pros::delay(300);
  hookIntake.move(127);
  pros::delay(500);
  hookIntake.brake();
  pros::delay(10);
  turnToHeading(140,90);
  wait();
  drive(4.25,110);
  wait();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =4;
  target = states[currState];
  pros::delay(1100);
  //////////////////////////////
  //moves to mogo
  drive(-10,110);
  pros::delay(100);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(-12, 35, 65, dir::rev);
  pros::delay(1150);
  mogoMech.set_value(true);
  intakeMove(127);
  pros::delay(200);
  //////////////////////////////
  //moves to score a Ring
  moveToPoint(-36,30,110);
  wait();
  //////////////////////////////
  //moves to center Ring, uses Doinker to  pull off Ring
  moveToPoint(-3,-4,110); // change if needed
  wait();
  turnToHeading(90,90);
  wait();
  doinker.set_value(true);
  pros::delay(200);
  moveToPoint(-8, 20, DRIVE_SPEED,dir::rev);
  wait();
  doinker.set_value(false);
  pros::delay(250);
  //////////////////////////////
  //scores second Ring and contacts ladder
  moveToPoint(2,-2,110);
  wait();
  moveToPoint(0,30,110);
  enableMogoMech = true;
  wait();
  currState =2;
  target = states[currState];
  //////////////////////////////
}

void ringRushLeft_soloAWP () {
  //rushes center Rings, uses floating intake and Ring Rush
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(16,70);
  wait();
  turnToHeading(-24,90);
  wait();
  drive(43.25,110);
  floatingIntake.move(127);
  ringRush.set_value(true);
  wait();
  multPoints({-16.0, 35.967, 0.0, 100}, {-16.0, 28.0, 0.0, 110}, dir::rev);
  wait();
  ringRush.set_value(false);
  pros::delay(10);
  //////////////////////////////
  //moves to mogo
  drive(-5,110);
  wait();
  turnToHeading(-135,90);
  wait();
  drive(-30,85);
  pros::delay(865);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 2 rings
  moveToPoint(-21.5,41,110);
  pros::delay(200);
  intakeMove(127);
  wait();
  //////////////////////////////
  //moves to score thrid ring
  moveToPoint(-24,12,110);
  wait();
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  //////////////////////////////
  //intake ring for Alliance Stake, scores using Lady Brown
  moveToPoint(12, 0, 110);
  wait();
  moveToPoint(15,-5,110);
  wait();
  turnToHeading(139,90);
  wait();
  intakeBrake();
  floatingIntake.move(127);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState = 4;
  target = states[currState];
  drive(-0.75,110);
  wait();
  pros::delay(825);
  //////////////////////////////
  //moves to contact ladder
  intakeMove(127);
  drive(-15,110);
  wait();
  enableMogoMech = true;
  moveToPoint(12,33,110);
  currState = 2;
  target = states[currState];
  wait();
  //////////////////////////////
}

void ringRushRight_soloAWP () {
  //rushes center Rings, uses floating intake and Ring Rush
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(16,70);//12
  wait();
  turnToHeading(33,90);
  wait();
  drive(45.5,110);//old speed 85, old distance 46.5 (crosses line), worked at 43
  floatingIntake.move(127);
  ringRush.set_value(true);
  wait();
  multPoints({23.5, 35.967, 0.0, 110}, {23.5, 27.0, 0.0, 110}, dir::rev);// old speed 65, 90
  wait();
  ringRush.set_value(false);
  //////////////////////////////
  //moves to mogo
  drive(-7,110);
  wait();
  turnToHeading(132,90);
  wait();
  drive(-35,85); //old speed 70
  pros::delay(855);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 3 rings
  moveToPoint(30,44,110);
  pros::delay(200);
  intakeMove(127);
  wait();
  //////////////////////////////
  //intakes ring for Alliance Stake, scores using Lady Brown
  moveToPoint(-6, 0, 110);
  pros::delay(250);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  wait();
  moveToPoint(-12,-2.25,110);
  wait();
  turnToHeading(-145,90);
  wait();
  intakeBrake();
  floatingIntake.move(127);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState = 4;
  target = states[currState];
  wait();
  pros::delay(825);
  intakeMove(127);
  //////////////////////////////
  //moves to contact ladder
  drive(-15,110);
  wait();
  enableMogoMech = true;
  moveToPoint(-9,33,110);
  currState = 2;
  target = states[currState];
  wait(); 
  //////////////////////////////
}

void ringRushLeft_ElimAuton () {
  //rushes center Rings, uses floating intake and Ring Rush
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(16,70);//12
  wait();
  turnToHeading(-24,90);
  wait();
  drive(43,110);//old speed 85, old distance 46.5 (crosses line), worked at 44
  floatingIntake.move(127);
  ringRush.set_value(true);
  wait();
  multPoints({-16.0, 35.967, 0.0, 110}, {-16.0, 28.0, 0.0, 110}, dir::rev);// old speed 65, 90
  wait();
  ringRush.set_value(false);
  pros::delay(50);
  //////////////////////////////
  //moves to mogo
  drive(-5,110);
  wait();
  turnToHeading(-135,90);
  wait();
  drive(-30,85); //old speed 70
  pros::delay(865);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 2 rings
  moveToPoint(-21.5,41,110);
  pros::delay(200);
  intakeMove(127);
  wait();
  //////////////////////////////
  //moves to score thrid ring
  moveToPoint(-26,12,110);
  wait();
  //////////////////////////////
  //intake ring for Alliance Stake, scores using Lady Brown
  moveToPoint(12, 0, 110);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  wait();
  moveToPoint(15,-5,110);
  wait();
  turnToHeading(139,90);
  wait();
  intakeBrake();
  floatingIntake.move(127);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState = 4;
  target = states[currState];
  drive(-0.5,110);
  wait();
  pros::delay(825);
  //////////////////////////////
  //moves to contact ladder
  hookIntake.move(127);
  floatingIntake.move(-127);
  drive(-5,110);
  wait();
  enableMogoMech = true;
  moveToPoint(124,3,110);
  currState =0;
  target = states[currState];
  wait();
  //////////////////////////////
}

void ringRushRight_ElimAuton () {
  //rushes center Rings, uses floating intake and Ring Rush
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(16,70);//12
  wait();
  turnToHeading(33,90);
  wait();
  drive(45.5,110);//old speed 85, old distance 46.5 (crosses line), worked at 43
  floatingIntake.move(127);
  ringRush.set_value(true);
  wait();
  multPoints({23.5, 35.967, 0.0, 110}, {23.5, 27.0, 0.0, 110}, dir::rev);// old speed 65, 90
  wait();
  ringRush.set_value(false);
  //////////////////////////////
  //moves to mogo
  drive(-7,110);
  wait();
  turnToHeading(132,90);
  wait();
  drive(-35,85); //old speed 70
  pros::delay(855);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 3 rings
  moveToPoint(30,44,110);
  pros::delay(200);
  intakeMove(127);
  wait();
  //////////////////////////////
  //intakes ring for Alliance Stake, scores using Lady Brown
  moveToPoint(-6, 0, 110);
  pros::delay(250);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState = 1;
  target = states[currState];
  wait();
  moveToPoint(-12,-2.25,110);
  wait();
  turnToHeading(-145,90);
  wait();
  intakeBrake();
  floatingIntake.move(127);
  hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState = 4;
  target = states[currState];
  wait();
  pros::delay(825);
  hookIntake.move(127);
  floatingIntake.move(-127);
  //////////////////////////////
  //moves to contact ladder
  drive(-5,110);
  wait();
  enableMogoMech = true;
  moveToPoint(-124,33,110);
  currState = 0;
  target = states[currState];
  wait(); 
  //////////////////////////////
}

void rightGoalRush_soloAWP () {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  intakeMove(127);
  moveToPoint(5,47,110);
  wait();
  doinker.set_value(true);
  intakeBrake();
  pros::delay(350);
  moveToPoint(0,20,110,dir::rev);
  wait();
  doinker.set_value(false);
  pros::delay(200);
  turnToHeading(160,90);
  wait();
  drive(-20,60);
  wait();
  mogoMech.set_value(true);
  pros::delay(200);
  intakeMove(127);
  turnToHeading(-15,90);
  wait();
  drive(-15,110);
  wait();
  mogoMech.set_value(false);
  pros::delay(200);
}
 
void leftGoalRush_soloAWP () {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  moveToPoint(0, 17.54, DRIVE_SPEED);
  wait();
  floatingIntake.move(127);
  chassis.pid_odom_set({{{-24.507, 25.95}, fwd, DRIVE_SPEED},
                        {{-21.624, 44.69}, fwd, DRIVE_SPEED}});
  floatingIntake.brake();
  wait();
  doinker.set_value(true);
  pros::delay(250);
  moveToPoint(-21.624, 27.871, DRIVE_SPEED, dir::rev);
  wait();
  doinker.set_value(false);
  moveToPoint(-21.624, 40.969, DRIVE_SPEED, dir::rev);
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(-36.389, 9.37, DRIVE_SPEED, dir::rev);
  intakeMove(127);
  pros::delay(1000);
  intakeBrake();
  mogoMech.set_value(false);
  pros::delay(100);
  chassis.pid_odom_set({{{-36.389, 34.167, 0}, fwd, DRIVE_SPEED},
                        {{-5.046, 38.683, 90}, rev, DRIVE_SPEED}});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(24, 14.416, DRIVE_SPEED);
  intakeMove(127);
  wait();
  intakeBrake();
  moveToPoint(24, 24, DRIVE_SPEED);
}