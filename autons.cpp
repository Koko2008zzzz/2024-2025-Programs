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
  chassis.pid_drive_constants_set(16.5, 7.5, 100.0);  // Sets forward and backward
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.5, .05, 20.0, 15.0);     // Turn in place constants
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
  chassis.slew_drive_constants_forward_set(7_in, 40);
  chassis.slew_drive_constants_backward_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}


enum class dir { fwd, rev };
void wait() {
  chassis.pid_wait();
}

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
  chassis.pid_odom_set(distance * 1_in, speed, true);
}

//sets up function for turning to a specific heading, used for turning to a specific angle
void turnToHeading (double theta, int speed) {
  chassis.pid_turn_set(theta * 1_deg, speed);
}


void skillsAuton () {
  alliance =2;
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  //scores on Alliance Stake
  intakeMove(127);
  pros::delay(250);
  intakeMove(0);
  //////////////////////////////
  //moves to the first mogo
  moveToPoint(0,12,110);
  wait();
  turnToHeading(-90,90);
  pros::delay(500);
  moveToPoint(29,12,85,dir::rev);
  pros::delay(790);//795
  mogoMech.set_value(true);
  pros::delay(150);
  //////////////////////////////
  //moves to score 2 rings
  moveToPoint(22,35,110);
  intakeMove(127);
  wait();
  moveToPoint(56,35,110);
  wait();
  moveToPoint(41,35,110,dir::rev);
  pros::delay(500);
  currState = 1;
  target = states[currState];
  wait();
  //////////////////////////////
  //moves to intake Ring and score on Wall Stake
  moveToPoint(58,61,110);
  wait();
  //////
  moveToPoint(50, 79, 110);
  pros::delay(775);
  intakeMove(0);
  currState =2;
  target = states[currState];
  pros::delay(225);
  intakeMove(127);
  wait();
  //////
  moveToPoint(54.5, 63, 110, dir::rev); //62.8
  wait();
  turnToHeading(89, 90);
  wait();
  drive(9.25,110); //4.125
  wait();
  intakeMove(0);
  currState = 3;
  target = states[currState];
  pros::delay(800);
  //////////////////////////////
  //moves to score 2 more rings on Mogo 
  moveToPoint(46.75,63,110,dir::rev);
  intakeMove(127);
  wait();
  moveToPoint(46.75, 3, 75);
  currState= 0;
  target = states[currState];
  wait();
  //////////////////////////////
  //moves to 5th ring on Mogo
  moveToPoint(35,10,80,dir::rev);
  wait();
  moveToPoint(57.5,10,110);
  wait();
  //////////////////////////////
  //puts mogo in corner
  moveToPoint(55,9,110,dir::rev);
  wait();
  moveToPoint(60,0,110,dir::rev);
  pros::delay(750);
  intakeMove(-127);
  mogoMech.set_value(false);
  pros::delay(150);
  //////////////////////////////
  //moves to intake two Rings
multPoints({47.5, 93, 0, 110}, 
             {47.5, 105, 0, 75}, dir::fwd);
  intakeMove(127);
  wait();
  //////////////////////////////
  //move to 2nd mogo & puts mogo in corner
  moveToPoint(18, 119, 65, dir::rev);
  pros::delay(110);
  intakeMove(0);
  pros::delay(1805-150);
  mogoMech.set_value(true);
  turnToHeading(-100,90);
  currState =1;
  target = states[currState];
  pros::delay(500);
  drive(-44,127); //-37
  intakeMove(127);
  mogoMech.set_value(false);
  wait();
  //////////////////////////////
  //move to Center Mogo
  moveToPoint(25, 111.5, 80);
  wait();
  turnToHeading(90,90);
  wait();
  moveToPoint(-5, 111.5, 75, dir::rev);
  pros::delay(765);//755
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //Scores on Alliacne Stake
  moveToPoint(8, 113, 110);
  wait();
  turnToHeading(0.25, 90);
  wait();
  drive(3.5, 110);
  wait();
  intakeMove(0);
  currState =6;
  target = states[currState];
  pros::delay(750);
  //////////////////////////////
  //moves to intake two Rings
  drive(-6, 110);
  wait();
  moveToPoint(33, 88.5, 110);
  currState =0;
  target = states[currState];
  intakeMove(127);
  wait();
  pros::delay(500);
  //////////////////////////////
  //moves through center & scores 2 Rings
  moveToPoint(-13, 39, 110);//change this
  pros::delay(500);
  intakeMove(0);
  wait();
  intakeMove(127);
  //////////////////////////////
  //moves to score two Rings
  drive(32,90);
  wait();
  moveToPoint(-35, 2, 110);
  wait();
  moveToPoint(-30, 12, 110, dir::rev);
  wait();
  moveToPoint(-48, 12, 110);
  wait();
  //////////////////////////////
  //puts Mogo in corner
  moveToPoint(-52, 3, 110, dir::rev);
  wait();
  mogoMech.set_value(false);
  pros::delay(100);
  intakeMove(0);
  //////////////////////////////
  //moves to clamp Mogo
  drive(7, 110);
  wait();
  moveToPoint(5, 12, 110, dir::rev);
  pros::delay(1300);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 2 Rings
  moveToPoint(-48, 38, 110);
  intakeMove(127);
  wait();
  moveToPoint(-50, 63, 90);
  pros::delay(500);
  currState = 1;
  target = states[currState];
  wait();
  moveToPoint(-48, 58, 110, dir::rev);
  wait();
  turnToHeading(-91, 90);
  wait();
  drive(10.5, 80);
  wait();
  intakeMove(0);
  currState = 3;
  target = states[currState];
  pros::delay(800);
  moveToPoint(-36, 58, 110, dir::rev);
  intakeMove(127);
  pros::delay(500);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(-36, 73, 110);
  wait();
  moveToPoint(-12, 73, 110);
  wait();
  moveToPoint(-24, 84, 110);
  wait();
  //////////////////////////////
}


void soloAWPState() {
  //scores on Alliance Stake with Lady Brown
  alliance =1;
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState =1;
  target = states[currState];
  pros::delay(260);
  intakeMove(127);
  pros::delay(500);
  intakeMove(0);
  pros::delay(10);
  turnToHeading(-141,90);
  wait();
  drive(3.8,110);
  wait();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeMove(-20);
  currState =6;
  target = states[currState];
  pros::delay(650);
  //////////////////////////////
  //moves to mogo
  drive(-10,110);
  pros::delay(100);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(16, 37, 80, dir::rev);
  pros::delay(900);
  mogoMech.set_value(true);
  intakeMove(127);
  pros::delay(150);
  //////////////////////////////
  //moves to score a Ring
  moveToPoint(32.5,28,120); //37.5 og value
  wait();
  //////////////////////////////
  //moves to center Ring, intakes bottom Ring and flings it out, keeps second Ring
  moveToPoint(0,7,120);
  wait();
  moveToPoint(-35,7,110);
  mogoMech.set_value(false);
  wait();
  intakeMove(60);
  //////////////////////////////
  //moves to second mogo
  turnToHeading(-185,95);
  pros::delay(250);
  intakeMove(0);
  wait();
  drive(-29,80);
  pros::delay(700);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 2 rings
  moveToPoint(-55, 28, 110);
  pros::delay(200);
  intakeMove(127);
  enableMogoMech = true;
  wait();
  //////////////////////////////
  //moves to contact ladder
  turnToHeading(90,90);
  wait();
  drive(37,110);
  currState =3;
  target = states[currState];
  alliance =0;
  wait();
  intakeMove(0); 
}


void rightAutonSixRing () {
  //scores on Alliance Stake with Lady Brown
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  currState =1;
  target = states[currState];
  pros::delay(260);
  intakeMove(127);
  pros::delay(500);
  intakeMove(0);
  pros::delay(10);
  turnToHeading(-141,90);
  wait();
  drive(3.8,110);
  wait();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  currState =6;
  target = states[currState];
  pros::delay(640);
  //////////////////////////////
  //moves to mogo
  drive(-10,110);
  pros::delay(300);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(16, 37, 65, dir::rev);
  pros::delay(1150);
  mogoMech.set_value(true);
  pros::delay(75); //100
  /////////////////////////////
  //moves to grab both center Rings
  moveToPoint(-5.5,42.75,110);
  intakeMove(-5);
  wait();
  turnToHeading(-20, 90);
  wait();
  leftDoinker.set_value(true);
  pros::delay(100); //100
  turnToHeading(-41.45, 110);
  pros::delay(300);
  rightDoinker.set_value(true);
  pros::delay(75); //100
  //////////////////////////////
  //lines up all three Rings
  drive(-22,110); //24.5
  enableMogoMech = true;
  wait();
  turnToHeading(0, 110);
  pros::delay(500);
  moveToPoint(12, 3, 127, dir::rev);
  pros::delay(575);
  rightDoinker.set_value(false);
  pros::delay(200);
  leftDoinker.set_value(false);
  pros::delay(100);
  //////////////////////////////
  //scores 3 Rings on Mogo
  moveToPoint(7,19, 110);
  intakeMove(127);
  pros::delay(625);
  chassis.pid_swing_set(LEFT_SWING, 90, 110);
  drive(23,110);
  wait();
  //////////////////////////////
  //moves to corner & clear corner
  //////////////////////////////
}


void ringRushRight_fourRingMogo () {
  //scores on Alliance Stake with Lady Brown
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(47,110);
  rightDoinker.set_value(true);
  intakeMove(127);
  wait();
  moveToPoint(-21, 22, 110, dir::rev);
  pros::delay(320);//350
  intakeMove(0);
  pros::delay(800);//765,775
  mogoMech.set_value(true);
  rightDoinker.set_value(false);
  pros::delay(120);
  multPoints({0, 31, 90, 110}, 
             {10, 30, 115, 110}, dir::fwd);
  pros::delay(150);
  intakeMove(127);
  wait();
  moveToPoint(18,6,110);
  wait();
  moveToPoint(35,2,110);
  wait();
  drive(5,110);
  pros::delay(500);
  drive(-7,80);
  intakeLift.set_value(true);
  wait();
  drive(5,110);
  pros::delay(500);
  drive(-9,110);
  pros::delay(250);
  intakeLift.set_value(false);
  wait();
  moveToPoint(-10, -14, 110);
  pros::delay(300);
  currState =1;
  target = states[currState];
  pros::delay(1250);
  currState =2;
  target = states[currState];
  pros::delay(150);
  intakeLift.set_value(true);
  wait();
}

