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
  intakeMove(110);
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
  pros::delay(785);
  intakeMove(0);
  currState =2;
  target = states[currState];
  pros::delay(215);
  intakeMove(110);
  wait();
  //////
  moveToPoint(54.5, 63.1, 110, dir::rev); //62.8
  wait();
  turnToHeading(89, 90);
  wait();
  drive(4,110); //4.125
  wait();
  intakeMove(0);
  currState = 3;
  target = states[currState];
  pros::delay(800);
  //////////////////////////////
  //moves to score 2 more rings on Mogo 
  moveToPoint(46.75,63.1, 110, dir::rev);
  intakeMove(110);
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
  intakeMove(-110);
  mogoMech.set_value(false);
  pros::delay(150);
  //////////////////////////////
  //moves to intake two Rings
  multPoints({48.5, 98, 0, 110}, 
             {48.5, 108, 0, 75}, dir::fwd);
  intakeMove(127);
  pros::delay(2750);
  intakeMove(0);
  wait();
  //////////////////////////////
  //move to 2nd mogo & puts mogo in corner
  moveToPoint(17, 118, 65, dir::rev);
  pros::delay(1700);//1780
  mogoMech.set_value(true);
  turnToHeading(-220, 90);
  pros::delay(300);
  drive(30, 110);
  wait();
  turnToHeading(-130,90);
  currState =1;
  target = states[currState];
  pros::delay(600);
  drive(-28,127); //-37
  intakeMove(127);
  mogoMech.set_value(false);
  wait();
  //////////////////////////////
  //move to Center Mogo
  moveToPoint(25, 108, 80);
  wait();
  turnToHeading(90,90);
  wait();
  moveToPoint(-7, 108, 75, dir::rev);
  pros::delay(765);//755
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //Scores on Alliacne Stake
  moveToPoint(7, 108, 110);
  wait();
  turnToHeading(0, 90);
  wait();
  drive(4, 110);
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
  moveToPoint(-17, 39, 90);//change this
  pros::delay(500);
  intakeMove(0);
  wait();
  intakeMove(127);
  //////////////////////////////
  //moves to score two Rings
  drive(32,90);
  wait();
  moveToPoint(-42, 2, 110);
  wait();
  moveToPoint(-35, 12, 110, dir::rev);
  wait();
  moveToPoint(-48, 12, 110);
  wait();
  //////////////////////////////
  //puts Mogo in corner
  moveToPoint(-52, 0, 110, dir::rev);
  wait();
  mogoMech.set_value(false);
  pros::delay(100);
  intakeMove(0);
  //////////////////////////////
  //moves to clamp Mogo
  drive(7, 110);
  wait();
  moveToPoint(6, 6, 110, dir::rev);
  pros::delay(1320);
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 2 Rings
  moveToPoint(-48, 38, 110);
  intakeMove(127);
  wait();
  moveToPoint(-50, 66, 90);
  pros::delay(500);
  currState = 1;
  target = states[currState];
  wait();
  moveToPoint(-48, 60, 110, dir::rev);
  wait();
  turnToHeading(-91, 90);
  wait();
  drive(10.5, 80);
  wait();
  intakeMove(0);
  currState = 3;
  target = states[currState];
  pros::delay(800);
  moveToPoint(-36, 60, 110, dir::rev);
  intakeMove(127);
  pros::delay(500);
  currState =0;
  target = states[currState];
  wait();
  moveToPoint(-36, 76, 110);
  wait();
  moveToPoint(-12, 76, 110);
  wait();
  moveToPoint(-24, 84, 110);
  wait();
  //////////////////////////////
}


void soloAWP() {
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
  //////////////////////////////
}


void blueNegRush61Elim () {
  //sets color sorting to sort red rings
  alliance =1;
  //////////////////////////////
  //rushes the ring rush
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  drive(46.5,110);//47.5
  rightDoinker.set_value(true);
  intakeMove(127);
  wait();
  //////////////////////////////
  //moves to mogo and clamps it
  moveToPoint(-17, 16.5, 110, dir::rev);
  pros::delay(165);
  intakeMove(0);
  pros::delay(500);//800
  mogoMech.set_value(true);
  pros::delay(100);
  //////////////////////////////
  //moves to score 3 Rings
  multPoints({0, 31, 90, 110}, 
             {10, 30, 115, 110}, dir::fwd);
             rightDoinker.set_value(false);
  pros::delay(150);
  intakeMove(127);
  wait();
  //////////////////////////////
  //moves to score preLoad and towards corner
  /*moveToPoint(18,8,110);//18, 8
  wait();
  moveToPoint(37.5,4.25,110);
  wait();
  //////////////////////////////
  //moves to score 1 corner rings
  drive(8,90);//6
  currState =4;
  target = states[currState];
  pros::delay(1000);
  drive(-8,110);//-6
  //////////////////////////////
  //moves towards middle stack of Rings
  moveToPoint(-13, -13, 110);
  pros::delay(1000);
  currState =1;
  target = states[currState];
  intakeLift.set_value(true);
  enableMogoMech = true;
  pros::delay(1000);//tune this
  intakeLift.set_value(false);//tune
  antiJam = false;
  wait();
  leftDoinker.set_value(true);
  //////////////////////////////
  //scores Ring on Alliance Stake
  turnToHeading(-208, 90);
  pros::delay(500);
  leftDoinker.set_value(false);
  wait();
  drive(4, 110);//6.35
  wait();
  pros::delay(350);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeMove(-20);
  currState =6;
  target = states[currState];
  pros::delay(500); //700
  //////////////////////////////
  //moves towards corner
  drive(-13, 110);
  wait();
  currState =0;
  target = states[currState];*/
  //////////////////////////////
}


void redNegRush61Elim () {
    //sets color sorting to sort red rings
    alliance =2;
    //////////////////////////////
    //rushes the ring rush
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    drive(47.5,110);//47.5
    leftDoinker.set_value(true);
    intakeMove(127);
    wait();
    //////////////////////////////
    //moves to mogo and clamps it
    moveToPoint(16.5, 21, 110, dir::rev);
    pros::delay(165);// 185
    intakeMove(0);
    pros::delay(920);//900
    mogoMech.set_value(true);
    leftDoinker.set_value(false);
    pros::delay(100);
    //////////////////////////////
    //moves to score 3 Rings
    multPoints({0, 31, -90, 110}, 
               {-10, 30, -115, 110}, dir::fwd);
    pros::delay(150);
    intakeMove(127);
    wait();
    //////////////////////////////
    //moves to score preLoad and towards corner
    moveToPoint(-18,8,110);//18, 8
    wait();
    moveToPoint(-37.5,4.25,110);
    wait();
    //////////////////////////////
    //moves to score 1 corner rings
    drive(7,90);//6
    currState =4;
    target = states[currState];
    pros::delay(800);
    drive(-7,110);//-6
    //////////////////////////////
    //moves towards middle stack of Rings
    moveToPoint(13.5, -12, 110);
    pros::delay(1000);
    currState =1;
    target = states[currState];
    intakeLift.set_value(true);
    enableMogoMech = true;
    pros::delay(1200);//tune this, 1000
    intakeLift.set_value(false);//tune
    wait();
    rightDoinker.set_value(true);
    //////////////////////////////
    //scores Ring on Alliance Stake
    turnToHeading(204, 90);
    pros::delay(500);
    rightDoinker.set_value(false);
    wait();
    drive(3.5, 110);//4.9
    wait();
    pros::delay(350);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeMove(-20);
    currState =6;
    target = states[currState];
    pros::delay(500); //700
    //////////////////////////////
    //moves towards corner
    drive(-13, 110);
    wait();
    currState =0;
    target = states[currState];
    //////////////////////////////
}

//fully tuned
void blueNeg61Qual () {
  chassis.odom_xyt_set(0,0, -118);
  //scores on Alliance Stake with Lady Brown
  currState =5;
  alliance = 1;
  enableMogoMech = true;
  target = states[currState];
  pros::delay(300);
  //////////////////////////////
  //moves towards Mobile Goal
  drive(-5, 110);
  pros::delay(500);
  moveToPoint(14.25, 37, 110, dir::rev);
  pros::delay(1050);
  mogoMech.set_value(true);
  currState = 0;
  target = states[currState];
  pros::delay(100);
  /////////////////////////////
  //moves to score 2 rings
  intakeMove(565);
  moveToPoint(29, 48.5, 110);
  pros::delay(1225);
  moveToPoint(45, 48.5, 110);
  pros::delay(720);
  ////////////////////////////
  //moves backwards to score Ring
  moveToPoint(18, 33, 110, dir::rev);
  wait();
  moveToPoint(30, 31.25, 110);
  wait();
  ////////////////////////////
  //lines up with Corner Stack
  drive(-10, 110);
  pros::delay(500);
  turnToHeading(141.5, 90);
  currState = 4;
  target = states[currState];
  pros::delay(400);
  drive(64, 127);
  pros::delay(1759);
  drive(-17, 110);
  currState = 0;
  target = states[currState];
  wait();
  drive(12, 110);
  pros::delay(900);
  ////////////////////////////
  //moves to score 6th Ring
  moveToPoint(-10, 13, 127);
  pros::delay(750);
  intakeLift.set_value(true);
  pros::delay(1350);
  intakeLift.set_value(false);
  wait();
  ////////////////////////////
  //moves to contact ladder
  moveToPoint(-12, 25, 110);
  pros::delay(250);
  intakeLift.set_value(true);
  wait();
  ////////////////////////////
}


void redNeg61Qual () {
  chassis.odom_xyt_set(0,0, 118);
  //scores on Alliance Stake with Lady Brown
  currState =5;
  alliance = 1;
  enableMogoMech = true;
  target = states[currState];
  pros::delay(300);
  drive(-5, 110);
  pros::delay(500); //tune this
  moveToPoint(-14.25, 37, 110, dir::rev);
  pros::delay(1050); //930
  mogoMech.set_value(true);
  currState = 0;
  target = states[currState];
  pros::delay(100);
  intakeMove(565);
  moveToPoint(-29, 48.5, 110);
  pros::delay(1225);
  moveToPoint(-45, 48.5, 110);
  pros::delay(720);//tune this
 // moveToPoint(29, 48, 110, dir::rev);
  //pros::delay(500);
  moveToPoint(-18, 33, 110, dir::rev);
  wait();
  moveToPoint(-30, 31.25, 110);
  wait();
  drive(-10, 110);
  pros::delay(500);
  turnToHeading(-141.5, 90);//155
  currState = 4;
  target = states[currState];
  pros::delay(400);//tune this
  drive(64, 127);
  pros::delay(1759);
  drive(-17, 110);
  currState = 0;
  target = states[currState];
  wait();
  drive(12, 110);
  pros::delay(900);
  moveToPoint(10, 13, 127);
  pros::delay(750);
  intakeLift.set_value(true);
  pros::delay(1350);
  intakeLift.set_value(false);
  wait();
  moveToPoint(12, 25, 110);
  pros::delay(250);
  intakeLift.set_value(true);
  wait();
}

//fully tuned
void redPos41Qual () {
  chassis.odom_xyt_set(0,0, -118);
  //scores on Alliance Stake with Lady Brown
  currState =5;
  alliance = 2;
  target = states[currState];
  pros::delay(400);
  //////////////////////////////
  //moves towards Mobile Goal
  drive(-5, 110);
  wait(); 
  moveToPoint(14.98, 39, 110, dir::rev);
  pros::delay(1015); //1050
  mogoMech.set_value(true);
  currState = 0;
  target = states[currState];
  pros::delay(100);
  /////////////////////////////
  //moves to score Ring
  moveToPoint(34.5, 35, 110);
  intakeMove(550);
  wait();
  ////////////////////////////
  //moves towards Corner
  moveToPoint(34.5, 10, 110);
  wait();
  turnToHeading(134, 90);
  pros::delay(50);
  currState = 4;
  target = states[currState];
  intakeMove(575);
  wait();
  ////////////////////////////
  //moves to score 2 Corner Rings
  drive(45, 127);
  pros::delay(1400);
  drive(-17, 110);
  wait();
  drive(16, 110);
  wait(); 
  ///////////////////////////
  //moves towards Ladder
  drive(-24, 110);
  wait();
  turnToHeading(-42, 90);
  pros::delay(300);
  moveToPoint(15.75, 31, 110);
  currState = 0;
  target = states[currState];
  pros::delay(800);
  drive(35, 110);
  intakeMove(0);
  rightDoinker.set_value(true);
  wait();
  //////////////////////////
  //moves to contact ladder & score Ring
  drive(-35, 110);
  enableMogoMech = true;
  wait();
  rightDoinker.set_value(false);
  moveToPoint(17, 42, 110);
  intakeMove(550);
  pros::delay(750);
  currState =3;
  target = states[currState];
  wait();
  //////////////////////////
}

//fully tuned
void redPos6Elim () {
  chassis.odom_xyt_set(0, 0, 180);
  enableMogoMech = false;
  alliance =2; 
  moveToPoint(0, 32, 110, dir::rev);
  pros::delay(700);
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(575);
  moveToPoint(-7, 32, 110);
  pros::delay(770);
  intakeMove(0);
  wait();
  turnToHeading(-28, 90);
  pros::delay(400);//tune this
  drive(11, 110);
  pros::delay(435);//tune this
  leftDoinker.set_value(true);
  pros::delay(250);
  chassis.pid_turn_constants_set(5.5, .05, 20.0, 15.0); 
  turnToHeading(-43, 90);
  pros::delay(225);
  chassis.pid_turn_constants_set(3.5, .05, 20.0, 15.0); 
  rightDoinker.set_value(true);
  pros::delay(250);
  moveToPoint(3, 11, 110, dir::rev);
  wait();
  leftDoinker.set_value(false);
  rightDoinker.set_value(false);
  pros::delay(200);
  intakeMove(400);
  moveToPoint(-1.0, 13, 110);//-3
  wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 110);
  pros::delay(750);
  intakeMove(575);
  moveToPoint(32, 25, 110);
  wait();
  moveToPoint(32, 1.15, 110);
  wait();
  turnToHeading(135.5, 90);
  pros::delay(50);
  currState = 4;
  target = states[currState];
  wait();
  drive(43, 127);
  pros::delay(1350);
  drive(-17, 127);
  pros::delay(700);
  drive(18, 127);
  pros::delay(700);
  drive(-17, 127);
  wait();
  moveToPoint(48, -28, 127, dir::rev);
  pros::delay(750);
  mogoMech.set_value(false);
  wait();
}


void bluePos41Qual () {
  chassis.odom_xyt_set(0,0, 118);
  //scores on Alliance Stake with Lady Brown
  currState =5;
  alliance = 2;
  target = states[currState];
  pros::delay(400);
  drive(-5, 110);
  wait(); 
  moveToPoint(-14.98, 39, 110, dir::rev);
  pros::delay(1015); //1050
  mogoMech.set_value(true);
  currState = 0;
  target = states[currState];
  pros::delay(100);
  moveToPoint(-34.5, 35, 110);
  intakeMove(550);
  wait();
  moveToPoint(-34.5, 10, 110);
  wait();
  turnToHeading(-134, 90);
  pros::delay(50);
  currState = 4;
  target = states[currState];
  intakeMove(575);
  wait();
  drive(45, 127);
  pros::delay(1400);
  drive(-17, 110);
  wait();
  drive(16, 110);
  wait(); 
  drive(-24, 110);
  wait();
  turnToHeading(42, 90);
  pros::delay(300);
  moveToPoint(-15.75, 31, 110);
  currState = 0;
  target = states[currState];
  pros::delay(800);
  drive(35, 110);
  intakeMove(0);
  rightDoinker.set_value(true);
  wait();
  drive(-35, 110);
  enableMogoMech = true;
  wait();
  rightDoinker.set_value(false);
  moveToPoint(-17, 42, 110);
  intakeMove(550);
  pros::delay(750);
  currState =3;
  target = states[currState];
  wait();
}


void bluePos6Elim () {
  chassis.odom_xyt_set(0, 0, 180);
  enableMogoMech = false;
  alliance =2; 
  moveToPoint(0, 32, 110, dir::rev);
  pros::delay(700);
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(575);
  moveToPoint(7, 32, 110);
  pros::delay(770);
  intakeMove(0);
  wait();
  turnToHeading(28, 90);
  pros::delay(400);//tune this
  drive(11, 110);
  pros::delay(435);//tune this
  leftDoinker.set_value(true);
  pros::delay(250);
  chassis.pid_turn_constants_set(5.5, .05, 20.0, 15.0); 
  turnToHeading(43, 90);
  pros::delay(225);
  chassis.pid_turn_constants_set(3.5, .05, 20.0, 15.0); 
  rightDoinker.set_value(true);
  pros::delay(250);
  moveToPoint(-3, 11, 110, dir::rev);
  wait();
  leftDoinker.set_value(false);
  rightDoinker.set_value(false);
  pros::delay(200);
  intakeMove(400);
  moveToPoint(1.0, 13, 110);//-3
  wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, -90, 110);
  pros::delay(750);
  intakeMove(575);
  moveToPoint(-32, 25, 110);
  wait();
  moveToPoint(-32, 1.15, 110);
  wait();
  turnToHeading(-135.5, 90);
  pros::delay(50);
  currState = 4;
  target = states[currState];
  wait();
  drive(43, 127);
  pros::delay(1350);
  drive(-17, 127);
  pros::delay(700);
  drive(18, 127);
  pros::delay(700);
  drive(-17, 127);
  wait();
  moveToPoint(-48, -28, 127, dir::rev);
  pros::delay(750);
  mogoMech.set_value(false);
  wait();
}
