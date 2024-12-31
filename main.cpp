#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"


pros::MotorGroup left_motors({-9, -8, -10},pros::MotorGear::blue);
pros::MotorGroup right_motors({18, 19, 20},pros::MotorGear::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.565,
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2 (for now)
);
pros::Imu imu(4);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Controller master(pros::E_CONTROLLER_MASTER);

//Motor for subsystems
pros::Motor hookIntake(2,pros::MotorGear::blue);
pros::Motor floatingIntake(-3,pros::MotorGear::blue);
pros::Motor ladyBrown(12,pros::MotorGear::green);

//sensors
pros::Rotation ladyBrownRotation(15);
pros::Optical colorSorter(13);
pros::Rotation hookRotation(14);

//Pneumatics
pros::adi::DigitalOut mogoMech('a', false);
pros::adi::DigitalOut doinker('b', false);
pros::adi::DigitalOut ringRush('c', false);
bool enableMogoMech = false;

//Variables
int autonSelection = 0;
bool auto_started = false;
int alliance = 0;

 void intakeMove(double voltage) {
    hookIntake.move(voltage);
    floatingIntake.move(voltage);
  }
  void intakeBrake() {
    hookIntake.brake();
    floatingIntake.brake();
  }

//Left button
void left_button() {
	static bool pressed1=false;
	pressed1 = !pressed1;
	if (pressed1) {
		autonSelection--; 
	} else if (autonSelection ==-1) {
		autonSelection =8;
	}
	//if left button pressed selection subtracts one
}

//Right Button
void right_button() {
	static bool pressed2 = false;
	pressed2 = !pressed2;
	if(pressed2) {
		autonSelection++;
	} else if (autonSelection ==9) {
		autonSelection =0;
	}
	// if right button pressed selection adds one
}

void center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if(pressed) {
		alliance++;
	} else if (alliance > 1) {
		alliance = 0;
	}
}

//PD Loop Code
const int numStates = 3;
int states[numStates] = {0,250,500};
int currState =0;
double target1 = 0;

void nextState () {
  currState += 1;
  if (currState ==3) {
    currState = 0;
  }
  target1 = states[currState];
}

void liftControl() {
  double kP1 = 5;
  double kD1 = 5;
  double prevError1 = 0;
  double error1 = target1 - ladyBrown.get_position();
  double derivative1 = error1 - prevError1;
  double motorVoltage1 = ((kP1 * error1)+(kD1 * derivative1)) / 12;
  ladyBrown.move(motorVoltage1);
}

//color sorting code loop
void colorSorting () {
    if (alliance ==0) {
      if (colorSorter.get_hue() < 265 & colorSorter.get_hue() > 190) {
        if (currState ==1) {
          currState =0;
          pros::delay(265);
          intakeMove(-127);
          pros::delay(100);
          intakeBrake();
          currState =1;
        } else {
          pros::delay(250);
          intakeMove(-127);
          pros::delay(100);
          intakeBrake();
        }
      }
    } else if (alliance ==1) {
      if (colorSorter.get_hue() > 330 || colorSorter.get_hue() < 30) {
        if (currState ==1) {
          currState =0;
          pros::delay(265);
          intakeMove(-127);
          pros::delay(100);
          intakeBrake();
          currState =1;
        } else {
          pros::delay(250);
          intakeMove(-127);
          pros::delay(100);
          intakeBrake();
        }
      }
    }
}

//variables for PD loop
/*double kP = 1;
double kD = 1;
double maxVoltage =80;

//PD variables
int error = 0; //Sensor value - target value 
int prevError = 0; //Error 20 msec ago
int derivative =0; //Error - prevError

void ladyBrownPD (double target) {
  while (1) {
    error = target - ladyBrown.get_position();
		derivative = error - prevError;

		double motorVoltage = (error * kP + derivative * kD) / 12; //change it -12 if spins reverse

    if (motorVoltage > maxVoltage) {
      ladyBrown.move(maxVoltage);
    } else {
      ladyBrown.move(motorVoltage);
    }
		prevError = error;
		pros::delay(10);
  }
} */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0,0,0);
  
	//sets brake modes
	ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	hookIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  floatingIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	//starts brain screen buttons
  pros::lcd::register_btn0_cb(left_button);
	pros::lcd::register_btn1_cb(center_button);
	pros::lcd::register_btn2_cb(right_button);

  pros::Task liftControlTask([]{
    while (true) {
      liftControl();
      pros::delay(10);
    }
  });

  pros::Task colorSort([] {
    while (true) {
      colorSorting();
      pros::delay(10);
    }
  });
}

//moving functions
  void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true) {
   chassis.moveToPoint(x, y, timeout, params, async );
  }
  void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = true) {
   chassis.moveToPose(x, y, theta, timeout, params, async);
  }

//turning & swinging functions NO PARAMETERS
  void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true) {
    chassis.turnToPoint(x, y, timeout, params, async);
  }
  void turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true) {
    chassis.turnToHeading(theta, timeout, params, async);
  }
  void swingToHeading(float theta, DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params = {}, bool async = true) {
    chassis.swingToHeading(theta, lockedSide, timeout, params, async);
  }
  void swingToPoint(float x, float y, DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params = {}, bool async = true) {
    chassis.swingToPoint(x, y, lockedSide, timeout, params, async);
  }

//wait until movement done
  void wait() {
   chassis.waitUntilDone();
  }

void disabled() {
}


//auton selecter & alliance color
void competition_initialize() {
  while (!auto_started) {	 // while auton isn't started
	switch(autonSelection) { // uses left and right buttons it change auton selection
      case 0:
        pros::lcd::set_text(1,"Forward/Backward Tuning");
        break;
      case 1:
        pros::lcd::set_text(1,"Right/Left Tuning");
        break;
      case 2:
        pros::lcd::set_text(1, "Right Side");
        pros::lcd::set_text(2, "1 Mobile Goal, 2 Rings");
        break;
      case 3:
        pros::lcd::set_text(1, "Right Side Solo AWP");
        pros::lcd::set_text(2, "1 Mobile Goal, 1 Alliance Stake, 3 Rings");
        break;
      case 4:
        pros::lcd::set_text(1, "Ring Rush");
        pros::lcd::set_text(2, "Red Alliance Solo AWP");
        pros::lcd::set_text(3, "1 Mobile Goal, 1 Alliance Stake, 4 Rings");
        break;
      case 5:
        pros::lcd::set_text(1, "Ring Rush");
        pros::lcd::set_text(2, "Blue Alliance Solo AWP");
        pros::lcd::set_text(3, "1 Mobile Goal, 1 Alliance Stake, 4 Rings");
        break;
      case 6:
        pros::lcd::set_text(1, "Ring Rush");
        pros::lcd::set_text(2, "Red Alliance Elim Auton");
        pros::lcd::set_text(3, "1 Mobile Goal, 5 Rings");
        break;
      case 7:
        pros::lcd::set_text(1, "Ring Rush");
        pros::lcd::set_text(2, "Blue Alliance Elim Auton");
        pros::lcd::set_text(3, "1 Mobile Goal, 5 Rings");
        break;
      case 8:
        pros::lcd::set_text(1, "Skills Auton");
        break;
    }
    switch (alliance) {
      case 0: 
       pros::lcd::set_text(5,"Alliance Color is RED");
       break;
      case 1:
       pros::lcd::set_text(5,"Alliance Color is BLUE");
       break;
    }
	pros::delay(15); 
   }
}


//Autonomous Program Functions
void forwardBackwardTuning () {
  moveToPoint(0,10,1000);
  wait();
  moveToPoint(0,0,1000);
  wait();
}

void turningTuning () {
  turnToHeading(90,1000);
  wait();
  turnToHeading(45,1000);
  wait();
  turnToHeading(0,1000);
  wait();
  }

void skillsAuton () {
chassis.setPose(0, 0, 0);
hookIntake.move(127);
pros::delay(500);
hookIntake.brake();
moveToPoint(0, 12, 1000);
wait();
moveToPoint(-23.55, 12, 1000, {.forwards = false});
wait();
mogoMech.set_value(true);
pros::delay(100);
moveToPoint(-23.55, 36, 1000);
intakeMove(127);
wait();
moveToPose(-47.57, 82.9, 0,1000);
currState = 1;
wait();
moveToPose(-40.1, 58, -90,1000);
wait();
intakeBrake();
floatingIntake.move(127);
moveToPoint(-58.9, 58, 1000);
wait();
currState = 2;
pros::delay(500);
currState = 0;
pros::delay(500);
floatingIntake.brake();
pros::delay(10);
intakeMove(127);
pros::delay(100);
moveToPose(-48.3, 36.28, 180, 1000, {.minSpeed = 65, .earlyExitRange = 5});
moveToPose(-48.3, 1, 180, 1000);
wait();
moveToPoint(-59.1, 12.5, 1000);
wait();
moveToPoint(-62.23, 1.68, 1000, {.forwards = false});
pros::delay(100);
intakeBrake();
wait();
mogoMech.set_value(false);
pros::delay(100);
moveToPoint(-7.7, 12, 1000);
wait();
moveToPoint(22.34, 12, 1000,{.forwards = false});
wait();
mogoMech.set_value(true);
pros::delay(100);
intakeMove(127);
moveToPoint(22.34, 36, 1000);
wait();
moveToPose(46.13, 83, 0, 1000);
currState = 1;
wait();
moveToPose(40.125, 58, 0, 1000);
wait();
intakeBrake();
pros::delay(10);
floatingIntake.move(127);
moveToPoint(58.9, 58, 1000);
wait();
currState = 2;
pros::delay(500);
currState = 0;
pros::delay(500);
intakeMove(127);
moveToPose(46.9, 36, 180, 1000, {.minSpeed = 65, .earlyExitRange = 5});
moveToPose(46.9, 1, 180, 1000);
wait();
moveToPoint(57.66, 11.5, 1000);
wait();
moveToPoint(60, 1.25, 1000);
pros::delay(100);
intakeBrake();
wait();
mogoMech.set_value(false);
pros::delay(100);
moveToPose(25, 81.2, -45, 1000);
floatingIntake.move(127);
wait();
moveToPose(0, 102.47, -45, 1000, {.forwards = false});
wait();
mogoMech.set_value(true);
pros::delay(100);
moveToPoint(-24.267, 82.5, 1000);
intakeMove(127);
wait();
moveToPose(-47.573, 105.117, 0, 1000, {.minSpeed = 65, .earlyExitRange = 5});
moveToPose(-47.573, 118.812, 0, 1000);
wait();
moveToPoint(-59.106, 106.799, 1000);
wait();
moveToPoint(-60.307, 117.01, 1000);
pros::delay(100);
intakeBrake();
wait();
mogoMech.set_value(false);
pros::delay(100);
moveToPose(-20.183, 103.796, 45, 1000, {.minSpeed = 65, .earlyExitRange = 5});
moveToPose(13.215, 115.569, -90, 1000, {.forwards = false});
wait();
mogoMech.set_value(true);
pros::delay(100);
moveToPoint(22.825, 120.855, 1000, {.forwards = false});
wait();
moveToPoint(60.307, 122.537, 1000, {.forwards = false});
wait();
mogoMech.set_value(false);
pros::delay(100);
moveToPoint(15,122.537,1000);
}

void twoRingRightSide () {
  chassis.setPose(0,0,-90);
  moveToPoint(-17,0,1000);
  wait();
  turnToHeading(0,1000);
  wait();
  moveToPoint(-17,-8.5,1000,{.forwards = false});
  wait();
  intakeMove(127);
  pros::delay(500);
  intakeBrake();
  moveToPoint(-17,0.6,1000);
  wait();
  turnToHeading(-90,1000);
  wait();
  moveToPoint(5,0.6,1000,{.forwards = false});
  wait();
  turnToHeading(-152.5,1000);
  wait();
  moveToPoint(31,24.6,1000,{.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  turnToHeading(90,1000);
  intakeMove(127);
  wait();
  moveToPoint(55,23.6,1000);
  wait();
  turnToHeading(-90,1000);
  wait();
  moveToPoint(17,23.6,1000);
  wait();
  intakeBrake();
  enableMogoMech = true;
}

void soloAWP () {
  chassis.setPose(0, 0, 180);
  moveToPoint(0, 29.616, 1000,{.forwards = false, .maxSpeed = 80});
  pros::delay(1200);
  mogoMech.set_value(false);
  pros::delay(100);
  intakeMove(127);
  //pros::delay(500);
  moveToPose(23.383, 29, 90,1000);
  wait();
  //pros::delay(150);
  moveToPose(-9.646, 20, -110,1000);
  wait();
  doinker.set_value(true);
  pros::delay(150);
  moveToPose(0, 23, -110,1000);
  wait();
  doinker.set_value(false);
  moveToPose(-9, 3, -130, 1000);
  pros::delay(600);
  intakeBrake();
  moveToPose(-10.5, 0, -90,1000);
  wait();
  turnToHeading(0,1000);
  wait();
  moveToPoint(-10.5, -3, 1000, {.forwards = false});
  wait();
  moveToPoint(-10.5,-1.5,1000);
  wait();
  intakeMove(127);
  pros::delay(500);
  intakeBrake();
  moveToPoint(-10.5,20,1000);
  enableMogoMech = false;
}

void ringRushLeft_soloAWP () {
  chassis.setPose(0, 0, 0);
  turnToHeading(-27,1000);
  wait();
  moveToPoint(-18.773, 43.269, 1000);
  ringRush.set_value(true);
  wait();
  moveToPose(-13.965, 32.967, 0, 1000, {.forwards = false, .minSpeed = 65, .earlyExitRange = 3});
  moveToPose(-13.965, 20.833, 0, 1000, {.forwards = false});
  wait();
  ringRush.set_value(false);
  pros::delay(250);
  moveToPoint(0.229, 33.196, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(127);
  moveToPoint(-24.268, 33.196, 1000);
  wait();
  moveToPose(7.326, 2, -207, 1000);
  wait();
  pros::delay(250);
  intakeBrake();
  pros::delay(10);
  floatingIntake.move(-127);
  moveToPoint(32.967, 2, 1000);
  wait();
  floatingIntake.brake();
  moveToPoint(24.039, 2, 1000);
  wait();
  turnToHeading(0, 1000);
  wait();
  hookIntake.move(127);
  pros::delay(500);
  moveToPoint(24.039, 44, 1000);
  enableMogoMech = false;
  hookIntake.brake();
}

void ringRushRight_soloAWP () {
  chassis.setPose(0, 0, 0);
  turnToHeading(27,1000);
  wait();
  moveToPoint(18.773, 43.269, 1000);
  ringRush.set_value(true);
  wait();
  moveToPose(13.965, 32.967, 0, 1000,{.forwards = false, .minSpeed = 65, .earlyExitRange = 3});
  moveToPose(13.965, 20.833, 0, 1000, {.forwards = false});
  wait();
  ringRush.set_value(false);
  pros::delay(250);
  moveToPoint(-0.229, 33.196, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(127);
  moveToPoint(24.268, 33.196, 1000);
  wait();
  moveToPose(-7.326, 2, -207, 1000);
  wait();
  pros::delay(250);
  intakeBrake();
  floatingIntake.move(-127);
  moveToPoint(-32.967, 2, 1000);
  wait();
  floatingIntake.brake();
  moveToPoint(-24.039, 2, 1000);
  wait();
  turnToHeading(0, 1000);
  wait();
  hookIntake.move(127);
  pros::delay(500);
  hookIntake.brake();
  moveToPoint(-24.039, 44, 1000);
  enableMogoMech = false;
}

void ringRushLeft_ElimAuton () {
  chassis.setPose(0, 0, 0);
  turnToHeading(-27,1000);
  wait();
  moveToPoint(-18.773, 43.269, 1000);
  ringRush.set_value(true);
  wait();
  moveToPose(-13.965, 32.967, 0, 1000, {.forwards = false, .minSpeed = 65, .earlyExitRange = 3});
  moveToPose(-13.965, 20.833, 0, 1000, {.forwards = false});
  wait();
  ringRush.set_value(false);
  pros::delay(250);
  moveToPoint(0.229, 33.196, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(127);
  moveToPoint(-24.268, 33.196, 1000);
  wait();
  moveToPose(-45, 2, 225, 1000);
  wait();
  moveToPose(-42,5,225,1000);
  wait();
  moveToPose(-45,5,225,1000);
  wait();
  moveToPose(7.326,2,90,1000);
  wait();
  enableMogoMech = true;
  moveToPose(72,10,90,1000);
  wait();
}

void ringRushRight_ElimAuton () {
  chassis.setPose(0, 0, 0);
  turnToHeading(27,1000);
  wait();
  ringRush.set_value(true);
  moveToPoint(18.773, 43.269, 1000);
  wait();
  moveToPose(13.965, 32.967, 0, 1000, {.forwards = false, .minSpeed = 65, .earlyExitRange = 3});
  moveToPose(13.965, 20.833, 0, 1000, {.forwards = false});
  wait();
  ringRush.set_value(false);
  pros::delay(250);
  moveToPoint(-0.229, 33.196, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  intakeMove(127);
  moveToPoint(24.268, 33.196, 1000);
  wait();
  moveToPose(45, 2, 225, 1000);
  wait();
  moveToPose(42,5,225,1000);
  wait();
  moveToPose(45,5,225,1000);
  wait();
  moveToPose(-7.326,2,90,1000);
  wait();
  enableMogoMech = true;
  moveToPose(-72,10,90,1000);
  wait();
}

void rightGoalRush_soloAWP () {
  chassis.setPose(0, 0, 0);
  moveToPoint(0, 17.54, 1000);
  wait();
  floatingIntake.move(127);
  moveToPoint(24.507, 25.95, 1000, {.minSpeed = 65, .earlyExitRange = 5});
  moveToPose(21.624, 44.69, 0, 1000);
  floatingIntake.brake();
  wait();
  doinker.set_value(true);
  pros::delay(250);
  moveToPoint(21.624, 27.871, 1000, {.forwards = false});
  wait();
  doinker.set_value(false);
  moveToPoint(21.624, 40.969, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(36.389, 9.37, 1000, {.forwards = false});
  intakeMove(127);
  pros::delay(1000);
  intakeBrake();
  mogoMech.set_value(false);
  pros::delay(100);
  moveToPose(36.389, 34.167, 0, 1000, {.minSpeed = 65, .earlyExitRange = 12});
  moveToPose(5.046, 38.683, -90, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(-24, 14.416, 1000);
  intakeMove(127);
  wait();
  intakeBrake();
  moveToPoint(-24, 24, 1000);
}

void leftGoalRush_soloAWP () {
  chassis.setPose(0, 0, 0);
  moveToPoint(0, 17.54, 1000);
  wait();
  floatingIntake.move(127);
  moveToPoint(-24.507, 25.95, 1000, {.minSpeed = 65, .earlyExitRange = 5});
  moveToPose(-21.624, 44.69, 0, 1000);
  floatingIntake.brake();
  wait();
  doinker.set_value(true);
  pros::delay(250);
  moveToPoint(-21.624, 27.871, 1000, {.forwards = false});
  wait();
  doinker.set_value(false);
  moveToPoint(-21.624, 40.969, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(-36.389, 9.37, 1000, {.forwards = false});
  intakeMove(127);
  pros::delay(1000);
  intakeBrake();
  mogoMech.set_value(false);
  pros::delay(100);
  moveToPose(-36.389, 34.167, 0, 1000, {.minSpeed = 65, .earlyExitRange = 12});
  moveToPose(-5.046, 38.683, 90, 1000, {.forwards = false});
  wait();
  mogoMech.set_value(true);
  pros::delay(100);
  moveToPoint(24, 14.416, 1000);
  intakeMove(127);
  wait();
  intakeBrake();
  moveToPoint(24, 24, 1000);
}

//Autonomous program
void autonomous() {
  auto_started = true;
  switch (autonSelection) {
    case 0:
     forwardBackwardTuning();
    break;

    case 1:
    turningTuning();
    break;

    case 2:
    twoRingRightSide();
    break;

    case 3:
    soloAWP();
    break;

    case 4: 
    ringRushLeft_soloAWP();
    break;

    case 5:
    ringRushRight_soloAWP();
    break;

    case 6:
    ringRushLeft_ElimAuton();
    break;

    case 7:
    ringRushRight_ElimAuton();
    break;

    case 8: 
    skillsAuton();
    break;

    case 9:
    rightGoalRush_soloAWP();
    break;

    case 10: 
    leftGoalRush_soloAWP();
    break;
  }
}

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
	 bool enableDoinker = false;
   auto_started = false;
   //int ladyBrownToggle =1; 
	while (true) {
     int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
     int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
     chassis.curvature(leftY, rightX);

	 if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    if(currState == 1) {
      if (hookIntake.get_efficiency() > 50 && hookIntake.get_actual_velocity() > 50) {
        intakeMove(127);
      } else {
        hookIntake.brake();
        floatingIntake.move(127);
      }
    } else {
      intakeMove(127);
    }
	 } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    if(currState == 1) {
      if (hookIntake.get_efficiency() > 50 && hookIntake.get_actual_velocity() > -50) {
        intakeMove(-127);
      } else {
        hookIntake.brake();
        floatingIntake.move(-127);
      }
    } else {
      intakeMove(-127);
    }
	 } else {
		  intakeBrake();
	 } 

   // sets up three toggles for the Lady Brown mech, first is intaking position then to scoring position, lastly to rest
    /*if (master.get_digital_new_press(DIGITAL_L1)) {
      ladyBrownToggle++;
    } else if (master.get_digital(DIGITAL_L2)) {
      ladyBrownToggle =1;
    } else if (ladyBrownToggle >3) {
      ladyBrownToggle =0;
    } else if (master.get_digital(DIGITAL_DOWN)) {
      ladyBrownToggle = 1;
    } 

   if(ladyBrownToggle ==0) {
    ladyBrown.move_relative(-50,200);
   } else if (ladyBrownToggle ==1) {
    ladyBrownPD(0);
   } else if (ladyBrownToggle ==2) {
    ladyBrownPD(250);
   } else if (ladyBrownToggle ==3) {
    ladyBrownPD(500);
   } */
   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
    nextState();
   } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    currState = 0;
   } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    currState = 2;
   }
   
     // sets up controllers for Mogo Mech, uses one toggle for enable and disable
	 if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		enableMogoMech = !enableMogoMech;
	 }
	 if (enableMogoMech) {
		mogoMech.set_value(true);
	 } else {
		mogoMech.set_value(false);
	 }

     // sets up controllers for Doinker, uses one toggle for enable and disable
   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    enableDoinker = !enableDoinker;
   }
   if(enableDoinker) {
    doinker.set_value(true);
   } else {
    doinker.set_value(false);
   }

     pros::delay(10);
	}
}