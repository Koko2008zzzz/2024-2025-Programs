#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "liblvgl/misc/lv_color.h"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/motors.h"
#include <sys/_intsup.h>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu Inertial(7);

pros::Rotation rotation_horizontal(8);

pros::MotorGroup LeftMotors({1,2,3},pros::MotorGear::blue);
pros::MotorGroup RightMotors({-4,-5,-6},pros::MotorGear::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&LeftMotors, // left motor group
                              &RightMotors, // right motor group
                              12.5695, // track width
                              lemlib::Omniwheel::NEW_275, // 2.75Inch omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&rotation_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr,
                            &Inertial // inertial sensor
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

//Motors for Sub-Assmeblies
 pros::Motor intake(9,pros::MotorGear::blue);
 pros::Motor fourBar(10,pros::MotorGear::red);

//Sensors
 pros::Rotation fourBarSensor(-11);

//Pneumatics
 pros::adi::DigitalOut mogoMech('a', false);
 pros::adi::DigitalOut rightClaw('b', false);
 pros::adi::DigitalOut leftClaw('c', false);
 pros::adi::DigitalOut openClaw('d', false);

//on/off switch for PD
bool enableFourBarPD = false;

//PD settings
double kP = 0.01;
double kD = 0.01;

//PD variables
int targetAngle = 0; //Target Angle
int error = 0; //Sensor value - target value 
int prevError = 0; //Error 20 msec ago
int derivative =0; //Error - prevError

int fourBarPD () {
	while (enableFourBarPD) {

		error = fourBarSensor.get_angle() - targetAngle;
		derivative = error - prevError;

		double motorVoltage = (error * kP + derivative * kD) / 12; //change it -12 if spins reverse

		fourBar.move(motorVoltage);

		prevError = error;

		//Optional can be commented out
		if (fourBarSensor.get_angle() >= targetAngle -1 && fourBarSensor.get_angle() <= targetAngle +1) {
			fourBar.brake();
		}
	 
		pros::delay(20);
	}
	return 1;
}

void score() {
  intake.move(-127); //revrses first ring onto Wall Stake
  rightClaw.set_value(true); //slam dunks the second ring forcing first down
  leftClaw.set_value(true); //slam dunks the second ring forcing first down
  pros::delay(500); //waits .5 seconds
  intake.brake(); //stops intake from running
}

/* int Alliance = 0;
int redAutons = 0;
int blueAutons = 0;
int autonSelection = 0; */

int autonselection = 0;
bool auto_started = false;

bool startNeutralWallStake = false;
bool startAllianceWallStake = false;

void center_button() {
	/* static bool pressed = false;
	pressed = !pressed;
	if(pressed) {
		Alliance++;
	} else if (Alliance > 1) {
		Alliance = 0;
	} */
}

void left_button() {
	static bool pressed1=false;
	pressed1 = !pressed1;
	if (pressed1) {
		autonselection--; 
	} else if (autonselection ==-1) {
		autonselection =8;
	}
	//if left button pressed selection subtracts one
}

void right_button() {
	static bool pressed2 = false;
	pressed2 = !pressed2;
	if(pressed2) {
		autonselection++;
	} else if (autonselection ==8) {
		autonselection =0;
	}
	// if right button pressed selection adds one
}

void neutralWallStake() {
	targetAngle =55; //angle needed for neutral wall stake
}

void allianceWallStake() {
	targetAngle = 45; //angle needed for alliance wall stake
}

void initialize() {
	//starts brain screen and calibrates drivetrain
	pros::lcd::initialize();
	chassis.calibrate();
	//sets brake modes
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	fourBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	//starts brain screen buttons
    pros::lcd::register_btn0_cb(left_button);
	pros::lcd::register_btn1_cb(center_button);
	pros::lcd::register_btn2_cb(right_button);
}

void disabled() {}

void competition_initialize() {
while (!auto_started) {	 // while auton isn't started
	switch(autonselection){ // uses left and right buttons it change auton selection
      case 0:
        pros::lcd::set_text(1,"Auton1");
        break;
      case 1:
        pros::lcd::set_text(1,"Auton2");
        break;
      case 2:
        pros::lcd::set_text(1,"Auton3");
        break;
      case 3:
        pros::lcd::set_text(1,"Auton4");
        break;
      case 4:
        pros::lcd::set_text(1,"Auton5");
        break;
      case 5:
        pros::lcd::set_text(1,"Auton6");
        break;
      case 6:
        pros::lcd::set_text(1,"Auton7");
        break;
      case 7:
        pros::lcd::set_text(1,"Auton8");
        break;
    }
	pros::delay(20); 
   }
   /*  while(true) {	
	  if (Alliance ==0) {
		if (redAutons ==0) {
			pros::lcd::set_text(1,"Red: XXX");
			autonSelection =0;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (redAutons ==1) {
			pros::lcd::set_text(1,"Red: XXX");
			autonSelection =1;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (redAutons ==2) {
			pros::lcd::set_text(1,"Red: XXX");
			autonSelection =2;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (redAutons ==3) {
			pros::lcd::set_text(1,"Red: XXX");
			autonSelection =3;
			pros::delay(1500);
			pros::lcd::clear();
		}
	 }
	 if (Alliance ==1) {
		if (blueAutons ==0) {
			pros::lcd::set_text(1,"Blue: XXX");
			autonSelection =4;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (blueAutons ==1) {
			pros::lcd::set_text(1,"Blue: XXX");
			autonSelection =5;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (blueAutons ==2) {
			pros::lcd::set_text(1,"Blue: XXX");
			autonSelection =6;
			pros::delay(1500);
			pros::lcd::clear();
		} else if (blueAutons ==3) {
			pros::lcd::set_text(1,"Blue: XXX");
			autonSelection =7;
			pros::delay(1500);
			pros::lcd::clear();
		}
	 }
   pros::delay(20);
 } */
}

void autonomous() {
	pros::Task FourBarPID (fourBarPD); //starts Four-Bar PD loop
	auto_started = true; //sets auton started to true, allows auton to run
  switch(autonselection){ //finds and changes what auton to run
    case 0:
      
      break;
    case 1:         
      
      break;
    case 2:
      
      break;
    case 3:
      
      break;
    case 4:
      
      break;
    case 5:
      
      break;
    case 6:
      
      break;
    case 7:
      
      break;
 }
	 /*if(autonSelection ==0) {
		pros::lcd::clear();
		pros::lcd::set_text(1,"Auton1");
     } else if (autonSelection ==1) {
        pros::lcd::clear();
		pros::lcd::set_text(1,"Auton2");
	 } else if (autonSelection ==2) {
		pros::lcd::clear();
		pros::lcd::set_text(1,"Auton3"); 
	 } */
}

void opcontrol() {
 //allows for use of toggle buttons, can be changed from true to false
 bool neutralWallStakeEnable = false;
 bool allianceWallStakeEnable = false;
 bool enableRing = false;
 bool enableMogoMech = false;
//sets number of toggles for intake
 int enableIntake = 0;
 //Controller buttons and joysticks code should be in while loop
	while (1) {
	 //sets up drivetrain controls, spilt arcade
	 int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	 int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
     //left is forward-backward, right is left-right turning
     chassis.arcade(leftY, rightX,false,0.75);

	 //sets up control for neutral wall stake
	 if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
		neutralWallStakeEnable = !neutralWallStakeEnable;
	 }
	 if(neutralWallStakeEnable) {
		enableFourBarPD = true;
		neutralWallStake();
	 } else {
		score();
		targetAngle =0;
		openClaw.set_value(true);
		pros::delay(1000);
		openClaw.set_value(false);
		rightClaw.set_value(false);
		leftClaw.set_value(false);
		enableFourBarPD = false;
	 }

	 //sets up control for alliance wall stake
	 if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
		allianceWallStakeEnable = !allianceWallStakeEnable;
	 }
	 if(allianceWallStakeEnable) {
		enableFourBarPD = true;
		allianceWallStake();
	 } else {
		score();
		targetAngle =0;
		pros::delay(250);
		openClaw.set_value(true);
		pros::delay(1000);
		openClaw.set_value(false);
		rightClaw.set_value(false);
		leftClaw.set_value(false);
		enableFourBarPD = false;
	 }

	 // sets up controls for intake, uses three toggles in one button
	 if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
		enableIntake++;
	 }
	 if (enableIntake > 2) {
		enableIntake =0;
	 }
	 if(enableIntake ==0) {
		intake.brake();
	 } else if (enableIntake ==1) {
		intake.move(127);
	 } else if (enableIntake ==2) {
		intake.move(-127);
	 }

	 //sets up control for grabbing ring with claw
	 if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
		enableRing = !enableRing;
	 }
	 if (enableRing) {
		openClaw.set_value(true);
		rightClaw.set_value(true);
		leftClaw.set_value(true);
	 } else {
		openClaw.set_value(false);
		rightClaw.set_value(false);
		leftClaw.set_value(false);
	 }

	 //sets up control for grabbing a Mobile Goal
	 if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		enableMogoMech = !enableMogoMech;
	 }
	 if (enableMogoMech) {
		mogoMech.set_value(true);
	 } else {
		mogoMech.set_value(false);
	 }

		pros::delay(20);
	} 
}