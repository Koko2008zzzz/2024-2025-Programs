#include "main.h"


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {19, -18, -20},     // Left Chassis Ports (negative port will reverse it!), 19, -18, -20
    {-1, 2, 3},  // Right Chassis Ports (negative port will reverse it!), -1, 2, 3

    12,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
 ez::tracking_wheel horiz_tracker(-10, 2, -1.75);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

// Function to sort objects based on color
void colorSort () {
  // Check if the alliance is set to 1 (e.g., red alliance)
  if (alliance == 1) {
    // If the detected hue is in the range for red (0 to 30)
    if (colorSorter.get_hue() > 0 && colorSorter.get_hue() < 30) {
      pros::delay(3); // Small delay before braking the intake
      intake.brake();   // Stop the intake motor
      pros::delay(200); // Wait for 200 milliseconds
      intake.move(127); // Resume intake motor movement
    }
  } 
  // Check if the alliance is set to 2 (e.g., blue alliance)
  else if (alliance == 2) {
    // If the detected hue is in the range for blue (100 to 250)
    if (colorSorter.get_hue() > 100 && colorSorter.get_hue() < 250) {
      pros::delay(12);   // Small delay before braking the intake
      intake.brake();   // Stop the intake motor
      pros::delay(200); // Wait for 200 milliseconds
      intake.move(127); // Resume intake motor movement
    }
  } 
  // If the alliance is set to 0 (neutral or no sorting)
  else if (alliance == 0) {
    // Do nothing
  }
}

// Define the intakeMove function
void intakeMove(double input) {
  // Move the intake motor with the specified input value
  intake.move(input);
  floatingIntake.move(input); // Move the floating intake motor with the same input value
  
  // Set the target input value for the intake motor
  targetInput = input;
} 

// Define the antiJamCode function
void antiJamCode() {
  // Check if the anti-jam mechanism is enabled
  if (antiJam) {
    // If the intake is jammed
    if (isJammed) {
      intake.move(-127); // Reverse the intake motor to clear the jam
      jamCounter += delayTime; // Increment the jam counter by the delay time
      if (jamCounter > outtakeTime) { // If the jam counter exceeds the outtake time
        jamCounter = 0; // Reset the jam counter
        isJammed = false; // Mark the intake as no longer jammed
        intake.move(targetInput); // Resume intake motor movement with the target input
      }
    }

    // Check if the intake motor is stalled (target input is above minimum speed but velocity is zero)
    if (targetInput >= minSpeed && intake.get_actual_velocity() == 0) {
      jamCounter += delayTime; // Increment the jam counter by the delay time
      if (jamCounter > waitTime) { // If the jam counter exceeds the wait time
        jamCounter = 0; // Reset the jam counter
        isJammed = true; // Mark the intake as jammed
      }
    }

    // If the target input is below the minimum speed, reset the jam counter
    if (targetInput <= minSpeed) {
      jamCounter = 0;
    }
  } else {
    // If the anti-jam mechanism is disabled, move the intake motor with the target input
    intake.move(targetInput);
  }
} 




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  //sets rotation  sensor to zero  position and sets it reversed
  
  colorSorter.set_integration_time(10);
  colorSorter.set_led_pwm(100);


  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);//was 0.0   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Skills\n\nSkills Auton", skillsAuton},
      {"State Solo AWP", soloAWP},
      {"right side five on mogo\n\nRIght side", rightAutonSixRing},
      {"Ring Rush \n\nRed Side", ringRushLeft_fiveRingMogo},
      {"Ring Rush\n\nBlue Side", ringRushRight_fiveRingMogo},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

pros::Task colorSortTask([]{
    while (1){
    colorSort();
    pros::delay(10);
    }
  });
  // Starts Lady Brown PD Loop Control
  pros::Task liftControlTask([]{
    while (true) {
      liftControl();
      pros::delay(10);
    }
  });
  pros::Task antiJamTask([]{
    while (true) {
      antiJamCode();
      pros::delay(10);
    }
  });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

void autonomous() {
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/*
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 */

void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.
  antiJam =  false; // Enable anti-jam mechanism
  alliance = 0; 

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // spilt arcade drive
    
  

   // sets up controls for intake
   if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // moves both intake motors forwrard
     intakeMove(127);
   } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // moves floating intake motor backwards and brakes the hook intake motor
            intakeMove(-127);
   } else { // brakes both motors
     intakeMove(0);
   } 
           
   
  // sets up controls for ladyBrown
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) { // toggle for next state, used for scoring on Wall Stakes
   nextState(); // Move ladyBrown to the next state
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { // sets ladyBrown to rest position
   restPosition(); // Move ladyBrown to the rest position
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { // toggles ladyBrown allianceState, scores on alliance stake
   allianceState++; // Increment allianceState to score on alliance stake
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { // sets ladyBrown to tilt and untilt a Mobile Goal
   mogoUnTilt(); // Tilt or untilt a Mobile Goal
  }


  // sets up controls for allianceState, used for scoring on alliance stakes
  if (allianceState == 1) { // If allianceState is 1, set ladyBrown to directly above alliance stake
    currState = 3; // Set current state to 3
    target = states[currState]; // Set target to the state corresponding to currState
    pros::delay(10); // Delay for 10 milliseconds
    allianceState = 0; // Reset allianceState to 0
  } else {
    //nothing
  }

  
  // sets up controllers for Mogo Mech, uses one toggle for enable and disable
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { // Toggle Mogo Mech on RIGHT button press
    enableMogoMech = !enableMogoMech;
  }
  if (enableMogoMech) {
    mogoMech.set_value(true); // Enable the Mogo Mech
  } else {
    mogoMech.set_value(false); // Disable the Mogo Mech
  }

  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { // Toggle Mogo Mech on LEFT button press
    enableLeftDoinker = !enableLeftDoinker;
  }

  if (enableLeftDoinker) {
    leftDoinker.set_value(true); // Enable the Doinker
  } else {
    leftDoinker.set_value(false); // Disable the Doinker
  }


  // sets up controllers for Doinker, uses one toggle for enable and disable
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { // Toggle Doinker on Y button press
    enableRightDoinker = !enableRightDoinker;
  }
  if(enableRightDoinker) {
    rightDoinker.set_value(true); // Enable the doinker
  } else {
  // Disable the doinker
    rightDoinker.set_value(false);
  }

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { // Toggle Intake Piston on X button press
    enableIntakePiston = !enableIntakePiston;
  }

  if(enableIntakePiston) {
    intakeLift.set_value(true); // Enable the Intake Piston
  } else {
    intakeLift.set_value(false); // Disable the Intake Piston
  }

  /*if (currState == 1) {
    kP = 0.35;
    kD = 0.0;
  } else if (currState == 2) {
    kP = 0.25;
    kD = 0.05;
  } else if (currState == 3) {
    kP = 0.25;
    kD = 0.05;
  } else if (currState == 4) {
    kP = 0.25;
    kD = 0.05;
  } else if (currState == 5) {
    kP = 0.25;
    kD = 0.05;
  } else if (currState == 6) {
    kP = 0.25;
    kD = 0.05;
  } else {
    kP = 0.35;
    kD = 0.0;
  } */

   /*if (currState == 1) {
    kP = 6;
    kD = 0.0;
  } else if (currState == 2) {
    kP = 15;
    kD = 0.0;
  } else if (currState == 3) {
    kP = 8;
    kD = 0.0;
  } else if (currState == 4) {
    kP = 6;
    kD = 0.0;
  } else if (currState == 5) {
    kP = 8;
    kD = 0.0;
  } else if (currState == 6) {
    kP = 3.5;
    kD = 0.0;
  } else {
    kP = 6;
    kD = 0.0;
  } */


   pros::delay(ez::util::DELAY_TIME);  // Delay to prevent the CPU from getting overwhelmed
  }
}
