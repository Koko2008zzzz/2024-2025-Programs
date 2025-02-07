#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

//Motor for subsystems
inline pros::Motor hookIntake(2,pros::MotorGear::blue);
inline pros::Motor floatingIntake(-20,pros::MotorGear::blue);
inline pros::Motor ladyBrown(-10,pros::MotorGear::green);

//sensors
inline pros::Rotation ladyBrownRotation(3);
inline pros::Optical colorSorter(4);
inline pros::Rotation intakeRotation(15);
inline pros::Distance intakeDistance(5);

//Pneumatics
inline pros::adi::DigitalOut mogoMech('b', false);
inline pros::adi::DigitalOut doinker('g', false);
inline pros::adi::DigitalOut ringRush('a', false);
inline pros::adi::DigitalOut rightDoinker('h', false);

//variables for sensors
inline int alliance = 0; // 0 for red, 1 for blue

// variables for subsystems
inline bool enableMogoMech = false;
inline bool enableDoinker = false;
inline int colorSorted =0;
inline int allianceState = 0;
                    
//pd loop for lift
inline const int numStates = 6; // number of total states for Lady Brown
inline int states[numStates] = {0, 3800, 15625, 17300, 21600, 23600}; // states for Lady Brown 33, 156.25, 177, 216, 236 degrees
inline int currState = 0; // current state for Lady Brown
inline double target = 0; // target for Lady Brown
inline double prevError = 0; // previous error for Lady Brown
inline double kP =0.15; // proportional constant for Lady Brown
inline double kD =0.01; // derivative constant for Lady Brown

///////// Lady Brown Controller
// Function to move to the next state for Lady Brown
inline void nextState () {
  // Increment the current state
  currState++;
  
  // If the current state exceeds the maximum defined states, reset to 0
  if (currState >= 3) {
    currState = 0;
  }
  
  // Set the target position to the new state's position
  target = states[currState];
}

// Function to reset Lady Brown to the rest position
inline void restPosition () {
  currState = 0;
  target = states[currState];
}

// Function to tilt and untilt the Mobile Goal
inline void mogoUnTilt () {
  currState = 5;
  target = states[currState];
}

//lift control function
inline void liftControl () {
  // Calculate the error between the target position and the current position
  double error = target - ladyBrownRotation.get_position();
  
  // Calculate the derivative of the error
  double derivative = error - prevError;
  
  // Calculate the motor voltage using the PD control formula
  double motorVoltage = ((kP * error) + (kD * derivative)) / 12;
  
  // Move the motor to the calculated voltage
  ladyBrown.move(motorVoltage);
  
  // Update the previous error for the next iteration
  prevError = error;
}

//////////////////////////////
///////// Color Sorter
// Variable to track if a ring is detected
/*inline bool ringDetected = false;

// Function to sort rings based on color and distance
inline void colorSort () {
  // Get the hue value from the color sensor, checks everytime it runs
  //double color = colorSorter.get_hue();
  
  // Get the distance value from the distance sensor, checks everytime it runs
  double distance = intakeDistance.get();
  
  // Check if the alliance is blue
  if (alliance == 1) {
    // If the color is red (hue > 330 or hue < 30), set ringDetected to true
    if (colorSorter.get_hue() > 330 || colorSorter.get_hue() < 30) {
      ringDetected = true;
    }
    
    // If a ring is detected and the distance is less than 20
    if (ringDetected) {
      if (distance < 20) {
        // Move the hook intake motor to eject the ring
        pros::delay(200);
        hookIntake.move(-127);
        pros::delay(50);
        hookIntake.move(127);
        
        // Reset ringDetected to false
        ringDetected = false;
      } 
    } 
  } 
  // Check if the alliance is red
  else if (alliance == 0) {
    // If the color is blue (hue > 150 and hue < 210), set ringDetected to true
    if (colorSorter.get_hue() > 120 && colorSorter.get_hue() < 250) {
      pros::delay(10);
      hookIntake.brake();
      pros::delay(2000);
    }
  }
}*/
    // If a ring is detected and the distance is less than 20
    /*if (ringDetected) {
      if (distance < 20 && distance > 5) {
        // Move the hook intake motor to eject the ring
        pros::delay(200);
        hookIntake.move(-127);
        pros::delay(50);
        hookIntake.move(127);
        
        // Reset ringDetected to false
        ringDetected = false;
      } 
    } 
  }
} 
/*inline bool ringDetected = false;
inline void colorSort () {
  double color = colorSorter.get_hue();
  if (alliance == 1) {
    if (color > 330 || color < 30) {
      ringDetected = true;
    }
    if (ringDetected) {
      if (intakeRotation.get_position() > 20000 && intakeRotation.get_position() < 21000) {
        hookIntake.move(-127);
        pros::delay(50);
        hookIntake.move(127);
        ringDetected = false;
      } 
    } 
  } else if (alliance == 0) {
    if (color > 150 && color < 210) {
      ringDetected = true;
    }
    if (ringDetected) {
      if (intakeRotation.get_position() > 20000 && intakeRotation.get_position() < 21000) {
        hookIntake.move(-127);
        pros::delay(50);
        hookIntake.move(127);
        ringDetected = false;
      } 
    } 
  }
} */

//intake control function
inline void intakeMove(double voltage) { //moves both intake motors to a set voltage
    hookIntake.move(voltage);
    floatingIntake.move(voltage);
}

inline void intakeBrake() { //brakes both intake motors
    hookIntake.brake();
    floatingIntake.brake();
}