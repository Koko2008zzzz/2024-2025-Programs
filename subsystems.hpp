#pragma once


#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

//motors 
inline pros::Motor ladyBrown(11,pros::MotorGear::red);
inline pros::Motor intake(5, pros::MotorGear::blue);  // Make this number negative if you want to reverse the motor

//sensors
inline pros::Rotation ladyBrownRotation(15);
inline pros::Optical colorSorter(13);

//Pneumatics & 3 wire ports
inline pros::adi::DigitalOut mogoMech('g', false);
inline pros::adi::DigitalOut leftDoinker('f', false);
inline pros::adi::DigitalOut intakeLift('e', false);
inline pros::adi::DigitalOut rightDoinker('d', false);

//variables for sensors
inline int alliance = 0; // 0 for no sorting, 1 for red, 2 for blue

void intakeMove(double input);


// variables for subsystems
inline bool enableMogoMech = false;
inline bool enableLeftDoinker = false;
inline bool enableRightDoinker = false;
inline bool enableIntakePiston = false;
inline int colorSorted =0;
inline int allianceState = 0;





//pd loop for lift
inline const int numStates = 7; // number of total states for Lady Brown
inline int states[numStates] = {0, 3900, 9800, 16300, 17000, 18400, 22500}; // states for Lady Brown 30, 88, 145, 160, 207, and 226 degrees
inline int currState = 0; // current state for Lady Brown
inline double target = 0; // target for Lady Brown
inline double prevError = 0; // previous error for Lady Brown
inline double kP =0.2; // proportional constant for Lady Brown, 5kp for only motor, 0.2
inline double kD =0.00; // derivative constant for Lady Brown 



///////// Lady Brown Controller
// Function to move to the next state for Lady Brown
inline void nextState () {
  // Increment the current state
  currState++;
  
  // If the current state exceeds the maximum defined states, reset to 1
  if (currState >= 4) {
    currState = 1;
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
  currState = 6;
  target = states[currState];
}

// Function to reset the position of Lady Brown
//lift control function
inline void liftControl () {
  // Calculate the error between the target position and the current position
  double error = target - ladyBrownRotation.get_position();
 
  
  // Calculate the derivative of the error
  double derivative = error - prevError;
  
  // Calculate the motor voltage using the PD control formula
  double motorVoltage = ((kP * error) + (kD * derivative)) / 12;
  
  ladyBrown.move(motorVoltage); // Set the motor velocity to the calculated output
  prevError = error; // Update the previous error for the next iteration
}
