#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

//Motor for subsystems
inline pros::Motor intake(5,pros::MotorGear::blue);
inline pros::Motor ladyBrown(-11,pros::MotorGear::red);

//sensors
inline pros::Rotation ladyBrownRotation(15);
inline pros::Optical colorSorter(6);

//Pneumatics & 3 wire ports
inline pros::adi::DigitalOut mogoMech('h', false);
inline pros::adi::DigitalOut leftDoinker('g', false);
inline pros::adi::DigitalOut intakeLift('f', false);
inline pros::adi::DigitalOut rightDoinker('d', false);
inline pros::adi::DigitalIn bumper('b');

//variables for sensors
inline int alliance = 0; // 0 for red, 1 for blue

//variables for antiJamCode
inline int targetInput =0;
inline bool isJammed = false;
inline bool ringDetected = false;
inline const int waitTime = 30;
inline const int outtakeTime = 300;
inline const int minSpeed = 20;
inline int jamCounter =0;
inline const int delayTime =10;

// variables for subsystems
inline bool enableMogoMech = false;
inline bool enableLeftDoinker = false;
inline bool enableRightDoinker = false;
inline bool enableIntakePiston = false;
inline int colorSorted =0;
inline int allianceState = 0;
                    
//pd loop for lift
inline const int numStates = 7; // number of total states for Lady Brown
//inline int states[numStates] = {0, 180, 345, 675, 780, 1000, 1050}; // states for Lady Brown 37.5, 156.25, 170, 216, 236 degrees
inline int states[numStates] = {0, 3900, 8800, 14500, 16000, 20700, 22600}; // states for Lady Brown 37.5, 156.25, 170, 216, 236 degrees
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
  
  // If the current state exceeds the maximum defined states, reset to 0
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

//lift control function
inline void liftControl () {
  // Calculate the error between the target position and the current position
  //double error = target - ladyBrown.get_position();
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

void intakeMove (int input) {
  intake.move(input);
  targetInput = input;
}

void antiJamCode () {
  if(isJammed) {
    if (currState ==1 && ringDetected) {
      intake.move(127);
    } else {
    intake.move(-127);
    jamCounter += delayTime;
    if (jamCounter > outtakeTime) {
      jamCounter = 0;
      isJammed = false;
      intake.move(targetInput);
    }
   }
  } 
  ringDetected = false;
    if (targetInput >= minSpeed && intake.get_actual_velocity() ==0) {
    jamCounter += delayTime;
    if (jamCounter > waitTime) {
      jamCounter = 0;
      isJammed = true;
    }
  }
  if (colorSorter.get_hue() > 0 && colorSorter.get_hue() <15 || colorSorter.get_hue() > 100 && colorSorter.get_hue()<250) {
    ringDetected =true;
  }
  
  if (targetInput <= minSpeed) {
    jamCounter = 0;
  }
}