/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\aarya                                            */
/*    Created:      Sat Sep 18 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RightFront           motor         11              
// RightBack            motor         4               
// LeftFront            motor         19              
// LeftBack             motor         5               
// Clamp                motor         20              
// Lift                 motor         2               
// Descorer             motor         10              
// FourBar              motor         3               
// EncoderA             encoder       A, B            
// EncoderC             encoder       C, D            
// Inertial             inertial      12              
// RightMiddle          motor         1               
// LeftMiddle           motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include <iostream>
#include "vex.h"
//#include "robot-config.cpp"
using namespace vex;
using namespace std;
competition Competition;
motor_group baseMotors = motor_group(RightFront, RightMiddle, RightBack, LeftFront, LeftMiddle, LeftBack);
motor_group RightMotors = motor_group(RightFront, RightMiddle, RightBack);
motor_group LeftMotors = motor_group(LeftFront, LeftMiddle, LeftBack);
motor_group AutonRight = motor_group(RightFront, RightMiddle);
motor_group AutonLeft = motor_group(LeftFront, LeftMiddle);
motor_group driveMotors = motor_group(LeftFront, LeftMiddle, RightFront, RightMiddle);


void pre_auton(void){
  Inertial.startCalibration();
  vexcodeInit();
  while(Inertial.isCalibrating()){
    vex::task::sleep(20);
  }
}

//tuning values
double kP = 0.028;
double kI = 0.0;
double kD = 0.005;

double leftGain = 0.95;
double rightGain = 1.1;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

int pos;
int prevPos;
int desiredValue = 0;
double desiredTurnValue = 0;

int error;
int prevError = 0;
int derivative;
int totalError = 0;


double turnError;
double prevTurnError = 0.0;
int turnDerivative;
int turnTotalError = 0;

double currHeading;
double prevHeading;
bool enablePID = true;
bool resetEncoders = true;
bool inPosition = false;
int timeCounter = 0;
int counter = 0;
int turnCounter = 0;

int drivePID(){
  printf("in PID \n");
  while(enablePID && !inPosition){
    if(resetEncoders){
      printf("in reset if \n");
      resetEncoders = false;
      //EncoderA.resetRotation();
      EncoderC.resetRotation();
      vex::task::sleep(100);
    }
  
  pos = EncoderC.position(deg);
  
  if(prevPos != pos)
    std::cout << pos * (3.14*2.8)/360 << std::endl;
  prevPos = pos;

  error = desiredValue - pos;
  derivative = error - prevError;
  totalError += error;

  double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
  prevError = error;

  RightMotors.spin(directionType::fwd, lateralMotorPower * rightGain, volt);
  LeftMotors.spin(directionType::fwd, lateralMotorPower * leftGain, volt);

  vex::task::sleep(20);

  if(abs(error) < 30){
    counter++;
   // std::cout << "counter incremented" << std::endl;
  }

  if(prevError == error){
    timeCounter++;
   // std::cout << "time counter incremented" << std::endl;
  }
  if(counter > 20 || timeCounter > 100) inPosition = true;
  }
  return 1;
}

int turnPID(){
  printf("in turnPID \n");
  while(enablePID && !inPosition){

  double val = Inertial.heading();
  if(val > 180) val -= 360;
  currHeading = val;

  if(fabs(prevHeading - currHeading) > 0.05){
    std::cout << "heading" << std::endl;
    std::cout << currHeading << std::endl;
  }
  
  prevHeading = currHeading;
  turnError = desiredTurnValue - currHeading;

  if(fabs(prevTurnError - turnError) > 0.1){
    std::cout << "turnError" << std::endl;
    std::cout << turnError << std::endl;
  }
  

  turnDerivative = turnError;

  double turnMotorPower = turnError * turnkP + turnDerivative * turnkD /*+totalError * kI*/;
  
  prevError = error;
  prevTurnError = turnError;

  RightMotors.spin(directionType::fwd, turnMotorPower*-1, volt);
  LeftMotors.spin(directionType::fwd, turnMotorPower, volt);

  vex::task::sleep(20);

  if(fabs(turnError) < 0.5){
    turnCounter++;
    std::cout << "turnCounter increment" << std::endl;
  }
  if(turnCounter > 30) inPosition = true;
  }
  return 1;
}

void mogoline(){
  driveMotors.setVelocity(50, percent);
  driveMotors.spinToPosition(1.02, rev);
  Clamp.spin(directionType::fwd, -12, volt);
  vex::task::sleep(500);
  driveMotors.spinToPosition(0, rev);
  Clamp.spin(directionType::fwd, 5, volt);
  vex::task::sleep(100);
  Clamp.spin(directionType::fwd, 0, volt);
  
 
}

void mogoramp(){
  driveMotors.setVelocity(70, percent);
  driveMotors.spinToPosition(1.15, rev);
  Clamp.spin(directionType::fwd, -12, volt);
  vex::task::sleep(500);
  Clamp.spin(directionType::fwd, 5, volt);
  vex::task::sleep(100);
  Clamp.spin(directionType::fwd, 0, volt);
}

void leftSide(){
  driveMotors.resetPosition();
  driveMotors.setVelocity(100, percent);
  driveMotors.spinToPosition(-3.5, rev);
  driveMotors.setVelocity(80, percent);
  Lift.setVelocity(100, percent);
  Lift.spinToPosition(2, rev, false);
  driveMotors.spinToPosition(-4.9, rev);
  vex::task::sleep(250);
  driveMotors.spinToPosition(0, rev);

}

void rampup(){
  driveMotors.setVelocity(100, percent);
  driveMotors.spinToPosition(5.3, rev);
}

void rightSide(){
  driveMotors.resetPosition();
  driveMotors.setVelocity(100, percent);
  driveMotors.spinToPosition(-3.5, rev);
  driveMotors.setVelocity(80, percent);
  Lift.setVelocity(100, percent);
  Lift.spinToPosition(2, rev, false);
  driveMotors.spinToPosition(-4.5, rev);
  vex::task::sleep(250);
  driveMotors.spinToPosition(0, rev);
}

void skills(){
  //CHANGE ONLY THESE VARIABLES
  double distance_one = 1.3;
  double turn_one = 0.1;
  double distance_two = 7;

  //directionType::fwd
  driveMotors.resetPosition();
  driveMotors.setVelocity(70, percent);
  driveMotors.spinToPosition(distance_one, rev);
  Clamp.spin(directionType::fwd, -12, volt);
  vex::task::sleep(500);

  //turn
  AutonLeft.spinToPosition(distance_one - turn_one, rev, false);
  AutonRight.spinToPosition(distance_one + turn_one, rev);

  //directionType::fwd
  driveMotors.setVelocity(100, percent);
  AutonLeft.spinToPosition(distance_one - turn_one + distance_two, rev, false);
  AutonRight.spinToPosition(distance_one + turn_one + distance_two, rev);
}

void moveDistance(int inches){
  //convert inches to ticks
  desiredValue = inches * 360.0/(3.141*2.8);

  //run the motion
  drivePID();

  //reset values 
  resetEncoders = true;
  inPosition = false;
  counter = 0;
  timeCounter = 0;
}

void rotate(double rotVal){

}

void autonomous(void){
  printf("in auton \n"); 
  enablePID = true;

  Lift.setVelocity(53, percent);
  Lift.spinToPosition(1.3, rev, false);
  moveDistance(-55);
  moveDistance(40);
 // moveDistance(-48);
  //moveDistance(12);
  printf("exited 2 \n"); 
  //vex::task driveProfile(drivePID);
  //desiredValue = 1080;
  
  //leftSide();
  //rightSide();
  //mogoline();
}

void usercontrol(void) {
  printf("in usercontrol \n");
  enablePID = false;
  // User control code here, inside the loop
  while (1) { 
    //printf("in driver loop \n");
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    
    if(abs(Controller1.Axis2.value()) > 10) RightMotors.spin(directionType::fwd, Controller1.Axis2.value()/8, volt);
      else RightMotors.spin(directionType::fwd, 0, volt);
    if(abs(Controller1.Axis3.value()) > 10) LeftMotors.spin(directionType::fwd, Controller1.Axis3.value()/8, volt);
      else LeftMotors.spin(directionType::fwd, 0, volt);
      
    // std::cout << "hello" << std::endl;

    Lift.spin(directionType::fwd, (Controller1.ButtonR1.pressing() - Controller1.ButtonR2.pressing())*12, volt);
    Clamp.spin(directionType::fwd, (Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing())*12, volt);
    //Descorer.spin(directionType::fwd, (Controller1.ButtonDown.pressing() - Controller1.ButtonB.pressing())*12, volt);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {

  Inertial.startCalibration();
  printf("calibrating inertial \n");
  while(Inertial.isCalibrating()){
    vex::task::sleep(1000);
    //printf("calibrating inertial \n");
  }

  Brain.Screen.print("ready");


  // Set up callbacks for autonomous and driver control periods.
  printf("in main \n");
  Competition.autonomous(autonomous);
  printf( "after auton \n");
  Competition.drivercontrol(usercontrol);
 printf( "after driver \n");
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

/*int drivePID(){
  printf("in PID \n");
  while(enablePID && !inPosition){
    if(resetEncoders){
      printf("in reset if \n");
      resetEncoders = false;
      EncoderA.resetRotation();
      EncoderC.resetRotation();
    }
  
  pos = EncoderC.position(deg);
  if(prevPos != pos)
  std::cout << pos << std::endl;
  prevPos = pos;
  //EncoderA.position(deg);

  error = desiredValue - pos;
  derivative = error - prevError;
  totalError += error;

  double lateralMotorPower = error * kP + derivative * kD + totalError * kI;


  double val = Inertial.heading();
  if(val > 180) val -= 360;
  currHeading = val;

  if(fabs(prevHeading - currHeading) > 0.05){
    std::cout << "heading" << std::endl;
    std::cout << currHeading << std::endl;
  }
  
  prevHeading = currHeading;
  

  turnError = desiredTurnValue - currHeading;

  if(fabs(prevTurnError - turnError) > 0.1){
    std::cout << "turnError" << std::endl;
    std::cout << turnError << std::endl;
  }
  

  turnDerivative = turnError;

  double turnMotorPower = turnError * turnkP + turnDerivative * turnkD;
  
  prevError = error;
  prevTurnError = turnError;

  RightMotors.spin(directionType::fwd, lateralMotorPower - turnMotorPower, volt);
  LeftMotors.spin(directionType::fwd, lateralMotorPower + turnMotorPower, volt);

  vex::task::sleep( 20);

  if(abs(error) < 3 && fabs(turnError) < 0.5) counter++;
  if(counter > 30) inPosition = true;

  }
  return 1;
}*/

