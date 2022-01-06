#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightFront = motor(PORT11, ratio18_1, true);
motor RightBack = motor(PORT4, ratio18_1, true);
motor LeftFront = motor(PORT19, ratio18_1, false);
motor LeftBack = motor(PORT5, ratio18_1, false);
motor Clamp = motor(PORT20, ratio6_1, true);
motor Lift = motor(PORT2, ratio36_1, false);
motor Descorer = motor(PORT10, ratio18_1, true);
motor FourBar = motor(PORT3, ratio18_1, true);
encoder EncoderA = encoder(Brain.ThreeWirePort.A);
encoder EncoderC = encoder(Brain.ThreeWirePort.C);
inertial Inertial = inertial(PORT12);
motor RightMiddle = motor(PORT1, ratio18_1, true);
motor LeftMiddle = motor(PORT9, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}