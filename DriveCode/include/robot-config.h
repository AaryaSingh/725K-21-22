using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RightFront;
extern motor RightBack;
extern motor LeftFront;
extern motor LeftBack;
extern motor Clamp;
extern motor Lift;
extern motor Descorer;
extern motor FourBar;
extern encoder EncoderA;
extern encoder EncoderC;
extern inertial Inertial;
extern motor RightMiddle;
extern motor LeftMiddle;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );