#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

//Controller (a second controller may be set up with using "partner" instead of "primary")
controller controller1 = controller(primary);

/*
  Syntax for motor setup:
  motor nameOfMotor = motor(port number, ratio of cartridge in motor, reverse status)
*/

/*
  If motors don't function, check:
  - Reverse status (the motors may be attempting to move in opposite directions)
  - Check your cartriges and make them match with the program
  - Make sure the program matches port assignments on the robot
*/

/*
  Drivetrain
  Motor names:
  - LeftF: Front left motor
  - LeftB: Back left motor
  - RightF: Front right motor
  - RightB: Back right motor
*/
motor leftF = motor(PORT11, ratio18_1, false);
motor leftB = motor(PORT12, ratio18_1, false);
motor leftE = motor(PORT14, ratio18_1, false);
motor rightF = motor(PORT17, ratio18_1, true);
motor rightB = motor(PORT15, ratio18_1, true);
motor rightE = motor(PORT16, ratio18_1, true);

/*
  Accumulator
*/
motor bottomAccumulator = motor(PORT20, ratio18_1, true);
motor topAccumulator = motor(PORT10, ratio18_1, true);

//Sensors
inertial inertial1 = inertial(PORT18);
optical optical1 = optical(PORT6);

// Pneumatics
digital_out clamp1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp2 = digital_out(Brain.ThreeWirePort.F);
digital_out hang1 = digital_out(Brain.ThreeWirePort.B);
digital_out hang2 = digital_out(Brain.ThreeWirePort.C);
digital_out ejector = digital_out(Brain.ThreeWirePort.D);
digital_out tipper = digital_out(Brain.ThreeWirePort.E);

timer failsafe = timer();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}