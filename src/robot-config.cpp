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
motor leftF = motor(PORT1, ratio18_1, true);
motor leftB = motor(PORT2, ratio18_1, true);
motor rightF = motor(PORT3, ratio18_1, false);
motor rightB = motor(PORT4, ratio18_1, false);

/*
  Lift (Arm)
  Motor Names:
  - leftLift: left arm motor (also the default name if you only use one motor)
  - rightLift: right arm motor (delete if you only have 1 motor)
*/
motor leftLift = motor(PORT18, ratio18_1, false);
motor rightLift = motor(PORT19, ratio18_1, true);

/*
  Accumulator
*/
motor bottomAccumulator = motor(PORT5, ratio18_1, true);
motor topAccumulator = motor(PORT16, ratio18_1, true);

//Sensors
inertial inertial1 = inertial(PORT17);

// Pneumatics
digital_out clamp1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp2 = digital_out(Brain.ThreeWirePort.B);

optical optical1 = optical(PORT9);
vision vision1 = vision(PORT10);
limit limit1 = limit(Brain.ThreeWirePort.C);

timer failsafe = timer();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}