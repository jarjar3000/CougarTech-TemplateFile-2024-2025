using namespace vex;

extern brain Brain;

//Controller
extern controller controller1;

/*
    Syntax for declaring externals:
    extern nameOfThing typeOfThing
*/

//Drive motors (6)
extern motor leftF;
extern motor leftB;
extern motor rightF;
extern motor rightB;
extern motor leftE;
extern motor rightE;

//Accumulator
extern motor bottomAccumulator;
extern motor topAccumulator;

extern inertial inertial1;
extern optical optical1;
extern limit limit1;

// Pneumatics
extern digital_out clamp1;
extern digital_out clamp2;
extern digital_out hang1;
extern digital_out hang2;
extern digital_out tipper;
extern digital_out ejector;

extern timer failsafe;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
