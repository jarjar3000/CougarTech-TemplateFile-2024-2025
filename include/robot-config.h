using namespace vex;

extern brain Brain;

//Controller
extern controller controller1;

/*
    Syntax for declaring externals:
    extern nameOfThing typeOfThing
*/

//Drive motors (4)
extern motor leftF;
extern motor leftB;
extern motor rightF;
extern motor rightB;

//Lift
extern motor leftLift;
extern motor rightLift;

//Accumulator
extern motor bottomAccumulator;
extern motor topAccumulator;

extern inertial inertial1;

// Pneumatics
extern digital_out clamp1;
extern digital_out clamp2;

extern timer failsafe;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
