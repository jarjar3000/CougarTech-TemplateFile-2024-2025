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

//Accumulator
extern motor bottomAccumulatorL;
extern motor bottomAccumulatorR;
extern motor topAccumulator;

// Arm
extern motor rightArm;

extern inertial inertial1;
extern optical optical1;
extern limit limit1;
extern rotation leftTracking;
extern rotation rightTracking;
extern rotation centerTracking;
extern distance distance1;

// Pneumatics
extern triport expander1;
extern digital_out clamp1;
extern digital_out clamp2;
extern digital_out intakeLift1;
extern digital_out hang1;
extern timer failsafe;
extern timer positionCalculationTimer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
