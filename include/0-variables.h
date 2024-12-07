using namespace vex;
#include "vex.h"

// PID Variables
const double kP = 7;
const double kI = 3;
const double kD = 6;

const double turnKP = 0.6;
const double turnKI = 0.3;
const double turnKD = 0.5;

double error = 0;
double integral = 0;
double derivative = 0;
double prevError = 0;
double leftSpeed = 0;
double rightSpeed = 0;

// Constants
const double DRIVE_INTEGRAL_WINDUP = 2;
const double TURN_INTEGRAL_WINDUP = 10;
const double DRIVE_ERROR_TOLERANCE = 0.5; // in inches
const double TURN_ERROR_TOLERANCE = 1;
const double WHEEL_DIAMETER = 3.25; // in inches
const double WHEEL_GEAR_RATIO = (double) 1 / 1;
double heading = 0;

// Driving Variables
const double MAX_DRIVE_SPEED = 100;
bool accumulatorIsSpinning = false;

/*
    This boolean MUST be changed and the program must be redownloaded based on the alliance color.
*/
bool allianceIsRed = true;