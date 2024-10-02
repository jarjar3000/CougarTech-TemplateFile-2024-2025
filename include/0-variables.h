using namespace vex;
#include "vex.h"

// PID Variables
const double kP = 3;
const double kI = 2;
const double kD = 1;

const double turnKP = 0.3;
const double turnKI = 0.2;
const double turnKD = 0.1;

double error = 0;
double integral = 0;
double derivative = 0;
double prevError = 0;
double leftSpeed = 0;
double rightSpeed = 0;

// Constants
const double DRIVE_INTEGRAL_WINDUP = 3;
const double TURN_INTEGRAL_WINDUP = 45;
const double DRIVE_ERROR_TOLERANCE = 5;
const double TURN_ERROR_TOLERANCE = 1;
const double WHEEL_DIAMETER = 3.25; //in inches
double heading = 0;

// Driving Variables
const double MAX_DRIVE_SPEED = 100;