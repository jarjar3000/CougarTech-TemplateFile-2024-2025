using namespace vex;
#include "vex.h"

// PID Variables
const double kP = 4;
const double kI = 2;
const double kD = 3;

const double turnKP = 0.4;
const double turnKI = 0.2;
const double turnKD = 0.3;

double error = 0;
double integral = 0;
double derivative = 0;
double prevError = 0;
double leftSpeed = 0;
double rightSpeed = 0;

// Constants
const double DRIVE_INTEGRAL_WINDUP = 2;
const double TURN_INTEGRAL_WINDUP = 5;
const double DRIVE_ERROR_TOLERANCE = 0.5; // in inches
const double TURN_ERROR_TOLERANCE = 0.5;
const double WHEEL_DIAMETER = 3.25; //in inches 5
const double WHEEL_GEAR_RATIO = (double) 60 / 36;
double heading = 0;

// Driving Variables
const double MAX_DRIVE_SPEED = 100;

// Enum for arm directions
enum armDirections
{
    up,
    down
};