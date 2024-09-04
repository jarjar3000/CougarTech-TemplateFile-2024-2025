using namespace vex;
#include "vex.h"

bool extended = false;

// PID Variables
const double kP = 0.3;
const double kI = 0.2;
const double kD = 0.1;
double error = 0;
double integral = 0;
double derivative = 0;
double prevError = 0;
double leftSpeed = 0;
double rightSpeed = 0;