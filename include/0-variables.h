using namespace vex;
#include "vex.h"

bool extended = false;
bool slow = false;
const double LOWER_ACCUMULATOR_THRESHOLD = 0; // 200
const double HIGHER_ACCUMULATOR_THRESHOLD = 200; // 150

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