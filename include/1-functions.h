using namespace vex;
#include "0-variables.h"
#include "robot-config.h"

void clamp()
{
    if (extended)
    {
        clamp1.set(true);
        clamp2.set(true);
        extended = false;
    }
    else
    {
        clamp1.set(false);
        clamp2.set(false);
        extended = true;
    }
}

void slowDown()
{
    if (slow)
    {
        slow = false;
    }
    else
    {
        slow = true;
    }
}

void drive(vex::directionType d, double deg1, double failsafeTime)
{
    // Reset all motors + encoders
    leftF.setPosition(0, degrees);
    leftB.setPosition(0, degrees);
    rightF.setPosition(0, degrees);
    rightB.setPosition(0, degrees);

    // Set velocity
    leftF.setVelocity(80, percent);
    leftB.setVelocity(80, percent);
    rightF.setVelocity(80, percent);
    rightB.setVelocity(80, percent);

    // Reset PID variables
    double avgPosition = 0;

    // Reset failsafe timer
    failsafe.clear();

    switch (d)
    {
    case vex::directionType::fwd:
        // Start driving forward
        leftF.spin(forward);
        leftB.spin(forward);
        rightF.spin(forward);
        rightB.spin(forward);

        while (1)
        {
            // Error (Proportional)
            avgPosition = (leftF.position(degrees) + rightF.position(degrees)) / 2;
            error = deg1 - avgPosition;

            // Integral
            integral += error;

            // Derivative
            derivative = error - prevError;
            prevError = error;

            // Calculate
            leftSpeed = error * kP + integral * kI + derivative * kD;
            rightSpeed = error * kP + integral * kI + derivative * kD;

            // Change motor speed
            leftF.setVelocity(leftSpeed, percent);
            leftB.setVelocity(leftSpeed, percent);
            rightF.setVelocity(rightSpeed, percent);
            rightB.setVelocity(rightSpeed, percent);
        }

        break;

    case vex::directionType::rev:
        break;

    case vex::directionType::undefined:
        break;
    }
}