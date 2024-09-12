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
            leftF.spin(forward, 80, percent);
            leftB.spin(forward, 80, percent);
            rightF.spin(forward, 80, percent);
            rightB.spin(forward, 80, percent);

            while (1)
            {
                // Error (Proportional)
                avgPosition = fabs((leftF.position(degrees) + rightF.position(degrees)) / 2);
                error = deg1 - avgPosition;

                // Integral
                integral += error;

                // Prevent integral windup
                if (error > DRIVE_INTEGRAL_WINDUP)
                {
                    integral = 0;
                }

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

                // Break if desination is reached
                if (error >= -DRIVE_ERROR_TOLERANCE && error <= DRIVE_ERROR_TOLERANCE)
                {
                    break;
                }

                // If failsafe timer is greater than input time, break
                if (failsafe.time() >= failsafeTime)
                {
                    break;
                }

                // Conserve brain resources
                wait(20, msec);
            }

            break;

        case vex::directionType::rev:
            // Start driving backwards
            leftF.spin(reverse, 80, percent);
            leftB.spin(reverse, 80, percent);
            rightF.spin(reverse, 80, percent);
            rightB.spin(reverse, 80, percent);

            while (1)
            {
                // Error (Proportional)
                avgPosition = fabs((leftF.position(degrees) + rightF.position(degrees)) / 2);
                error = deg1 - avgPosition;

                // Integral
                integral += error;

                // Prevent integral windup
                if (error > DRIVE_INTEGRAL_WINDUP)
                {
                    integral = 0;
                }

                // Derivative
                derivative = error - prevError;
                prevError = error;

                // Calculate
                leftSpeed = error * kP + integral * kI + derivative * kD;
                rightSpeed = error * kP + integral * kI + derivative * kD;

                // Change motor speed
                leftF.setVelocity(-leftSpeed, percent);
                leftB.setVelocity(-leftSpeed, percent);
                rightF.setVelocity(-rightSpeed, percent);
                rightB.setVelocity(-rightSpeed, percent);

                // Break if desination is reached
                if (error >= -DRIVE_ERROR_TOLERANCE && error <= DRIVE_ERROR_TOLERANCE)
                {
                    break;
                }

                // If failsafe timer is greater than input time, break
                if (failsafe.time() >= failsafeTime)
                {
                    break;
                }

                // Conserve brain resources
                wait(20, msec);
            }
            break;

        case vex::directionType::undefined:
            break;
    }

    // Stop motors
    leftF.stop();
    leftB.stop();
    rightF.stop();
    rightB.stop();

    // Wait
    wait(500, msec);
}