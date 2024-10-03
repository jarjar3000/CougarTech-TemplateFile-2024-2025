using namespace vex;

#include "0-variables.h"
#include "robot-config.h"

void clamp()
{
    if (clamp1.value() == 0)
    {
        clamp1.set(true);
        clamp2.set(true);
    }
    else
    {
        clamp1.set(false);
        clamp2.set(false);
    }
}

// Drives the robot in a direction for deg1 degrees
// TODO: Convert degrees to an actual measurement of distance (inches)
void drive(vex::directionType d, double distance, double failsafeTime)
{
    // Reset all motors + encoders
    leftF.setPosition(0, degrees);
    leftB.setPosition(0, degrees);
    rightF.setPosition(0, degrees);
    rightB.setPosition(0, degrees);

    // Reset PID variables
    double avgPosition = 0;
    prevError = 0;

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
        break;

    case vex::directionType::rev:
        // Start driving backwards
        leftF.spin(reverse, 80, percent);
        leftB.spin(reverse, 80, percent);
        rightF.spin(reverse, 80, percent);
        rightB.spin(reverse, 80, percent);
        break;

    case vex::directionType::undefined:
        break;
    }

    // Both Cases
    while (true)
    {
        controller1.Screen.clearScreen();
        // Error (Proportional)
        avgPosition = fabs((leftF.position(degrees) + rightF.position(degrees)) / 2);
        if (avgPosition == 0)
        {
            avgPosition = 0.0000001;
        }
        error = distance - (WHEEL_DIAMETER * M_PI)/avgPosition;

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

        // Change motor speed (or switch on d)
        if (d == vex::directionType::fwd)
        {
            leftF.setVelocity(leftSpeed, percent);
            leftB.setVelocity(leftSpeed, percent);
            rightF.setVelocity(rightSpeed, percent);
            rightB.setVelocity(rightSpeed, percent);
        }
        else if (d == vex::directionType::rev)
        {
            leftF.setVelocity(-leftSpeed, percent);
            leftB.setVelocity(-leftSpeed, percent);
            rightF.setVelocity(-rightSpeed, percent);
            rightB.setVelocity(-rightSpeed, percent);
        }

        printf("Error: %f\n", error);

        // Break if desination is reached AND failsafe timer is greater than input time
        if ((error >= -DRIVE_ERROR_TOLERANCE && error <= DRIVE_ERROR_TOLERANCE) && failsafe.time(seconds) >= failsafeTime)
        {
            break;
        }

        // Conserve brain resources
        wait(20, msec);
    }

    // Stop motors
    leftF.stop();
    leftB.stop();
    rightF.stop();
    rightB.stop();

    // Wait
    wait(500, msec);
}

// Turn to a set heading
void turn(vex::turnType d, double heading, double failsafeTime)
{
    // TODO: Find a way to not use the inertial? Use the encoders to figure out turn values?
    switch (d)
    {
    case vex::turnType::left:
        // Set motors to turn left
        leftF.spin(reverse, -80, percent);
        leftB.spin(reverse, -80, percent);
        rightF.spin(forward, 80, percent);
        rightB.spin(forward, 80, percent);

        // Reset Failsafe Timer
        failsafe.clear();

        // Reset variables
        prevError = 0;

        while (1)
        {
            // Error
            error = fabs(inertial1.heading(degrees) - heading);

            // Ensure error is less than 180
            if (error >= 180)
            {
                error -= 180;
            }

            // Integral
            integral += error;

            // Prevent integral windup
            if (error > TURN_INTEGRAL_WINDUP)
            {
                integral = 0;
            }

            // Derivative
            derivative = error - prevError;
            prevError = error;

            // Calculate and set motor speeds
            leftSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
            rightSpeed = error * turnKP + integral * turnKI + derivative * turnKD;

            // Change motor speed
            leftF.setVelocity(-leftSpeed, percent);
            leftB.setVelocity(-leftSpeed, percent);
            rightF.setVelocity(rightSpeed, percent);
            rightB.setVelocity(rightSpeed, percent);
        }
        break;
    case vex::turnType::right:
        // Set motors to turn left
        leftF.spin(forward, 80, percent);
        leftB.spin(forward, 80, percent);
        rightF.spin(reverse, -80, percent);
        rightB.spin(reverse, -80, percent);

        // Reset Failsafe Timer
        failsafe.clear();

        // Reset variables
        prevError = 0;

        while (1)
        {
            // Error
            error = fabs(inertial1.heading(degrees) - heading);

            // Ensure error is less than 180
            if (error >= 180)
            {
                error -= 180;
            }

            // Integral
            integral += error;

            // Prevent integral windup
            if (error > TURN_INTEGRAL_WINDUP)
            {
                integral = 0;
            }

            // Derivative
            derivative = error - prevError;
            prevError = error;

            // Calculate and set motor speeds
            leftSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
            rightSpeed = error * turnKP + integral * turnKI + derivative * turnKD;

            // Change motor speed
            leftF.setVelocity(leftSpeed, percent);
            leftB.setVelocity(leftSpeed, percent);
            rightF.setVelocity(-rightSpeed, percent);
            rightB.setVelocity(-rightSpeed, percent);
        }
        break;
    }

    // Stop motors and wait
    leftF.stop();
    leftB.stop();
    rightF.stop();
    rightB.stop();

    wait(500, msec);
}

void spinAccumulator(vex::directionType d, double vel)
{
    bottomAccumulator.spin(d, vel, percent);
    topAccumulator.spin(d, vel, percent);
}

void stopAccumulator()
{
    bottomAccumulator.stop();
    topAccumulator.stop();
}
