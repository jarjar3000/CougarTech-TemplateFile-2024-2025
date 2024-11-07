using namespace vex;

#include "0-variables.h"
#include "robot-config.h"

/**
 * @brief Function to set speed of left side of the drive
 * @param vel The speed to set the left side to.
 */
void setLeftSpeed(double vel)
{
    leftB.setVelocity(vel, percent);
    leftF.setVelocity(vel, percent);
    leftE.setVelocity(vel, percent);
}

/**
 * @brief Function to set speed of right side of the drive
 * @param vel The speed to set the right side to.
 */
void setRightSpeed(double vel)
{
    rightB.setVelocity(vel, percent);
    rightF.setVelocity(vel, percent);
    rightE.setVelocity(vel, percent);
}

/**
 * @brief Function to spin the drive motors in a direction
 * @param d The direction to spin the drive in.
 */
void drive(vex::directionType d)
{
    switch(d)
    {
        case vex::directionType::fwd:
        leftF.spin(forward);
        leftB.spin(forward);
        leftE.spin(forward);
        rightF.spin(forward);
        rightB.spin(forward);
        rightE.spin(forward);
        break;

        case vex::directionType::rev:
        leftF.spin(reverse);
        leftB.spin(reverse);
        leftE.spin(reverse);
        rightF.spin(reverse);
        rightB.spin(reverse);
        rightE.spin(reverse);
        break;

        case vex::directionType::undefined:
        break;
    }
}

/**
 * @brief Stops all the drive motors
 */
void stopDrive()
{
    leftF.stop();
    leftB.stop();
    leftE.stop();
    rightF.stop();
    rightB.stop();
    rightE.stop();
}

/** 
 * @brief Function to toggle clamp. Starting position is up.
*/
void clamp()
{
    if (clamp1.value() == 0)
    {
        clamp1.set(true);
    }
    else
    {
        clamp1.set(false);
    }
}

/** 
 * @brief Function to toggle alliance color. Default is red (true)
*/
void changeAllianceColor()
{
    if (allianceIsRed)
    {
        allianceIsRed = false;
    }
    else
    {
        allianceIsRed = true;
    }
}

/** 
    * @brief Thread Function to detect if rings of the opposite alliance color pass the intake, and ejects them.
*/
int eject()
{
    // This should run the entire match, EXCEPT when we intake our first ring.
    // The first ring that passes the sensor will be the alliance color (since it's the preload), so this can dictate the alliance color.
    while (1)
    {
        // Search for the color opposite the alliance color
        if ((optical1.color() == blue && allianceIsRed) || (optical1.color() == red && !allianceIsRed))
        {
            // Wait for a little while (until the ring is close)
            this_thread::sleep_for(40);

            // Extend the ejector
            ejector.set(true);

            // Wait until the ring ejects
            this_thread::sleep_for(40);

            // Retract the ejector
            ejector.set(false);
        }
        // Sleep thread to not consume all of CPU resources
        this_thread::sleep_for(20);
    }
    return 0;
}

/**
 * @brief Drive the robot in a direction for a certain distance in inches. Used in autonomous.
 * @param d The direction to drive in.
 * @param distance The distance the robot should drive in inches.
 * @param failsafeTime The amount of time that needs to pass before the robot automatically stops moving.
 */
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

    // Set initial velocity
    setLeftSpeed(80);
    setRightSpeed(80);

    switch (d)
    {
    case vex::directionType::fwd:
        // Start driving forward
        drive(forward);
        break;

    case vex::directionType::rev:
        // Start driving backwards
        drive(reverse);
        break;

    case vex::directionType::undefined:
        break;
    }

    // Both Cases
    while ((error < -DRIVE_ERROR_TOLERANCE && error > DRIVE_ERROR_TOLERANCE) || failsafe.time(seconds) <= failsafeTime)
    {
        // Error (Proportional)
        avgPosition = fabs((leftF.position(degrees) + rightF.position(degrees)) / 2);
        // avgPosition = fabs(leftF.position(degrees));

        // The distance in inches minus the distance traveled (wheel circumfrence times rotations)
        error = distance - (WHEEL_DIAMETER * M_PI) * (avgPosition / 360) * WHEEL_GEAR_RATIO;

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
            setLeftSpeed(leftSpeed);
            setRightSpeed(rightSpeed);
        }
        else if (d == vex::directionType::rev)
        {
            setLeftSpeed(-leftSpeed);
            setRightSpeed(-rightSpeed);
        }

        // printf("Error: %f\n", error);

        // Conserve brain resources
        wait(20, msec);
    }

    // Stop motors
    stopDrive();

    // Wait
    wait(500, msec);
}

/**
 * @brief Spin the accumulator in a direction
 * @param d The direction to spin the accumulator in
 * @param vel The velocity to spin the accumulator at
*/
void spinAccumulator(vex::directionType d, double vel)
{
    bottomAccumulator.spin(d, vel, percent);
    topAccumulator.spin(d, vel, percent);
}

/**
 * @brief Stops the accumulator.
 */
void stopAccumulator()
{
    bottomAccumulator.stop();
    topAccumulator.stop();
}