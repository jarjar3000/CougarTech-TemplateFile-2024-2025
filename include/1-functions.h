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
    switch (d)
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
        clamp2.set(true);
    }
    else
    {
        clamp1.set(false);
        clamp2.set(false);
    }
}

/**
 * @brief Function to toggle hanging mechanism. Starting position is down.
 */
void hang()
{
    if (hang1.value() == 0)
    {
        hang1.set(true);
        hang2.set(true);
    }
    else
    {
        hang1.set(false);
        hang2.set(false);
    }
}

/**
 * @brief Function to toggle tipper. Starting position is up.
 */
void toggleTipper()
{
    if (tipper.value() == 0)
    {
        tipper.set(true);
    }
    else
    {
        tipper.set(false);
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
        controller1.Screen.clearScreen();
        controller1.Screen.setCursor(1, 1);
        // Search for the color opposite the alliance color
        if ((optical1.color() == blue && allianceIsRed) || (optical1.color() == red && !allianceIsRed))
        {
            controller1.Screen.print("It detected it");
            // Spin in reverse
            topAccumulator.setVelocity(-100, percent);

            // Wait until the ring ejects
            wait(5000, msec);

            topAccumulator.setVelocity(100, percent);
        }
        controller1.Screen.print("Not Detected");
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
    integral = 0;
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
    do
    {
        // Error (Proportional)
        avgPosition = fabs((leftF.position(degrees) + rightF.position(degrees)) / 2);

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

        // Conserve brain resources
        wait(20, msec);
    }
    while ((error < -DRIVE_ERROR_TOLERANCE || error > DRIVE_ERROR_TOLERANCE) && failsafe.time(seconds) <= failsafeTime);

    // Stop motors
    stopDrive();

    // Wait
    wait(500, msec);
}

/**
 * @brief Turns the robot in a direction for a specific amount of degrees
 * @param d The direction to turn in
 * @param deg The amount of degrees to turn in
 * @param failsafeTime Amount of time, in seconds, until the robot automatically stops the move
 */
void turn(vex::turnType d, double deg, double failsafeTime)
{
    // Reset PID variables
    inertial1.setRotation(0, degrees);
    integral = 0;
    prevError = 0;

    // Reset failsafe timer
    failsafe.clear();

    // Start the PID turn
    controller1.Screen.print("right before the loop");
    drive(forward);
    do
    {
        // Proportional (abs the rotation so both directions are the same)
        error = (deg - fabs(inertial1.rotation(degrees))) - 5;
        controller1.Screen.clearScreen();
        controller1.Screen.setCursor(0, 0);
        controller1.Screen.print("Error: %.2f", error);

        // Integral
        integral += error;

        if (integral > TURN_INTEGRAL_WINDUP)
        {
            integral = 0;
        }

        // Derivative
        derivative = error - prevError;
        prevError = error;

        // Change Motor Speed
        leftSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
        rightSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
        switch (d)
        {
        case vex::turnType::left:
            setLeftSpeed(-leftSpeed);
            setRightSpeed(rightSpeed);
            break;

        case vex::turnType::right:
            setLeftSpeed(leftSpeed);
            setRightSpeed(-rightSpeed);
            break;
        }
        wait(20, msec);
    } while ((error > TURN_ERROR_TOLERANCE || error < -TURN_ERROR_TOLERANCE) && failsafe.time(seconds) <= failsafeTime);

    // Stop the robot
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