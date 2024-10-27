using namespace vex;

#include "0-variables.h"
#include "robot-config.h"

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

        // printf("Error: %f\n", error);

        // Break if desination is reached AND failsafe timer is greater than input time
        if ((error >= -DRIVE_ERROR_TOLERANCE && error <= DRIVE_ERROR_TOLERANCE) || failsafe.time(seconds) >= failsafeTime)
        {
            break;
        }

        // Conserve brain resources
        wait(2, msec);
    }

    // Stop motors
    leftF.stop();
    leftB.stop();
    rightF.stop();
    rightB.stop();

    // Wait
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

// Move arm for certain number of degrees, at a certain velocity
void liftArm(armDirections d, double deg, double vel, bool waitForCompletition=true)
{
    leftLift.setVelocity(vel, percent);
    rightLift.setVelocity(vel, percent);
    switch(d)
    {
        case armDirections::up:
        leftLift.spinFor(forward, deg, degrees, false);
        rightLift.spinFor(forward, deg, degrees, waitForCompletition);
        break;
        
        case armDirections::down:
        leftLift.spinFor(reverse, deg, degrees, false);
        rightLift.spinFor(reverse, deg, degrees, waitForCompletition);
        break;
    }
}
