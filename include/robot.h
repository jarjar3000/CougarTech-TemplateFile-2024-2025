using namespace vex;

#include "robot-config.h"
#include "vex.h"

/**
 * A class that represents the robot.
 * @note Since there is only one robot, the class is only comprised of static data and methods.
 */
class robot
{
    private:
        static const double kP = 7;
        static const double kI = 3;
        static const double kD = 6;

        static const double turnKP = 0.6;
        static const double turnKI = 0.3;
        static const double turnKD = 0.5;

        static inline double error = 0;
        static inline double integral = 0;
        static inline double derivative = 0;
        static inline double prevError = 0;
        static inline double leftSpeed = 0;
        static inline double rightSpeed = 0;

        // Constants
        static const double DRIVE_INTEGRAL_WINDUP = 2;
        static const double TURN_INTEGRAL_WINDUP = 10;
        static const double DRIVE_ERROR_TOLERANCE = 0.5; // in inches
        static const double TURN_ERROR_TOLERANCE = 0.5;
        static const double WHEEL_DIAMETER = 3.25; // in inches
        static const double WHEEL_GEAR_RATIO = (double) 1 / 1;
        static inline double heading = 0;

    public:
        // Driving Variables
        static const double MAX_DRIVE_SPEED = 100;

        /*
            This boolean MUST be changed and the program must be redownloaded based on the alliance color.
        */
        static inline bool allianceIsRed = true;

        /**
         * @brief Function to set speed of left side of the drive
         * @param vel The speed to set the left side to.
         */
        static void setLeftSpeed(double vel)
        {
            leftB.setVelocity(vel, percent);
            leftF.setVelocity(vel, percent);
            leftE.setVelocity(vel, percent);
        }

        /**
         * @brief Function to set speed of right side of the drive
         * @param vel The speed to set the right side to.
         */
        static void setRightSpeed(double vel)
        {
            rightB.setVelocity(vel, percent);
            rightF.setVelocity(vel, percent);
            rightE.setVelocity(vel, percent);
        }

        /**
         * @brief Function to spin the drive motors in a direction
         * @param d The direction to spin the drive in.
         */
        static void drive(vex::directionType d)
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
        static void stopDrive()
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
        static void clamp()
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
        static void hang()
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
        static void toggleTipper()
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
        static void changeAllianceColor()
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
        static int eject()
        {
            // This should run the entire match, EXCEPT when we intake our first ring.
            // The first ring that passes the sensor will be the alliance color (since it's the preload), so this can dictate the alliance color.
            while (1)
            {
                controller1.Screen.clearScreen();
                controller1.Screen.setCursor(1, 1);
                controller1.Screen.print("Optical Hue: %.2f", optical1.hue());
                // Search for the color opposite the alliance color
                if ((optical1.hue() >= 100 && allianceIsRed) || (optical1.color() == red && !allianceIsRed))
                {
                    waitUntil(limit1.pressing());

                    controller1.rumble(".");

                    wait(50, msec);

                    // Spin in reverse
                    topAccumulator.setVelocity(0, percent);

                    // Wait until the ring ejects
                    wait(200, msec);

                    topAccumulator.setVelocity(80, percent);
                }
                // Sleep thread to not consume all of CPU resources
                // this_thread::sleep_for(20);
            }
            return 0;
        }

        /**
         * @brief Drive the robot in a direction for a certain distance in inches. Used in autonomous.
         * @param d The direction to drive in.
         * @param distance The distance the robot should drive in inches.
         * @param failsafeTime The amount of time that needs to pass before the robot automatically stops moving.
         */
        static void drive(vex::directionType d, double distance, double failsafeTime)
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
            setRightSpeed(80);
            setLeftSpeed(80);
            
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
        static void turn(vex::turnType d, double deg, double failsafeTime)
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
                error = (deg - fabs(inertial1.rotation(degrees)));
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
        static void spinAccumulator(vex::directionType d, double vel)
        {
            bottomAccumulator.spin(d, vel, percent);
            topAccumulator.spin(d, vel, percent);
        }

        /**
         * @brief Stops the accumulator.
         */
        static void stopAccumulator()
        {
            bottomAccumulator.stop();
            topAccumulator.stop();
        }
};