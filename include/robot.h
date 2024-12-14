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
        // Odometry Variables
        static inline double x = 0; // IN INCHES!
        static inline double y = 0; // IN INCHES!
        static inline double heading = 0; // IN RADIANS!

        // The distance between the right and left tracking wheels
        static const double L_R_WHEEL_DISTANCE = 12; // IN INCHES!

        // The distance between the back tracking wheel and the center of the robot
        static const double BACK_WHEEL_DISTANCE = 5.25; // IN INCHES!

        // PID Variables
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
        static const double ENCODER_TICKS_PER_REVOLUTION = 360; // Speed motor is 300, Normal is 900, Torque is 1800

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
        }

        /**
         * @brief Function to set speed of right side of the drive
         * @param vel The speed to set the right side to.
         */
        static void setRightSpeed(double vel)
        {
            rightB.setVelocity(vel, percent);
            rightF.setVelocity(vel, percent);
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
                rightF.spin(forward);
                rightB.spin(forward);
                break;

            case vex::directionType::rev:
                leftF.spin(reverse);
                leftB.spin(reverse);
                rightF.spin(reverse);
                rightB.spin(reverse);
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
            rightF.stop();
            rightB.stop();
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
                error = distance - (WHEEL_DIAMETER * M_PI) * (avgPosition / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_GEAR_RATIO;

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
            drive(forward);
            do
            {
                // Proportional (abs the rotation so both directions are the same)
                error = (deg - fabs(inertial1.rotation(degrees)));

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
            bottomLeftAccumulator.spin(d, vel, percent);
            bottomRightAccumulator.spin(d, vel, percent);
            topAccumulator.spin(d, vel, percent);
        }

        /**
         * @brief Stops the accumulator.
         */
        static void stopAccumulator()
        {
            bottomLeftAccumulator.stop();
            bottomRightAccumulator.stop();
            topAccumulator.stop();
        }

        /**
         * @brief A function that sets the motors to spin to turn in a direction
         * @param d The direction to turn in
         */
        static void turn(vex::turnType d)
        {
            switch(d)
            {
                case vex::turnType::left:
                leftF.spin(reverse);
                leftB.spin(reverse);
                rightF.spin(forward);
                rightB.spin(forward);
                break;

                case vex::turnType::right:
                leftF.spin(forward);
                leftB.spin(forward);
                rightF.spin(reverse);
                rightB.spin(reverse);
                break;
            }
        }

        /**
         * @brief A function intended to be ran in a seperate thrad that will calculate the position of the robot using odometry
         * @return 0, but if a value is returned, it means a fatal error has happened, since the thread should never end.
         * @note The robot's heading should be 0 in the positive Y direction.
         */
        static int calculateRobotPosition()
        {
            // Initialize all variables (previous x, etc...)
            double leftEncoder = 0, rightEncoder = 0, backEncoder = 0; // These variables can start at 0 because the encoders start at 0 in the beginning
            double prevLeftEncoder = 0, prevRightEncoder = 0, prevBackEncoder = 0; // These variables can start at 0 because the encoders start at 0 in the beginning
            Brain.resetTimer();

            while(true)
            {
                // Get and store encoder values
                leftEncoder = leftF.position(degrees);
                rightEncoder = rightF.position(degrees); // Flip signs
                backEncoder = backWheel.position(degrees); // Negative?

                // Get the robot's current heading using the encoders
                double deltaLeft = leftEncoder - prevLeftEncoder;
                double deltaRight = rightEncoder - prevRightEncoder;
                double deltaBack = backEncoder - prevBackEncoder;

                // Convert the degree values of the deltas to a linear measurement (inches) so it works with the heading equation
                deltaLeft = (WHEEL_DIAMETER * M_PI) * (deltaLeft / ENCODER_TICKS_PER_REVOLUTION);
                deltaRight = (WHEEL_DIAMETER * M_PI) * (deltaRight / ENCODER_TICKS_PER_REVOLUTION);
                deltaBack = (WHEEL_DIAMETER * M_PI) * (deltaBack / ENCODER_TICKS_PER_REVOLUTION); // Back doesn't have a gear ratio, it's a dead wheel

                // Equation takes in linear unit (in) and radian value and outputs a distance
                double deltaHeading = (deltaRight - deltaLeft) / L_R_WHEEL_DISTANCE; // Equation outputs RADIANS
                robot::heading += deltaHeading; // Add to the running total heading

                // Calculate deltaFwd and deltaStrafe
                double deltaFwd = (deltaRight + deltaLeft) / 2;
                double deltaStrafe = deltaBack - (BACK_WHEEL_DISTANCE * deltaHeading);

                // PROBLEM
                // Calculate deltaX and deltaY

                double deltaX, deltaY;

                if (fabs(deltaHeading) < 1e-6)
                {
                    deltaX = deltaFwd * cos(heading) - deltaStrafe * sin(heading);
                    deltaY = deltaStrafe * cos(heading) + deltaFwd * sin(heading);
                }   
                else
                {
                    deltaX = (deltaFwd / deltaHeading) * sin(heading) - (deltaStrafe - deltaHeading) * (1 - cos(deltaHeading));
                    deltaY = (deltaStrafe / deltaHeading) * sin(heading) - (deltaFwd - deltaHeading) * (1 - cos(deltaHeading));
                }
                

                // Update x and y
                double rotatedX = (deltaX * cos(heading) - deltaY * sin(heading));
                double rotatedY = (deltaY * cos(heading) + deltaX * sin(heading));

                // PROBLEM

                robot::x += rotatedX;
                robot::y += rotatedY;
                
                // Save current data to become previous data in the next iteration
                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;
                prevBackEncoder = backEncoder;

                // Brain 
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("X: %.2f, dX: %.2f, Y: %.2f, dY: %.2f", robot::x, deltaX, robot::y, deltaY);

                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("dL: %.2f, dR: %.2f, dB: %.2f", deltaLeft, deltaRight, deltaBack);

                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("Rotated X: %.2f, Rotated Y: %.2f", rotatedX, rotatedY);

                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("dFwd: %.2f, dStr: %.2f", deltaFwd, deltaStrafe);

                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("Hdg: %.2f, dHdg: %.2f", heading * (180/M_PI), deltaHeading * (180/M_PI));

                // Clear the screen
                controller1.Screen.clearScreen();

                // Set the cursor for printing
                controller1.Screen.setCursor(1, 0);

                // Print X, Y, and Heading Values
                controller1.Screen.print("x: %.2f, dX: %.2f", robot::x, deltaX);

                // Set the cursor for printing
                controller1.Screen.setCursor(2, 0);

                // Print X, Y, and Heading Values
                controller1.Screen.print("dL: %.2f, dR: %.2f", deltaLeft, deltaRight);

                // Wait to not consume all of the CPU's resources
                wait(20, msec); // Refresh rate of 100Hz
            }

            // If a value is return, something bad happened
            return 1;
        }

        /**
         * @brief Function to turn to a heading, using the absolute robot heading.
         * @param targetHeading The heading to turn to, in degrees.
         */
        static void turnTo(double targetHeading)
        {
            // Initialize the PID variables
            double error = 0, integral = 0, derivative = 0, prevError = heading;
            double motorSpeed = 0;
            
            error = targetHeading - heading;
            
            /*
                This attempts to counter the issue when turning to heading 0. 
                Scenario 1: when turning from heading 270 to 0, the current error calculation would put it at 270.
                This edge case needs to be solved. It will occur whenever heading 0 is involved in the turn.
            */
            if (error > 180)
            {
                error -= 180;
            }

            // Determine the direction to turn
            // Calculate with both a right and left turn. Whichever value is less is the one we want to use.
            // If the values are equal (180 degree turn), turn right.
            if (error < 0)
            {
                // Turn left
                turn(left);
            }
            else
            {
                // Turn right
                turn(right);
            }

            // Do the turn
            do
            {
                // Proportional
                error = targetHeading - heading;

                // Integral - only integrate when motor speed is less than 100 so integral doesn't wind
                if (motorSpeed <= 100)
                {
                    integral += error;
                }

                // Derivative
                derivative = error - prevError;
                prevError = error;

                // Change motor speed
                motorSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
                setLeftSpeed(motorSpeed);
                setRightSpeed(motorSpeed);

            } while (true);
        }

        /**
         * @brief Function to go to a point, (x, y), on the field
         * @param targetX The X value the robot should go to
         * @param targetY The Y value the robot should go to
         * @param reverse Whether the robot should drive in reverse to get to the point. Default is false
         */
        static void goTo(double targetX, double targetY, bool reverse = false)
        {

        }
        
        /**
         * @brief Function intended to be run in a thread to print helpful information to the controller screen.
         */
        static int printInfoToController()
        {
            while(true)
            {
                // Clear the screen
                controller1.Screen.clearScreen();

                // Set the cursor for printing
                controller1.Screen.setCursor(1, 0);

                // Print X, Y, and Heading Values
                controller1.Screen.print("X: %.2f. Y: %.2f. Hdg: %.2f\n", x, y, heading);

                // Print Drivetrain and intake temperatures
                controller1.Screen.setCursor(2, 0);
                controller1.Screen.print("Hdg: %.2f", heading * (180/M_PI));
                // controller1.Screen.print("DT: %.2f. IT: %.2f\n", leftF.temperature(fahrenheit), topAccumulator.temperature(fahrenheit));
                
                // Print Battery Percentage
                controller1.Screen.setCursor(3, 0);
                controller1.Screen.print("Battery: %.2f\%", Brain.Battery.capacity(percent));
                
                wait(20, msec);
            }
            return 1;
        }
};