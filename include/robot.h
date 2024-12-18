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
        static const double L_R_WHEEL_DISTANCE = 12.30394713; // IN INCHES!

        // The distance between the back tracking wheel and the center of the robot
        static const double BACK_WHEEL_DISTANCE = 3.160259389; // IN INCHES! 5.25

        // PID Variables
        static const double kP = 7;
        static const double kI = 3;
        static const double kD = 6;

        static const double turnKP = 12; // 0.2 works with no ocilation and other values at 0
        static const double turnKI = 0;
        static const double turnKD = 0;

        static const double PID_TIMESTEP = 0.02; // Measured in seconds

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
        static inline bool CALIBRATE = true;

    public:
        // Driving Variables
        static const double MAX_DRIVE_SPEED = 50;

        /*
            This boolean MUST be changed and the program must be redownloaded based on the alliance color.
        */
        static inline bool allianceIsRed = true;

        // Boolean to dictate if data should be printed to the screen (not via the thread)
        static inline bool PRINT_DATA = false;

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
                wait(20, msec);
            }
            return 0;
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
         * @brief A function intended to be ran in a seperate thrad that will calculate the position of the robot using odometry
         * @return 0, but if a value is returned, it means a fatal error has happened, since the thread should never end.
         * @note The robot's heading should be 0 in the positive X direction.
         */
        static int calculateRobotPosition()
        {
            // Initialize all variables (previous x, etc...)
            double leftEncoder = 0, rightEncoder = 0, backEncoder = 0; // These variables can start at 0 because the encoders start at 0 in the beginning
            double prevLeftEncoder = 0, prevRightEncoder = 0, prevBackEncoder = 0; // These variables can start at 0 because the encoders start at 0 in the beginning
            double sumBack = 0;

            while(true)
            {
                // Get and store encoder values
                leftEncoder = leftF.position(degrees);
                rightEncoder = rightF.position(degrees); 
                backEncoder = backWheel.position(degrees) * -1;

                // Get the robot's current heading using the encoders
                double deltaLeft = leftEncoder - prevLeftEncoder;
                double deltaRight = rightEncoder - prevRightEncoder;
                double deltaBack = backEncoder - prevBackEncoder;

                // Convert the degree values of the deltas to a linear measurement (inches) so it works with the heading equation
                deltaLeft = (WHEEL_DIAMETER * M_PI) * (deltaLeft / ENCODER_TICKS_PER_REVOLUTION);
                deltaRight = (WHEEL_DIAMETER * M_PI) * (deltaRight / ENCODER_TICKS_PER_REVOLUTION);
                deltaBack = (WHEEL_DIAMETER * M_PI) * (deltaBack / ENCODER_TICKS_PER_REVOLUTION); // Back doesn't have a gear ratio, it's a dead wheel

                // Calculate deltaHeading
                double deltaHeading = (deltaLeft - deltaRight) / L_R_WHEEL_DISTANCE; // Equation outputs RADIANS

                // Update heading and put it back in range 0-360
                robot::heading += deltaHeading; // Add to the running total heading
                if (!CALIBRATE)
                {
                    robot::heading = fmod(robot::heading, (2 * M_PI)); // Ensure heading wraps from 0-360 degrees (which is 0-2pi radians)
                }

                // Calculate deltaFwd and deltaStrafe
                double deltaFwd = (deltaRight + deltaLeft) / 2;
                double deltaStrafe = deltaBack - (BACK_WHEEL_DISTANCE * deltaHeading);

                // Calculate deltaX and deltaY linearly or arc-based
                double deltaX, deltaY;
                if (fabs(deltaHeading) < 1e-6 || false)
                {
                    deltaX = deltaFwd * cos(heading) + deltaStrafe * sin(heading);
                    deltaY = deltaStrafe * cos(heading) - deltaFwd * sin(heading);
                }
                // Arc now works within margins of error (The x seems to have a much larger error than the y)
                else
                {
                    deltaX = (deltaFwd / deltaHeading) * sin(deltaHeading) - (deltaStrafe / deltaHeading) * (1 - cos(deltaHeading));
                    deltaY = (deltaStrafe / deltaHeading) * sin(deltaHeading) - (deltaFwd / deltaHeading) * (1 - cos(deltaHeading));
                }
                
                // Update x and y 
                robot::x += (deltaX * cos(heading) + deltaY * sin(heading));
                robot::y += (deltaY * cos(heading) - deltaX * sin(heading));

                sumBack += deltaBack;
                                
                // Save current data to become previous data in the next iteration
                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;
                prevBackEncoder = backEncoder;

                // Printing data to the screen
                if (PRINT_DATA)
                {
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("X: %.2f, dX: %.2f, Y: %.2f, dY: %.2f", robot::x, deltaX, robot::y, deltaY);

                    Brain.Screen.setCursor(2, 1);
                    Brain.Screen.print("dL: %.2f, dR: %.2f, dB: %.2f", deltaLeft, deltaRight, deltaBack);

                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("dFwd: %.2f, dStr: %.2f", deltaFwd, deltaStrafe);

                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("Hdg: %.2f, dHdg: %.2f", heading * (180/M_PI), deltaHeading * (180/M_PI));
                    
                    Brain.Screen.setCursor(5, 1);
                    Brain.Screen.print("Back Wheel Distance: %f", sumBack / robot::heading);
                } 

                // Wait to not consume all of the CPU's resources
                wait(10, msec); // Refresh rate of 100Hz
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
            // Store the timestep in a variable
            const double timestep = 0.02; // In seconds

            // Make a local variable to store the heading in degrees (This shadows the other heading, which is in radians)
            // We fabs the heading here because it has already been calculated.
            double heading = fabs(robot::heading);

            // Change the targetHeading to radians, since we use radians internally
            targetHeading *= (M_PI / 180);

            // Initialize the PID variables
            double error = 0, integral = 0, derivative = 0, prevError = heading;
            double motorSpeed = 0;

            // Set motors to drive
            drive(forward);

            // Do the turn
            do
            {
                // Keep pulling the updated heading
                heading = fabs(robot::heading);

                // Proportional
                error = targetHeading - heading; // Raw input

                // Normalize error to [-pi, pi] (-180, 180)
                error = atan2(sin(error), cos(error)); // No need to negate since direction is already handled

                // Integral - only integrate when motor speed is less than 100 and the motor speed isn't the same sign as the error so integral doesn't wind
                if (!(motorSpeed >= 100 && (int) (-error / fabs(error)) == (int) (-motorSpeed / fabs(motorSpeed))))
                {
                    integral += (error * timestep);
                }

                // Derivative
                derivative = (error - prevError) / timestep;
                prevError = error;

                // Change motor speed
                motorSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
                setLeftSpeed(motorSpeed);
                setRightSpeed(-motorSpeed);

                // Data
                if (PRINT_DATA)
                {
                    Brain.Screen.setCursor(5, 1);
                    Brain.Screen.print("E: %.2f, I: %.2f, D: %.2f", error * turnKP, integral * turnKI, derivative * turnKD);

                    controller1.Screen.clearScreen();
                    controller1.Screen.setCursor(1,1);
                    controller1.Screen.print("%.2f, %.2f, %.2f", error * turnKP, integral * turnKI, derivative * turnKD);
                    controller1.Screen.setCursor(2,1);
                    controller1.Screen.print("%d", (motorSpeed >= 100 && (int) (-error / fabs(error)) == (int) (-motorSpeed / fabs(motorSpeed))));
                }

                // Don't consume all of the CPU's resources
                wait(timestep, seconds);
            } while (true); // If the derivative is near-0? Ideally, when the robot's speed is barely changing, the move is done

            // Stop motion
            stopDrive();
        }

        /**
         * @brief Function to turn towards a point, (x, y), on the field
         * @param targetX The X value of the point
         * @param targetY The Y value of the point
         */
        static void turnToPoint(double targetX, double targetY)
        {
            // Use atan2 to calculate the heading
            double targetHeading = -atan2(targetY - robot::y, targetX - robot::x);
            turnTo(targetHeading);
        }

        /**
         * @brief Function to go to a point, (x, y), on the field by turning towards it and driving to it
         * @param targetX The X value the robot should go to
         * @param targetY The Y value the robot should go to
         * @param reverse Whether the robot should drive in reverse to get to the point. Default is false
         */
        static void goTo(double targetX, double targetY, bool driveReverse = false)
        {
            // Turn to the target point. If reverse is true, turn towards the negation of the point, so the robot drives in reverse
            if (driveReverse)
            {
                turnToPoint(-targetX, -targetY);
                drive(reverse);
            }
            else
            {
                turnToPoint(targetX, targetY);
                drive(forward);
            }

            // Initialize the PID variables
            double error = 0, integral = 0, derivative = 0, prevError = heading;
            double motorSpeed = 0;

            // Calculate the required distance
            double targetDistance = sqrt(pow(targetX, 2) + pow(targetY, 2));
            const double timestep = 0.02; // In seconds
            
            // Drive towards the target point
            do
            {
                // Error
                double currentDistance = sqrt(pow(robot::x, 2) + pow(robot::y, 2));
                error = targetDistance - currentDistance;

                // Integral - only integrate when motor speed is less than 100 and the motor speed isn't the same sign as the error so integral doesn't wind
                if (!(motorSpeed >= 100 && (int) (-error / fabs(error)) == (int) (-motorSpeed / fabs(motorSpeed))))
                {
                    integral += (error * timestep);
                }

                // Derivative
                derivative = (error - prevError) / timestep;
                prevError = error;

                // Change motor speed based on the drive direction
                motorSpeed = error * turnKP + integral * turnKI + derivative * turnKD;
                if (driveReverse)
                {
                    motorSpeed *= -1;
                } 
                setLeftSpeed(motorSpeed);
                setRightSpeed(motorSpeed);

                // Don't hog CPU
                wait(timestep, seconds);
            } while (true);

            // Stop driving
            stopDrive();
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
                controller1.Screen.print("(%.2f, %.2f) %.2f°", x, y, heading * (180/M_PI));

                // Print Drivetrain and intake temperatures
                controller1.Screen.setCursor(2, 0);
                double avgDriveTemp = (leftF.temperature(fahrenheit) + leftB.temperature(fahrenheit) + rightF.temperature(fahrenheit) + rightB.temperature(fahrenheit)) / 4.0;
                double avgAccumulatorTemp = (topAccumulator.temperature(fahrenheit) + bottomLeftAccumulator.temperature(fahrenheit) + bottomRightAccumulator.temperature(fahrenheit)) / 3.0;
                controller1.Screen.print("D: %.2f°F. I: %.2f°F", avgDriveTemp, avgAccumulatorTemp);
                
                // Print Battery Percentage
                controller1.Screen.setCursor(3, 0);
                controller1.Screen.print("Battery: %d percent", Brain.Battery.capacity(percent));
                
                wait(100, msec);
            }
            return 1;
        }

        /**
         * @brief Set the robot's starting position and heading; Reset all the encoders
         * @param x The robot's starting X value
         * @param y The robot's starting Y value
         * @param heading The robot's starting heading, in degrees
         */
        static void init(double x, double y, double heading)
        {
            // Set the starting position
            robot::x = x;
            robot::y = y;
            robot::heading = heading * (M_PI / 180); // Convert deg to rad

            // Reset all of the encoders
            leftF.setPosition(0, degrees);
            rightF.setPosition(0, degrees);
            backWheel.setPosition(0, degrees);
        }
};