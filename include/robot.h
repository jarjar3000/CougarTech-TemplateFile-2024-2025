using namespace vex;

#include "robot-config.h"
#include "vex.h"
#include "matrix.h"
#include <deque>

/**
 * A class that represents the robot.
 * @note Since there is only one robot, the class is only comprised of static data and methods.
 */

class robot
{
    private:
        // Odometry Variables
        static inline double x; // IN INCHES!
        static inline double y; // IN INCHES!
        static inline double heading; // IN RADIANS!

        // Struct to represent an (x,y) point
        struct Point
        {
            double x, y;
        };

        // Lookahead distance for pure pursiut
        static const double LOOKAHEAD_DISTANCE = 5; // IN INCHES!

        // The distance the wheels are from the center of rotation
        static const double LEFT_WHEEL_DISTANCE = 3.208095654; // POSITIVE VALUE, IN INCHES!
        static const double RIGHT_WHEEL_DISTANCE = 0.7403297662; // POSITIVE VALUE, IN INCHES!

        // The distance between the back tracking wheel and the center of the robot
        static const double BACK_WHEEL_DISTANCE = 2.333; // IN INCHES! 5.25

        // Complementary filter tuning value, between 0 and 1. Values closer to 1 represents a greater trust in the odometry
        static const double ALPHA = 0.6;

        // PID Variables
        static const double kP = 5; //4.1
        static const double kI = 0.1; // 1
        static const double kD = 0.1;

        static const double turnKP = 24; // 36
        static const double turnKI = 3;
        static const double turnKD = 2;;

        // Straight PID Constants
        static const double straightKP = 0.5;
        static const double straightKI = 0.0;
        static const double straightKD = 0.0;

        static inline double error = 0;
        static inline double integral = 0;
        static inline double derivative = 0;
        static inline double prevError = 0;
        static inline double leftSpeed = 0;
        static inline double rightSpeed = 0;

        // Constants
        static const double DRIVE_ERROR_TOLERANCE = 1; // in inches
        static const double TURN_ERROR_TOLERANCE = 1.5 * (M_PI / 180); // in RADIANS!
        static const double PID_TIMESTEP = 0.02; // Measured in seconds
        static const double TIME_STABLE_TO_BREAK = 0.25; // Measured in seconds
        static const double VELOCITY_STABLE_TO_BREAK = 20; // Measured in percent

        static const double WHEEL_DIAMETER = 2; // in inches
        static const double WHEEL_GEAR_RATIO = (double) 1 / 1;
        static const double ENCODER_TICKS_PER_REVOLUTION = 360; // Speed motor is 300, Normal is 900, Torque is 1800

        // Degree of LIP calculations
        static const int DEGREE_OF_LIP_POLYNOMIAL = 3;

        // Odom or Inertial Only
        static const bool ODOM_ONLY = false;

        // Use Kalman Filter
        static const bool KALMAN_FILTER = true;

        // Kalman Filter Variables
        static inline double odomUncertainty = 3; // Tune this, lower represents greater trust
        static inline double inertialUncertainty = 3; // lower is greater trust
        static inline double stateUncertainty = 5; // lower is greater trust

        /**
         * @brief Calculate the Lagrange Interpolating Polynomial that passes through (x, y) points and returns result given an input
         * @param x An array holding the x values of the points
         * @param y An array holding the y values of the points
         * @param input The value to put into the LIP
         * @return The Y value (output) of the given X (input)
         */
        static double calculateLIP(const double x[], const double y[], double input)
        {
            // Make a variable to hold the output
            double output = 0;

            // Sum of capital PI (products)
            for (int i = 0; i < DEGREE_OF_LIP_POLYNOMIAL; i++)
            {
                // Find polynomial that passes through one point and is 0 at others
                double term = y[i];
                for (int j = 0; j < DEGREE_OF_LIP_POLYNOMIAL; j++)
                {
                    if (j != i)
                    {
                        term *= (input - x[i]) / (x[j] - x[i]);
                    }
                }
                // Add result of repeated multiplication
                output += term;
            }

            return output;
        }

        /**
         * Function to find and return the sign of a number
         * @param num The number to find the sign of
         * @return the sign of the number
         */
        static int sign(double num)
        {
            if (num < 0)
            {
                return -1;
            }
            else
            {
                return 1;
            }
        }

    public:
        // Is the robot calibrating?
        static const bool CALIBRATE = false;
        
        // Driving Variables
        static const double MAX_DRIVE_SPEED = 100;
        static const double MAX_TOP_ACCUMULATOR_SPEED = 100;

        /*
            This boolean MUST be changed and the program must be redownloaded based on the alliance color.
        */
        static inline bool allianceIsRed = true;

        // Boolean to dictate if data should be printed to the screen (not via the thread)
        static inline bool PRINT_DATA = false;

        // Blue and Red thresholds
        static const double OPTICAL_BLUE_HUE = 200; // Hues above this number are blue
        static const double OPTICAL_RED_HUE = 30; // Hues below this number are red

        // Toggle of if fish mech is active
        static inline bool armActive = false;

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
        static void intakeLift()
        {
            if (intakeLift1.value() == 0)
            {
                intakeLift1.set(true);
            }
            else
            {
                intakeLift1.set(false);
            }
        }

        /**
         * @brief Function to toggle intake lifter. Starting position is up.
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
            }
            else
            {
                hang1.set(false);
            }
        }

        /**
         * @brief Function to toggle arm mode
         */
        static void armStopper()
        {
            if (armActive)
            {
                controller1.rumble(".");
                armActive = false;
            }
            else
            {
                controller1.rumble("...");
                armActive = true;
            }
        }

        /**
         * @brief Function to toggle alliance color.
         */
        static void toggleAllianceColor()
        {
            if (robot::allianceIsRed)
            {
                controller1.rumble("....");
                robot::allianceIsRed = false;
            }
            else
            {
                controller1.rumble("....");
                robot::allianceIsRed = true;
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
                if (((optical1.hue() >= OPTICAL_BLUE_HUE && allianceIsRed) || (optical1.hue() <= OPTICAL_RED_HUE && !allianceIsRed)) && !armActive)
                {
                    controller1.rumble(".");
                    waitUntil(limit1.pressing());

                    controller1.rumble(".");

                    wait(175, msec);

                    // Spin in reverse
                    topAccumulator.setVelocity(0, percent);

                    // Wait until the ring ejects
                    wait(200, msec);

                    topAccumulator.setVelocity(MAX_TOP_ACCUMULATOR_SPEED, percent);
                }
                // Sleep thread to not consume all of CPU resources
                wait(20, msec);
            }
            return 0;
        }

        /**
         * @brief Thread function to stop rings at the right place for the arm at the toggle of a button
         */
        static int armRingStop()
        {
            while (1)
            {
                if (armActive && limit1.pressing()) //12
                {
                    // wait(65, msec);
                    controller1.rumble("......");
                    topAccumulator.setVelocity(0, percent);

                    waitUntil(!armActive || controller1.ButtonL1.pressing() || controller1.ButtonL2.pressing());
                    topAccumulator.setVelocity(MAX_TOP_ACCUMULATOR_SPEED, percent);
                    armActive = false;
                }
                wait(20, msec);
            }
        }

        /**
         * @brief Spin the accumulator in a direction
         * @param d The direction to spin the accumulator in
         * @param vel The velocity to spin the accumulator at
         */
        static void spinAccumulator(vex::directionType d, double vel = 100)
        {
            bottomAccumulatorL.spin(d, vel, percent);
            bottomAccumulatorR.spin(d, vel, percent);
            topAccumulator.spin(d, vel, percent);
        }

        /**
         * @brief Stops the accumulator.
         */
        static void stopAccumulator()
        {
            bottomAccumulatorL.stop();
            bottomAccumulatorR.stop();
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

            // Previous values used for LIP calculation
            double prevTime[DEGREE_OF_LIP_POLYNOMIAL];
            double prevDeltaX[DEGREE_OF_LIP_POLYNOMIAL];
            double prevDeltaY[DEGREE_OF_LIP_POLYNOMIAL];
            double prevHeading[DEGREE_OF_LIP_POLYNOMIAL];

            positionCalculationTimer.clear();
            
            // Calculate the first few points for LIP
            while(true)
            {
                // Get and store encoder values
                leftEncoder = leftTracking.position(degrees);
                rightEncoder = rightTracking.position(degrees); 
                backEncoder = centerTracking.position(degrees);

                // Get the robot's current heading using the encoders
                double deltaLeft = leftEncoder - prevLeftEncoder;
                double deltaRight = rightEncoder - prevRightEncoder;
                double deltaBack = backEncoder - prevBackEncoder;

                // Convert the degree values of the deltas to a linear measurement (inches) so it works with the heading equation
                deltaLeft = (WHEEL_DIAMETER * M_PI) * (deltaLeft / ENCODER_TICKS_PER_REVOLUTION);
                deltaRight = (WHEEL_DIAMETER * M_PI) * (deltaRight / ENCODER_TICKS_PER_REVOLUTION);
                deltaBack = (WHEEL_DIAMETER * M_PI) * (deltaBack / ENCODER_TICKS_PER_REVOLUTION); 

                // Calculate deltaHeading
                double deltaHeading = (deltaLeft - deltaRight) / (LEFT_WHEEL_DISTANCE + RIGHT_WHEEL_DISTANCE); // Equation outputs RADIANS

                // Special version for calibration to tune odometry wheel distances
                if (CALIBRATE)
                {
                    robot::heading += deltaHeading;
                }
                // Odometry Only
                else if (ODOM_ONLY && !KALMAN_FILTER)
                {
                    // Add to running total
                    robot::heading += deltaHeading;

                    // Normalize the heading
                    robot::heading = fmod(robot::heading, (2 * M_PI)); 
                    if (robot::heading < 0) 
                    {
                        heading += 2 * M_PI; // Make sure negative headings properly wrap into range
                    }
                }
                // Inertial Only
                else if (!ODOM_ONLY && !KALMAN_FILTER)
                {
                    // Merge inertial sensor data
                    double inertialHeading = inertial1.heading(degrees) * (M_PI / 180);
                    robot::heading = inertialHeading;

                    // Normalize the heading
                    robot::heading = fmod(robot::heading, (2 * M_PI)); 
                    if (robot::heading < 0) 
                    {
                        heading += 2 * M_PI; // Make sure negative headings properly wrap into range
                    }
                }
                // Kalman Filter
                else
                {
                    // Predict next heading with odometry
                    robot::heading += deltaHeading;

                    // Normalize the heading
                    robot::heading = fmod(robot::heading, (2 * M_PI)); 
                    if (robot::heading < 0) 
                    {
                        heading += 2 * M_PI; // Make sure negative headings properly wrap into range
                    }

                    // Predict state uncertainty
                    stateUncertainty += odomUncertainty;
                    
                    // Calculate Kalman Gain
                    double kalmanGain = stateUncertainty / (stateUncertainty + inertialUncertainty);

                    // Merge inertial sensor data
                    double inertialHeading = inertial1.heading(degrees) * (M_PI / 180);
                    robot::heading += kalmanGain * (inertialHeading - robot::heading);

                    // Normalize the heading again
                    robot::heading = fmod(robot::heading, (2 * M_PI)); 
                    if (robot::heading < 0) 
                    {
                        heading += 2 * M_PI; // Make sure negative headings properly wrap into range
                    }

                    // Update state uncertainty
                    stateUncertainty *= (1 - kalmanGain);
                }

                // Calculate deltaFwd and deltaStrafe
                double deltaFwd = (deltaLeft * LEFT_WHEEL_DISTANCE + deltaRight * RIGHT_WHEEL_DISTANCE) / (LEFT_WHEEL_DISTANCE + RIGHT_WHEEL_DISTANCE);
                double deltaStrafe = deltaBack - (BACK_WHEEL_DISTANCE * deltaHeading);

                // Calculate deltaX and deltaY linearly or arc-based
                double deltaX, deltaY;
                if (fabs(deltaHeading) < 1e-3)
                {
                    deltaX = deltaFwd;
                    deltaY = deltaStrafe;
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
         * @brief Function to drive straight for a certain distance
         * @param direction The direction to drive in
         * @param distance The distance to drive in inches
         */
        static void driveStraight(vex::directionType direction, double distance)
        {
            // Initialize the PID variables
            double error = 0, integral = 0, derivative = 0, prevError = 0;
            double motorSpeed = 0;
            double totalDistanceMoved = 0;

            // Encoder values
            double leftEncoder = 0, rightEncoder = 0, backEncoder = 0;
            double prevLeftEncoder = 0, prevRightEncoder = 0, prevBackEncoder = 0;
            
            // Set heading to hold
            double targetHeading = robot::heading;

            // reset the timer
            Brain.resetTimer();

            // Reset the tracking wheel encoders
            leftTracking.setPosition(0, degrees);
            rightTracking.setPosition(0, degrees);
            centerTracking.setPosition(0, degrees);

            // Set motors to drive
            switch(direction)
            {
                case vex::directionType::fwd:
                    drive(forward);
                    break;
                case vex::directionType::rev:
                    drive(reverse);
                    break;
                case vex::directionType::undefined:
                    break;
            }

            do 
            {
                // Get and store encoder values
                leftEncoder = leftTracking.position(degrees);
                rightEncoder = rightTracking.position(degrees); 
                backEncoder = centerTracking.position(degrees);

                // Get the robot's current heading using the encoders
                double deltaLeft = leftEncoder - prevLeftEncoder;
                double deltaRight = rightEncoder - prevRightEncoder;
                double deltaBack = backEncoder - prevBackEncoder;

                // Convert the degree values of the deltas to a linear measurement (inches) so it works with the heading equation
                deltaLeft = (WHEEL_DIAMETER * M_PI) * (deltaLeft / ENCODER_TICKS_PER_REVOLUTION);
                deltaRight = (WHEEL_DIAMETER * M_PI) * (deltaRight / ENCODER_TICKS_PER_REVOLUTION);
                deltaBack = (WHEEL_DIAMETER * M_PI) * (deltaBack / ENCODER_TICKS_PER_REVOLUTION); // Back doesn't have a gear ratio, it's a dead wheel

                double deltaFwd = (deltaLeft * LEFT_WHEEL_DISTANCE + deltaRight * RIGHT_WHEEL_DISTANCE) / (LEFT_WHEEL_DISTANCE + RIGHT_WHEEL_DISTANCE);
                totalDistanceMoved += deltaFwd;

                // Proportional
                error = distance - fabs(totalDistanceMoved);

                // Integral - only integrate when motor speed is less than 100 and the motor speed isn't the same sign as the error so integral doesn't wind
                if (!(motorSpeed >= 100 && (int) (-error / fabs(error)) == (int) (-motorSpeed / fabs(motorSpeed))))
                {
                    integral += (error * PID_TIMESTEP);
                }

                // Derivative
                derivative = (error - prevError) / PID_TIMESTEP;
                prevError = error;

                // Change motor speed and angluar velocity
                motorSpeed = error * kP + integral * kI + derivative * kD;
                switch (direction)
                {
                    case vex::directionType::fwd:
                        setLeftSpeed(motorSpeed);
                        setRightSpeed(motorSpeed);
                        break;
                    case vex::directionType::rev:
                        setLeftSpeed(-motorSpeed);
                        setRightSpeed(-motorSpeed);
                        break;
                    case vex::directionType::undefined:
                        break;
                }

                // Print info on the screen (always)
                controller1.Screen.setCursor(1, 1);
                controller1.Screen.print("E: %.2f, I: %.2f, D: %.2f", error, integral, derivative);

                // Check for completion with error check and velocity check.
                if (fabs(error) < DRIVE_ERROR_TOLERANCE)
                {
                    break;
                }

                // Set current encoder values to previous
                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;
                prevBackEncoder = backEncoder;

                wait(PID_TIMESTEP, seconds);
            }
            while(true);

            // Stop the motors
            controller1.rumble("...");
            stopDrive();
        }

        /**
         * @brief Function to turn to a heading, using the absolute robot heading.
         * @param targetHeading The heading to turn to, in DEGREES.
         */
        static void turnToHeading(double targetHeading)
        {
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
                    integral += (error * PID_TIMESTEP);
                }

                // Derivative
                derivative = (error - prevError) / PID_TIMESTEP;
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

                // Check for completion with error check and velocity check.
                if (fabs(error) < TURN_ERROR_TOLERANCE)
                {
                    break;
                }

                // Don't consume all of the CPU's resources
                wait(PID_TIMESTEP, seconds);
            } while (true); 

            // Stop motion
            stopDrive();
            
        }
        
        /**
         * @brief Function to turn towards a point, (x, y), on the field
         * @param targetX The X value of the point
         * @param targetY The Y value of the point
         * @param reverse Whether the robot should turn its back to the point. Default is false
         */
        static void turnToPoint(double targetX, double targetY, bool reverse = false)
        {
            // Use atan2 to calculate the heading
            double targetHeading = -atan2(targetY - robot::y, targetX - robot::x); // This outputs radians

            // If reverse is true, adjust the target heading by 180 degrees (pi radians) 
            if (reverse)
            {
                targetHeading += M_PI;
            }

            // Normalize the target heading to the range
            targetHeading = atan2(sin(targetHeading), cos(targetHeading));

            // Only turn if we need to
            if (fabs(robot::heading - targetHeading) >= TURN_ERROR_TOLERANCE)
            {
                turnToHeading(targetHeading * (180 / M_PI)); // This function takes in degrees
            }
        }

        /**
         * @brief Function to go to a point, (x, y), on the field by turning towards it and driving to it
         * @param targetX The X value the robot should go to
         * @param targetY The Y value the robot should go to
         * @param reverse Whether the robot should drive in reverse to get to the point. Default is false
         */
        static void goTo(double targetX, double targetY, bool driveReverse = false)
        {
            // Set motor speed to 80
            setLeftSpeed(80);
            setRightSpeed(80);
            
            // Turn to the target point. If reverse is true, call turn to point with the reverse parameter as true
            if (driveReverse)
            {
                turnToPoint(targetX, targetY, true);
                drive(reverse);
            }
            else
            {
                turnToPoint(targetX, targetY, false);
                drive(forward);
            }

            // Initialize the PID variables
            double error = 0, integral = 0, derivative = 0, prevError = 0;
            double strError = 0, strIntegral = 0, strDerivative = 0, strPrevError = 0;
            double motorSpeed = 0, strMotorSpeed = 0;

            // Calculate the required distance
            double targetDistance = sqrt(pow(targetX - robot::x, 2) + pow(targetY - robot::y, 2));

            // Get the heading that is required to go to the point (the current heading)
            double targetHeading = robot::heading;

            // Clear the brain timer
            Brain.resetTimer();
            
            // Drive towards the target point
            do
            {
                // Preverse signs in x and y difference to allow for negative distances so the PID works properly

                double diffX = sign(targetX - robot::x) * pow(targetX - robot::x, 2);
                double diffY = sign(targetY - robot::y) * pow(targetY - robot::y, 2);

                // Get current distance from the point 
                error = sqrt(diffX + diffY);

                // Integral - only integrate when motor speed is less than 100 and the motor speed isn't the same sign as the error so integral doesn't wind
                if (!(motorSpeed >= 100 && (int) (-error / fabs(error)) == (int) (-motorSpeed / fabs(motorSpeed))))
                {
                    integral += (error * PID_TIMESTEP);
                }

                // Derivative
                derivative = (error - prevError) / PID_TIMESTEP;
                prevError = error;

                // Calculate an error, integral, and derivative for the heading to prevent deviation from the path
                strError = targetHeading - robot::heading;

                // Normalize the heading error to [-pi, pi]
                strError = atan2(sin(strError), cos(strError));

                // Integral - only integrate when motor speed is less than 100 and the motor speed isn't the same sign as the error so integral doesn't wind
                if (!(motorSpeed >= 100 && (int) (-strError / fabs(strError)) == (int) (-motorSpeed / fabs(motorSpeed))))
                {
                    strIntegral += (strError * PID_TIMESTEP);
                }

                // Derivative
                strDerivative = (strError - strPrevError) / PID_TIMESTEP;
                strPrevError = strError;

                // Change motor speed based on the drive direction
                motorSpeed = (error * kP + integral * kI + derivative * kD);
                strMotorSpeed = (strError * straightKP + strIntegral * straightKI + strDerivative * straightKD);
                if (driveReverse)
                {
                    motorSpeed *= -1;
                } 

                // Print the error to the controller screen
                if (true)
                {
                    // Print error and robot position
                    controller1.Screen.clearScreen();
                    controller1.Screen.setCursor(1, 1);
                    controller1.Screen.print("E: %.2f, X: %.2f, Y: %.2f", error, x, y);
                }

                // Calculate left and right side speeds
                double leftMotorSpeed = motorSpeed + strMotorSpeed;
                double rightMotorSpeed = motorSpeed - strMotorSpeed;

                // Saturation protection
                double maxSpeed = std::max(fabs(leftMotorSpeed), fabs(rightMotorSpeed));
                if (maxSpeed > 100) {
                    leftMotorSpeed = (leftMotorSpeed / maxSpeed) * 100;
                    rightMotorSpeed = (rightMotorSpeed / maxSpeed) * 100;
                }

                // Set the motor speeds
                setLeftSpeed(leftMotorSpeed);
                setRightSpeed(rightMotorSpeed);

                // Check for completion with error check and velocity check.
                if (fabs(error) < DRIVE_ERROR_TOLERANCE && fabs(derivative) < VELOCITY_STABLE_TO_BREAK)
                {
                    // Use the brain's timer to see if the condition above has been held for sufficient length
                    if (Brain.Timer.value() > TIME_STABLE_TO_BREAK)
                    {
                        break;
                    }
                }
                else
                {
                    // Clear the timer for use when within margins
                    Brain.resetTimer();
                }

                // Don't hog CPU
                wait(PID_TIMESTEP, seconds);
            } while (true);

            // Stop driving
            stopDrive();
            controller1.rumble(".");
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

                char armStatus;
                if (armActive)
                {
                    armStatus = 'Y';
                }
                else
                {
                    armStatus = 'N';
                }

                char allianceColor;
                if (allianceIsRed)
                {
                    allianceColor = 'R';
                }
                else
                {
                    allianceColor = 'B';
                }

                // Print X, Y, and Heading Values
                controller1.Screen.print("(%.2f, %.2f) %.2f°", x, y, heading * (180/M_PI));

                // Print Drivetrain and intake temperatures
                controller1.Screen.setCursor(2, 0);
                double avgDriveTemp = (leftF.temperature(fahrenheit) + leftB.temperature(fahrenheit) + rightF.temperature(fahrenheit) + rightB.temperature(fahrenheit)) / 4.0;
                double avgAccumulatorTemp = (topAccumulator.temperature(fahrenheit) + bottomAccumulatorL.temperature(fahrenheit)) / 2.0;
                controller1.Screen.print("D: %.2f°F. I: %.2f°F", avgDriveTemp, avgAccumulatorTemp);
                
                // Print Battery Percentage
                controller1.Screen.setCursor(3, 0);
                controller1.Screen.print("By: %d, Arm: %c, AC: %c", Brain.Battery.capacity(percent), armStatus, allianceColor);
                
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
            leftTracking.setPosition(0, degrees);
            rightTracking.setPosition(0, degrees);
            centerTracking.setPosition(0, degrees);

            // Calibrate the inertial
            inertial1.calibrate();
            while(inertial1.isCalibrating())
            {
                controller1.rumble(".");
                wait(100, msec);
            }
            inertial1.setHeading(heading, degrees);
        }

        /**
         * @brief Function to follow a set of points
         * @param points The list of points to follow
         */
        static void followPath(std::deque<Point> points)
        {
            // Goal Point
            Point goalPoint;

            // Last found index
            int lastFoundIndex;

            // Prev error for PID
            double prevError = 0;

            // Spin the motors forward
            drive(forward);

            // Loop dictates PID and path following
            while (1)
            {
                for (int i = 0; i < points.size() - 1; i++)
                {
                    // Find the intersection with the path

                    // Transform robot position to be at origin for easier math
                    double x1 = robot::x - points[i].x;
                    double x2 = robot::x - points[i+1].x;
                    double y1 = robot::y - points[i].y;
                    double y2 = robot::y - points[i+1].y;

                    // Find variables
                    double dx = x2 - x1;
                    double dy = y2 - y1;
                    double dr = sqrt(pow(dx, 2) + pow(dy, 2));
                    double D = x1 * y2 - x2 * y1;

                    // Use the discriminant to see if there are intersection points between the line and circle
                    double discriminant = pow(LOOKAHEAD_DISTANCE, 2) * pow(dr, 2) - pow(D, 2);

                    // = 0 is tangent, > 0 is intersection
                    if (discriminant >= 0)
                    {
                        // Get the sign of dy
                        int dySign;
                        if (dy >= 0)
                        {
                            dySign = 1;
                        }
                        else
                        {
                            dySign = -1;
                        }

                        // Find possible solutions to quadratic
                        double xSolution1 = (D * dy + dySign * dy * dx * sqrt(pow(LOOKAHEAD_DISTANCE, 2) * pow(dr, 2) - pow(D, 2))) / pow(dr, 2);
                        double xSolution2 = (D * dy - dySign * dy * dx * sqrt(pow(LOOKAHEAD_DISTANCE, 2) * pow(dr, 2) - pow(D, 2))) / pow(dr, 2);
                        double ySolution1 = (-D * dy + fabs(dy) * sqrt(pow(LOOKAHEAD_DISTANCE, 2) * pow(dr, 2) - pow(D, 2))) / pow(dr, 2);
                        double ySolution2 = (-D * dy - fabs(dy) * sqrt(pow(LOOKAHEAD_DISTANCE, 2) * pow(dr, 2) - pow(D, 2))) / pow(dr, 2);

                        // Offset solutions back to true position and store in a point
                        Point solution1 = {xSolution1 + robot::x, ySolution1 + robot::y};
                        Point solution2 = {xSolution2 + robot::x, ySolution2 + robot::y};

                        // Verify if solutions are in the path
                        double minX = std::min(x1, x2);
                        double maxX = std::max(x1, x2);
                        double minY = std::min(y1, y2);
                        double maxY = std::max(y1, y2);

                        // Is in range function
                        auto isInRange = [](double num, double min, double max)
                        {
                            return num >= min && num <= max;
                        };

                        // Booleans if solutions are valid
                        bool solution1Valid = isInRange(solution1.x, minX, maxX) && isInRange(solution1.y, minY, maxY);
                        bool solution2Valid = isInRange(solution2.x, minX, maxX) && isInRange(solution2.y, minY, maxY);

                        // If any solution is valid
                        if (solution1Valid || solution2Valid)
                        {
                            // Euclidean Distance calculator function
                            auto getDistance = [](Point pt1, Point pt2)
                            {
                                return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2));
                            };

                            // If both solutions are valid, check which one is better and set that to be the goal point
                            if (solution1Valid && solution2Valid)
                            {
                                // Whichever point is closer to next point in path is the goal point
                                if (getDistance(solution1, points[2]) < getDistance({robot::x, robot::y}, points[2]))
                                {
                                    // Goal is point 1
                                    goalPoint = solution1;
                                }
                                else
                                {
                                    // Goal is point 2
                                    goalPoint = solution2;
                                }
                            }
                            // If not, the one in range is the goal point
                            else
                            {
                                if (solution1Valid)
                                {
                                    // Goal is point 1
                                    goalPoint = solution1;
                                }
                                else
                                {
                                    // Goal is point 2
                                    goalPoint = solution2;
                                }
                            }

                            // If the goal point is closer to the next path point than the robot's distance from the next path point, then we're in business
                            if (getDistance(goalPoint, points[i+1]) < getDistance({robot::x, robot::y}, points[i+1]))
                            {
                                // Set the last found index
                                lastFoundIndex = i;

                                // Break out of the loop
                                break;
                            }
                            // Otherwise, the robot probably skipped points in its pathfollowing, keep searching
                            else
                            {
                                // Goal is the last goal point
                                goalPoint = points[lastFoundIndex];
                            }
                        }
                        // Otherwise
                        else
                        {
                            // Goal is the last goal point
                            goalPoint = points[i];
                        }
                    }
                }

                // We have the goal point, now move to it

                // PID to calculate angular velocity and linear velocity
                double angleToGoal = atan2(goalPoint.y - robot::y, goalPoint.x - robot::x);

                // Normalize the angle
                angleToGoal = atan2(sin(angleToGoal), cos(angleToGoal));

                // Proportional using turnKP
                double angleError = angleToGoal - robot::heading;

                // Normalize the angle error (negate because clockwise is positive)
                angleError = -atan2(sin(angleError), cos(angleError));

                // Proportional for linear speed
                double distanceToGoal = sqrt(pow(goalPoint.x - robot::x, 2) + pow(goalPoint.y - robot::y, 2));
                double linearError = distanceToGoal;

                // Integral for angular speed
                double angleIntegral = 0;
                // Integral for linear speed
                double linearIntegral = 0;

                // Windup prevention for angular speed
                if (!(leftSpeed >= 100 && (int) (-angleError / fabs(angleError)) == (int) (-leftSpeed / fabs(leftSpeed))))
                {
                    angleIntegral += (angleError * PID_TIMESTEP);
                }

                // Windup prevention for linear speed
                if (!(leftSpeed >= 100 && (int) (-linearError / fabs(linearError)) == (int) (-leftSpeed / fabs(leftSpeed))))
                {
                    linearIntegral += (linearError * PID_TIMESTEP);
                }

                // Derivative for angular speed
                double angleDerivative = (angleError - prevError) / PID_TIMESTEP;
                prevError = angleError;

                // Derivative for linear speed
                double linearPrevError = 0;
                double linearDerivative = (linearError - linearPrevError) / PID_TIMESTEP;
                linearPrevError = linearError;

                // Calculate the motor speeds
                double linearSpeed = linearError * kP + linearIntegral * kI + linearDerivative * kD;
                double angularSpeed = angleError * turnKP + angleIntegral * turnKI + angleDerivative * turnKD;

                // Apply the speeds to the motors with saturation protection
                double leftMotorSpeed = linearSpeed + angularSpeed;
                double rightMotorSpeed = linearSpeed - angularSpeed;

                // Saturation protection
                double maxSpeed = std::max(fabs(leftMotorSpeed), fabs(rightMotorSpeed));
                if (maxSpeed > 100) {
                    leftMotorSpeed = (leftMotorSpeed / maxSpeed) * 100;
                    rightMotorSpeed = (rightMotorSpeed / maxSpeed) * 100;
                }

                setLeftSpeed(leftMotorSpeed);
                setRightSpeed(rightMotorSpeed);

                // Break if we're at the end of the path
                if (lastFoundIndex == points.size() - 1)
                {
                    break;
                }

                // Wait to not consume all of the CPU's resources
                wait(10, msec);
            }

            // Stop the motors
            stopDrive();
        }

        static void setHeading(double deg)
        {
            inertial1.setHeading(deg, degrees);
            robot::heading = deg * (M_PI / 180);
        }
};