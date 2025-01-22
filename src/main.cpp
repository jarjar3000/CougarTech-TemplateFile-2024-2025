/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jared                                                     */
/*    Created:      8/23/2024, 03:51:27 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*
   This file will configure a robot with a four motor drive, two motor arm, and 1 motor claw.
   Given everything is plugged into the right ports, it will function.
   If you are branching from this file, delete what you don't need.
*/

#include "vex.h"
#include "robot-config.h"
#include "1-functions.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Pre-auton
void pre_auton(void)
{
}

// Autonomous
void autonomous(void)
{
   /*
      Since the field isn't mirrored, two "versions" of
      the program are needed.
      Change the boolean depending if we are on the red side or not.
   */
   bool red = allianceIsRed;
   // inertial1.calibrate();
   // while (inertial1.isCalibrating())
   // {
   //    wait(200, msec);
   // }

   inertial1.setRotation(0, degrees);
   inertial1.resetRotation();
   skills.clear();

   
   drive(forward);
   setLeftSpeed(60);
   setRightSpeed(100);
   wait(2, seconds);
   stopDrive();

/*
   // Put ring on alliance stake
   spinAccumulator(forward, 90);
   wait(1, seconds);
   stopAccumulator();

   // Go forward, turn left, and clamp mobile goal
   drive(forward, 10, 10); // 9
   turn(left, 90, 10);
   drive(reverse, 18, 10); // into mogo
   clamp();

   // Turn right to face the red ring and grab it
   turn(right, 90, 5);
   spinAccumulator(forward, 90);
   drive(forward, 27, 10); //25

   // Turn towards the next ring and grab it
   turn(right, 80, 10);
   drive(forward, 25, 10);

   // Turn towards the 2 rings alligned straight and grab them
   turn(right, 90, 10);
   drive(forward, 35, 10);

   // Turn to grab other ring
   turn(left, 120, 10);
   drive(forward, 12, 10);

   // Put goal in corner
   turn(left, 105, 10);
   drive(reverse, 12, 10);
   stopAccumulator();
   clamp();

   // Release the goal, drive to the other side and get the goal
   drive(forward, 10, 10);
   turn(right, 128, 10); //130
   drive(reverse, 70, 10);
   clamp();

   // BEGINNING BUT OPPOSITE
   // Turn right to face the red ring and grab it
   turn(left, 90, 5);
   spinAccumulator(forward, 90);
   drive(forward, 25, 10); //25

   // Turn towards the next ring and grab it
   turn(left, 85, 10);
   drive(forward, 25, 10);

   // Turn towards the 2 rings alligned straight and grab them
   turn(left, 90, 10);
   drive(forward, 35, 10);

   // Turn to grab other ring
   turn(right, 120, 10);
   drive(forward, 12, 10);

   // Put goal in corner
   turn(right, 95, 10); //105
   drive(reverse, 15, 10);
   stopAccumulator();
   clamp();

   // LITERALLY TWO SECONDS BRUH HANG!
   hang();
   setLeftSpeed(100);
   setRightSpeed(100);
   drive(forward);
   waitUntil(skills.time(seconds) >= 59);
   hang();
*/
}

int driver()
{
   // Set up all pneumatic callbacks
   controller1.ButtonX.pressed(clamp);
   controller1.ButtonUp.pressed(hang);
   controller1.ButtonLeft.pressed(toggleTipper);

   // Set the drive motors to coast
   leftF.setStopping(coast);
   leftB.setStopping(coast);
   leftE.setStopping(coast);
   rightF.setStopping(coast);
   rightB.setStopping(coast);
   rightE.setStopping(coast);

   // Set the non-drive motors to hold its position (add non-drive motors as necessary)
   bottomAccumulator.setStopping(hold);
   topAccumulator.setStopping(hold);

   // Set the velocity of the non-drive motors (add non-drive motors as necessary)
   bottomAccumulator.setVelocity(100, percent);
   topAccumulator.setVelocity(100, percent);
   
   while (true)
   {
      // Joysticks, arms, claws, etc. go here

      // Controller Deadzone, a smaller number makes the joysticks more sensitive
      float deadband = 5;

      // Variables track both joystick positions and will update motor velocities
      float leftMotorSpeed = controller1.Axis3.position();
      float rightMotorSpeed = controller1.Axis2.position();

      /*
         Left Side of Drivetrain
         The joystick positions will set the speed of the left motors.
         If that number is less than the deadband, the motors will not move.
      */
      if (fabs(leftMotorSpeed) < deadband)
      {
         // Set the speed to 0
         setLeftSpeed(0);
      }
      else
      {
         if (leftMotorSpeed >= MAX_DRIVE_SPEED)
         {
            setLeftSpeed(MAX_DRIVE_SPEED);
         }
         else if (leftMotorSpeed <= -MAX_DRIVE_SPEED)
         {
            setLeftSpeed(-MAX_DRIVE_SPEED);
         }
         else
         {
            setLeftSpeed(leftMotorSpeed);
         }
      }

      /*
         Right Side of Drivetrain
         The joystick positions will set the speed of the right motors.
         If that number is less than the deadband, the motors will not move.
         */
      if (fabs(rightMotorSpeed) < deadband)
      {
         // Set the speed to 0
         setRightSpeed(0);
      }
      else
      {
         if (rightMotorSpeed >= MAX_DRIVE_SPEED)
         {
            setRightSpeed(MAX_DRIVE_SPEED);
         }
         else if (rightMotorSpeed <= -MAX_DRIVE_SPEED)
         {
            setRightSpeed(-MAX_DRIVE_SPEED);
         }
         else
         {
            setRightSpeed(rightMotorSpeed);
         }
      }

      /*
         Accumulator
      */
      if (controller1.ButtonR1.pressing())
      {
         bottomAccumulator.spin(forward);
         topAccumulator.spin(forward);
      }
      else if (controller1.ButtonR2.pressing())
      {
         bottomAccumulator.spin(reverse);
         topAccumulator.spin(reverse);
      }
      else
      {
         bottomAccumulator.stop();
         topAccumulator.stop();
      }

      // Make the drive motors spin so the robot moves
      drive(forward);

      // Wait to conserve brain resources
      this_thread::sleep_for(20);
   }
   return 0;
}
// Driver Control
void usercontrol(void)
{
   thread driverThread = thread(driver);
}

int main()
{
   // Set up callbacks for autonomous and driver control periods.
   Competition.autonomous(autonomous);
   Competition.drivercontrol(usercontrol);

   // Run the pre-autonomous function.
   pre_auton();

   // Start the thread
   // thread ejectThread = thread(eject);
   // optical1.setLight(ledState::on);

   // Prevent main from exiting with an infinite loop.
   while (true)
   {
      wait(100, msec);
   }
}
