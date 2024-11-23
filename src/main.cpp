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
      Skills Auto
      Start in front of the alliacne stake, facing forward.
      Preload goes in front of the optical sensor.
   */

   // Determine the alliance color
   if (optical1.color() == red)
   {
      allianceIsRed = true;
   }
   else if (optical1.color() == blue)
   {
      allianceIsRed = false;
   }

   // Start the thread
   thread ejectThread = thread(eject);

   // Reset rotation
   inertial1.calibrate();
   while (inertial1.isCalibrating())
   {
      wait(200, msec);
   }
   inertial1.resetRotation();

}

// Driver Control
void usercontrol(void)
{
   // Start the thread
   thread ejectThread = thread(eject);

   while (1)
   {
      // Assign buttons for pneumatics here (use a callback function)
      controller1.ButtonX.pressed(clamp);
      controller1.ButtonUp.pressed(hang);
      controller1.ButtonLeft.pressed(toggleTipper);
      controller1.ButtonDown.pressed(changeAllianceColor);

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

         // Make the drive motors spin so the robot moves
         drive(forward);

         // Set the drive motors to coast
         leftF.setStopping(coast);
         leftB.setStopping(coast);
         rightF.setStopping(coast);
         rightB.setStopping(coast);

         // Set the non-drive motors to hold its position (add non-drive motors as necessary)
         bottomAccumulator.setStopping(hold);
         topAccumulator.setStopping(hold);

         // Set the velocity of the non-drive motors (add non-drive motors as necessary)
         bottomAccumulator.setVelocity(100, percent);
         topAccumulator.setVelocity(100, percent);

         /*
          Accumulator
          Buttons that control the accumulator
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

         // Wait to conserve brain resources
         wait(20, msec);
      }
   }
}

int main()
{
   // Set up callbacks for autonomous and driver control periods.
   Competition.autonomous(autonomous);
   Competition.drivercontrol(usercontrol);

   // Run the pre-autonomous function.
   pre_auton();

   // Prevent main from exiting with an infinite loop.
   while (true)
   {
      wait(100, msec);
   }
}
