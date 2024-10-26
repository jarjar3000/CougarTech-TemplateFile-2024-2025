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
   while (inertial1.isCalibrating())
   {
      inertial1.calibrate();
   }

   // This is supposed to be 180 
   inertial1.setHeading(90, degrees);
   heading = 90;

   /*
      Because the field isn't truly mirrored, the red
      and blue versions of this program are different.
      (We have to turn in different directions)
   */
   bool red = true;

   /*
      Start forwards, right in front of the stack of rings on the left
      Ring shouldn't be in the way of the robot
      Preload goes in the clamp
   */

  // Drive into the stack of rings and accumulate
  spinAccumulator(forward, 100); 
  drive(forward, 24, 10); //24
  stopAccumulator();

  // Turn (left or right, depends on starting side)
  turn(left, 0, 10); //180

  // back one tile and accumulate
  drive(reverse, 24, 10);
  clamp();
  spinAccumulator(forward, 100);

  // Turn left 90 degrees
  turn(left, 270, 10); //90
  stopAccumulator();

  // Forward 1.25 tiles
  drive(forward, 28, 10);

  // Left 90 degrees
  turn(left, 180, 10);

  // Forward full tile
  drive(forward, 24, 10);

  // Right 90
  turn(right, 270, 10);
  
  // Arm up
  liftArm(up, 500, 100);

  // Move forward
  drive(forward, 12, 10);

  // Arm down
  liftArm(down, 400, 10);

  // Back up
  drive(reverse, 12, 10);


}

// Driver Control
void usercontrol(void)
{
   while (1)
   {
      // Assign buttons for pneumatics here (use a callback function)
      controller1.ButtonX.pressed(clamp);

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
            leftF.setVelocity(0, percent);
            leftB.setVelocity(0, percent);
         }
         else
         {
            if (leftMotorSpeed >= MAX_DRIVE_SPEED)
            {
               leftF.setVelocity(MAX_DRIVE_SPEED, percent);
               leftB.setVelocity(MAX_DRIVE_SPEED, percent);
            }
            else if (leftMotorSpeed <= -MAX_DRIVE_SPEED)
            {
               leftF.setVelocity(-MAX_DRIVE_SPEED, percent);
               leftB.setVelocity(-MAX_DRIVE_SPEED, percent);
            }
            else
            {
               leftF.setVelocity(leftMotorSpeed, percent);
               leftB.setVelocity(leftMotorSpeed, percent);
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
            rightF.setVelocity(0, percent);
            rightB.setVelocity(0, percent);
         }
         else
         {
            if (rightMotorSpeed >= MAX_DRIVE_SPEED)
            {
               rightF.setVelocity(MAX_DRIVE_SPEED, percent);
               rightB.setVelocity(MAX_DRIVE_SPEED, percent);
            }
            else if (rightMotorSpeed <= -MAX_DRIVE_SPEED)
            {
               rightF.setVelocity(-MAX_DRIVE_SPEED, percent);
               rightB.setVelocity(-MAX_DRIVE_SPEED, percent);
            }
            else
            {
               rightF.setVelocity(rightMotorSpeed, percent);
               rightB.setVelocity(rightMotorSpeed, percent);
            }
         }

         // Make the drive motors spin so the robot moves
         leftF.spin(forward);
         leftB.spin(forward);
         rightF.spin(forward);
         rightB.spin(forward);

         // Set the drive motors to coast
         leftF.setStopping(coast);
         leftB.setStopping(coast);
         rightF.setStopping(coast);
         rightB.setStopping(coast);

         // Set the non-drive motors to hold its position (add non-drive motors as necessary)
         bottomAccumulator.setStopping(hold);
         topAccumulator.setStopping(hold);
         leftLift.setStopping(hold);
         rightLift.setStopping(hold);

         // Set the velocity of the non-drive motors (add non-drive motors as necessary)
         bottomAccumulator.setVelocity(100, percent);
         topAccumulator.setVelocity(100, percent);
         leftLift.setVelocity(50, percent);
         rightLift.setVelocity(50, percent);

         /*
          Arm
          The code below assigns lifting the arm to the left bumpers on the controller
          This code can be applied to anything mechanism that will function using bumper presses
         */
         if (controller1.ButtonL1.pressing())
         {
            leftLift.spin(forward);
            rightLift.spin(forward);
         }
         else if (controller1.ButtonL2.pressing())
         {
            leftLift.spin(reverse);
            rightLift.spin(reverse);
         }
         else
         {
            leftLift.stop();
            rightLift.stop();
         }

         /*
          Accumulator
          Buttons that control the accumulator
         */
         if (controller1.ButtonR1.pressing())
         {
            if (limit1.pressing())
            {
               bottomAccumulator.spin(forward, 100, percent);
               topAccumulator.spin(forward, 20, percent);
            }
            else
            {
               bottomAccumulator.spin(forward, 100, percent);
               topAccumulator.spin(forward, 100, percent);
            }
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
