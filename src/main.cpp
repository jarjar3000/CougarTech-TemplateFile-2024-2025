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
   inertial1.calibrate();
   while (inertial1.isCalibrating())
   {
      wait(200, msec);
   }

   inertial1.setRotation(0, degrees);
   // Red 270, blue 90

   /*
       Start backwards, right in front of the mobile goal
       Preload goes in the accumulator, near the top.
       There should be a ring opposite the alliance color on the right.
   */
   
   // Drive backwards into the mobile goal
   drive(reverse, 19, 10); //20
   clamp();

   // Turn (left or right, depends on starting side), but the heading will be the same
   if (red)
   {
      turn(right, 90, 10); // 180
   }
   else
   {
      turn(left, 90, 10); // 180
   }  

   // Accumulate alliace ring into goal
   spinAccumulator(forward, 100);
   drive(forward, 19, 10); //20, 17
   wait(0.25, seconds);
   stopAccumulator();
   spinAccumulator(forward, 100);

   // Reverse back to original position
   drive(reverse, 16, 10);

   // Turn towards the corner and ram the ring to get it
   if (red)
   {
      turn(left, 45, 10);
   }
   else
   {
      turn(right, 45, 10);
   }

   // Forward
   stopAccumulator();
   spinAccumulator(reverse, 100);
   drive(forward, 55, 10); //50
   // wait(1, seconds);
   stopAccumulator();
   spinAccumulator(forward, 100);
   wait(1, seconds);
   drive(reverse, 10, 10);

}

int driver()
{
   // Set up all pneumatic callbacks
   controller1.ButtonX.pressed(clamp);
   controller1.ButtonUp.pressed(hang);
   controller1.ButtonLeft.pressed(toggleTipper);
   controller1.ButtonR1.pressed(driveAccumulatorForward);
   controller1.ButtonR2.pressed(driveAccumulatorReverse);

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
   thread ejectThread = thread(eject);
   optical1.setLight(ledState::on);

   // Prevent main from exiting with an infinite loop.
   while (true)
   {
      wait(100, msec);
   }
}
