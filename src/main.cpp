/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jared                                                     */
/*    Created:      8/23/2024, 03:51:27 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot.h"

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
   if (robot::allianceIsRed)
   {
      // Red autonomous
      robot::setHeading(270);
   }
   else
   {
      // Blue autonomous goes here
      robot::setHeading(90);
   }

   // Drive to the mobile goal
   robot::setPrecice();
   robot::driveStraight(reverse, 32); //28
   robot::setFast();
   robot::clamp();
   robot::spinAccumulator(forward, 100);

   // Turn towards heading 0 and drive into the ring
   robot::turnToHeading(0);
   robot::driveStraight(forward, 18); // 20

   // Turn towards the stacked rings
   if (robot::allianceIsRed)
   {
      robot::turnToHeading(90);
   }
   else
   {
      robot::turnToHeading(270);
   }

   // Drive to the stacked rings
   robot::driveStraight(forward, 10); // 8

   // Turn 45 degrees and get the other ring
   double degTurn = 45;
   if (robot::allianceIsRed)
   {
      robot::turnToHeading(90 - degTurn);
   }
   else
   {
      robot::turnToHeading(270 + degTurn);
   }

   robot::driveStraight(forward, 5); // 4

   // Drive backwards and go towards the corner stack
   robot::driveStraight(reverse, 24); // 20
   if (robot::allianceIsRed)
   {
      robot::turnToHeading(310);
   }
   else
   {
      robot::turnToHeading(140);
   }

   // Accumulate corner stack
   robot::drive(forward);
   robot::setLeftSpeed(100);
   robot::setRightSpeed(100);
   wait(1.5, seconds);
   robot::stopDrive();

   // Get other corner rings
   repeat(3)
   {
      robot::driveStraight(reverse, 5);
      robot::setLeftSpeed(100);
      robot::setRightSpeed(100);
      robot::drive(forward);
      wait(0.5, seconds);
      robot::stopDrive();
   }
   robot::stopDrive();

   // Drive back and pray
   robot::driveStraight(reverse, 24);
}

int driver()
{
   // Set up all pneumatic callbacks
   controller1.ButtonX.pressed(robot::clamp);
   controller1.ButtonUp.pressed(robot::hang);
   controller1.ButtonY.pressed(robot::armStopper);
   controller1.ButtonDown.pressed(robot::toggleAllianceColor);
   controller1.ButtonB.pressed(robot::intakeLift);

   // Set the drive motors to coast
   leftF.setStopping(coast);
   leftB.setStopping(coast);
   rightF.setStopping(coast);
   rightB.setStopping(coast);

   // Set the non-drive motors to hold its position (add non-drive motors as necessary)
   bottomAccumulatorL.setStopping(hold);
   bottomAccumulatorR.setStopping(hold);
   topAccumulator.setStopping(hold);
   rightArm.setStopping(hold);

   // Set the velocity of the non-drive motors (add non-drive motors as necessary)
   bottomAccumulatorL.setVelocity(100, percent);
   bottomAccumulatorR.setVelocity(100, percent);
   topAccumulator.setVelocity(robot::MAX_TOP_ACCUMULATOR_SPEED, percent);
   rightArm.setVelocity(100, percent);
   
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
         robot::setLeftSpeed(0);
      }
      else
      {
         if (leftMotorSpeed >= robot::MAX_DRIVE_SPEED)
         {
            robot::setLeftSpeed(robot::MAX_DRIVE_SPEED);
         }
         else if (leftMotorSpeed <= -(robot::MAX_DRIVE_SPEED))
         {
            robot::setLeftSpeed(-(robot::MAX_DRIVE_SPEED));
         }
         else
         {
            robot::setLeftSpeed(leftMotorSpeed);
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
         robot::setRightSpeed(0);
      }
      else
      {
         if (rightMotorSpeed >= robot::MAX_DRIVE_SPEED)
         {
            robot::setRightSpeed(robot::MAX_DRIVE_SPEED);
         }
         else if (rightMotorSpeed <= -(robot::MAX_DRIVE_SPEED))
         {
            robot::setRightSpeed(-(robot::MAX_DRIVE_SPEED));
         }
         else
         {
            robot::setRightSpeed(rightMotorSpeed);
         }
      }

      /*
         Accumulator
      */
      if (controller1.ButtonR1.pressing())
      {
         
         bottomAccumulatorL.spin(forward);
         bottomAccumulatorR.spin(forward);
         topAccumulator.spin(forward);
      }
      else if (controller1.ButtonR2.pressing())
      {
         bottomAccumulatorL.spin(reverse);
         bottomAccumulatorR.spin(reverse);
         topAccumulator.spin(reverse);
      }
      else
      {
         bottomAccumulatorL.stop();
         bottomAccumulatorR.stop();
         topAccumulator.stop();
      }

      /*
         Arm
      */
      if (controller1.ButtonL1.pressing())
      {
         rightArm.spin(forward);
      }
      else if (controller1.ButtonL2.pressing())
      {
         rightArm.spin(reverse);
      }
      else
      {
         rightArm.stop();
      }

      /**
       * Odometry button, if calibrate is true
       */
      if (robot::CALIBRATE)
      {
         if (controller1.ButtonRight.pressing())
         {
            robot::setLeftSpeed(robot::MAX_DRIVE_SPEED);
            robot::setRightSpeed(-robot::MAX_DRIVE_SPEED);
         }

         if (controller1.ButtonLeft.pressing())
         {
            robot::setLeftSpeed(-robot::MAX_DRIVE_SPEED);
            robot::setRightSpeed(robot::MAX_DRIVE_SPEED);
         }
      }

      // Make the drive motors spin so the robot moves
      robot::drive(forward);

      // Wait to conserve brain resources
      wait(20, msec);
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
   vexcodeInit();

   // Set up callbacks for autonomous and driver control periods.
   Competition.autonomous(autonomous);
   Competition.drivercontrol(usercontrol);

   // Run the pre-autonomous function.
   pre_auton();

   // Set the alliance color
   wait(200, msec);
   if (optical1.hue() <= robot::OPTICAL_RED_HUE)
   {
      robot::allianceIsRed = true;
      robot::init(0, 0, 270);
   }
   else
   {
      robot::allianceIsRed = false;
      robot::init(0, 0, 90);
   }
   

   // Start the thread
   thread ejectThread = thread(robot::eject);
   optical1.setLight(ledState::on);

   // Start the odometry tracking thread
   thread odometryTracking = thread(robot::calculateRobotPosition);

   // Start the printing thread
   thread controllerInfo = thread(robot::printInfoToController);

   // Start the arm thread
   thread arm = thread(robot::armRingStop);

   // Prevent main from exiting with an infinite loop.
   while (true)
   {
      wait(100, msec);
   }
}
