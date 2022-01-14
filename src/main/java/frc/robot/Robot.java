// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMSparkMaxMotorController;

/*
Main TODO List:
- Do CLEANUPS...
- Convert turn to be degrees per second - this will require additional drivetrain geometry info
- Create class MMJoystickAxis to handle data cleanups like Deadband and Scaling, so that 
  a configured MMJoystickAxis will return a clean value. (For example return speed in Feet/Sec.)
- When possible start tuning the test chassis pids - we'll discuss this before you do it.

- Start on shooter code:
  This will likely be a question of adjusting the speed of the shooter and the release angle.
  We don't know the final configuration, but there will probably be some number of shooter motors, 
  and some number of hood adjust motors. Hopefully, Falcon500s. 
  I would like the shoot method to take a distance as the primary input parameter. How will the 
  adjustments be calculated from the desired shot distance? 

*/

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  MMDiffDriveTrain driveTrain;
  Joystick controllerDriver;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    controllerDriver = new Joystick(4);

    // TODO CLEANUP setup constants for the constants below
    // TODO CLEANUP Create a class MMSparkMaxPIDFParms. The class ...
    // can be used in constants. And only one value needs to be
    // passed into setPIDFParameters.
    driveTrain = new MMDiffDriveTrain(
        new MMFollowingMotorGroup(
            new MMSparkMaxMotorController(4, MotorType.kBrushless)
                .setCurrentLimit(40, 40)
                .setInverted(true)
                .setPIDFParameters(0, 0, 0, 0.0002, 0, -1, 1),
            new MMSparkMaxMotorController(5, MotorType.kBrushless),
            new MMSparkMaxMotorController(6, MotorType.kBrushless)),
        new MMFollowingMotorGroup(
            new MMSparkMaxMotorController(1, MotorType.kBrushless)
                .setCurrentLimit(40, 40)
                .setInverted(false)
                .setPIDFParameters(0, 0, 0, 0.0002, 0, -1, 1),
            new MMSparkMaxMotorController(2, MotorType.kBrushless),
            new MMSparkMaxMotorController(3, MotorType.kBrushless)),
        4.67);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    double vertical = controllerDriver.getRawAxis(1);
    double horizonal = controllerDriver.getRawAxis(4);
    // TODO CLEANUP use constants for the #s below
    driveTrain.Drive(-10 * vertical, 10 * horizonal);
    SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
