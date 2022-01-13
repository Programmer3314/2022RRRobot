// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utility.MMDifDriveTrain;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMSparkMaxMotorController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  MMDifDriveTrain driveTrain;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new MMDifDriveTrain(
      new MMFollowingMotorGroup(
        new MMSparkMaxMotorController(4, MotorType.kBrushless), 
        new MMSparkMaxMotorController(5, MotorType.kBrushless),
        new MMSparkMaxMotorController(6, MotorType.kBrushless)
      ),
      new MMFollowingMotorGroup(
        new MMSparkMaxMotorController(1, MotorType.kBrushless), 
        new MMSparkMaxMotorController(2, MotorType.kBrushless), 
        new MMSparkMaxMotorController(3, MotorType.kBrushless) 
      ),
    1
    );
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
