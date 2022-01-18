// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSparkMaxMotorController;
import static frc.robot.Constants.*;

import javax.naming.spi.DirStateFactory.Result;

/*
Main TODO List:
- Do CLEANUPS...
- When possible start tuning the test chassis pids - we'll discuss this before you do it.

- Start on shooter code:
  This will likely be a question of adjusting the speed of the shooter and the release angle.
  We don't know the final configuration, but there will probably be some number of shooter motors, 
  and some number of hood adjust motors. Hopefully, Falcon500s. 
  I would like the shoot method to take a distance as the primary input parameter. How will the 
  adjustments be calculated from the desired shot distance? 

*/

//Control RPM of motors
//Control position of CAM
//Look at linear interpilation to determine firing solution
//dream plan- have firing solution always at the ready during a match

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
  MMJoystickAxis speed, turn;
  MMMotorGroup shooterWheels;
  MMMotorGroup shooterCAM;
  ShooterFormula shooterFormula;
  //PneumaticsControlModule PCM;
  Solenoid lightRing;
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    controllerDriver = new Joystick(4);
    speed = new MMJoystickAxis(4, 1, .2, kMaxSpeed);
    turn = new MMJoystickAxis(4, 4, .2, kMaxTurnRate);
    //PCM = new PneumaticsControlModule(1);
    lightRing = new Solenoid(1,PneumaticsModuleType.CTREPCM, 0);


    /**
    shooterCAM = new MMFollowingMotorGroup(
      new MMSparkMaxMotorController(11, MotorType.kBrushless)
      .setCurrentLimit(kNeoShooterCAMStallLimit, kNeoShooterCAMFreeLimit)
      .setInverted(false)
      .setPIDFParameters(kNeoShooterCAMP, kNeoShooterCAMI, kNeoShooterCAMD, kNeoShooterCAMF, kNeoShooterCAMIZ, kNeoShooterCAMMin, kNeoShooterCAMMax)
    );

    
    shooterWheels = new MMFollowingMotorGroup(
      new MMSparkMaxMotorController(10, MotorType.kBrushless)
      .setCurrentLimit(kNeoShooterWheelsStallLimit, kNeoShooterWheelsFreeLimit)
      .setInverted(false)
      .setPIDFParameters(kNeoShooterWheelsP, kNeoShooterWheelsI, kNeoShooterWheelsD, kNeoShooterWheelsF, kNeoShooterWheelsIZ, kNeoShooterWheelsMin,
       kNeoShooterWheelsMax)
    );
     */
    
    shooterFormula = new ShooterFormula();

    SmartDashboard.putNumber("Target Distance", 0);

    driveTrain = new MMDiffDriveTrain(
        new MMFollowingMotorGroup(
            new MMSparkMaxMotorController(4, MotorType.kBrushless)
                .setCurrentLimit(kNeoDriveTrainStallLimit, kNeoDriveTrainFreeLimit)
                .setInverted(true)
                .setPIDFParameters(kNeoDriveTrainP, kNeoDriveTrainI, kNeoDriveTrainD, kNeoDriveTrainF, kNeoDriveTrainIZ,
                    kNeoDriveTrainMin, kNeoDriveTrainMax),
            new MMSparkMaxMotorController(5, MotorType.kBrushless),
            new MMSparkMaxMotorController(6, MotorType.kBrushless)),
        new MMFollowingMotorGroup(
            new MMSparkMaxMotorController(1, MotorType.kBrushless)
                .setCurrentLimit(kNeoDriveTrainStallLimit, kNeoDriveTrainFreeLimit)
                .setInverted(false)
                .setPIDFParameters(kNeoDriveTrainP, kNeoDriveTrainI, kNeoDriveTrainD, kNeoDriveTrainF, kNeoDriveTrainIZ,
                    kNeoDriveTrainMin, kNeoDriveTrainMax),
            new MMSparkMaxMotorController(2, MotorType.kBrushless),
            new MMSparkMaxMotorController(3, MotorType.kBrushless)),
        4.67, 1.04);
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

    // double vertical = controllerDriver.getRawAxis(1);
    // double horizonal = controllerDriver.getRawAxis(4);
    driveTrain.Drive(speed.get(), turn.get());
    double shooterRPM=0; 
    double shooterAngle=0;
    if (controllerDriver.getRawButtonPressed(9)){
      lightRing.set(!lightRing.get());
    }

    //input distance via smartdashboard and then 
    double test = SmartDashboard.getNumber("Target Distance", 0);
    TargetPoint firingSolution = shooterFormula.calculate(test);
    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);      
    }else{
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);

    }
    

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
