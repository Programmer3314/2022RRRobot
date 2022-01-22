// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSRXMotorController;
import frc.robot.utility.MMSparkMaxMotorController;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;


/*
Main TODO List:
- Do CLEANUPS...
- Create code to test intake... (can bus 20 - confirm)
  Use trigger to control the intake power. 
  Output intake power to SmartDashboard.
  Later when power is established covert to a button that will intake a ball. 
  
- Get Angle Up/Down from vision system. 
  Correct angle to account for camera physical angle.
  Use angle to calculate horizontal distance to target.
  Pass result to firing solution calculation.
- Get Angle Left/Right from vision system.
  Create class to work as a PID controller - Only implement P and D components.
  Use PID controller to steer robot to target (angle only) when button pressed. 
  
- Drive robot to specific distance while pointing to target
  and indicate when good.   

- When possible start tuning the test chassis pids - we'll discuss this before you do it.

- On Hold for now... Start on shooter code: 
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
  Solenoid lightRing;
  MMJoystickAxis intakeTrigger;
  MMFollowingMotorGroup frontIntake; 
  NetworkTableInstance nt;
  NetworkTable visiontable;
  AHRS navx;
  double autocorrectTargetAngle; 
   Boolean lowConfidence; 
    int confidenceCounter;  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    nt = NetworkTableInstance.getDefault();
    visiontable = nt.getTable("Retroreflective Tape Target");
    lowConfidence=true; 
    confidenceCounter=0;

    controllerDriver = new Joystick(4);
    speed = new MMJoystickAxis(4, 1, .2, kMaxSpeed);
    turn = new MMJoystickAxis(4, 4, .2, kMaxTurnRate);
    lightRing = new Solenoid(1,PneumaticsModuleType.CTREPCM, 0);
    intakeTrigger = new MMJoystickAxis(4, 2, .5, 1);
    frontIntake = new MMFollowingMotorGroup( 
      new MMSRXMotorController(20)
  
    );

    navx = new AHRS(SPI.Port.kMXP);
     navx.reset();
  

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
    double currentAngle=navx.getYaw();
    SmartDashboard.putNumber("curentAngle", currentAngle);

    // double vertical = controllerDriver.getRawAxis(1);
    // double horizonal = controllerDriver.getRawAxis(4);
   
    double requestedSpeed=speed.get();
    double requestedTurn=turn.get();
   

    double IntakePower= intakeTrigger.get();
    frontIntake.setPower(IntakePower);
    SmartDashboard.putNumber("Intake Power", IntakePower);

   double verticalAngle= (Double)visiontable.getEntry("Vertical Angle").getNumber(-5000)+ kCameraVerticalAngle;
   double horizontalAngle = (Double)visiontable.getEntry("Horizontal Angle").getNumber(-5000);
   SmartDashboard.putNumber("Vertical Angle", verticalAngle);
   SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);
 

   //TODO handle lack of confidence for a while

if (visiontable.getEntry("Confidence").getBoolean(false)){
   autocorrectTargetAngle= currentAngle+horizontalAngle;
   lowConfidence=false;
   confidenceCounter=150;
}
else{
  if(confidenceCounter>0){
    confidenceCounter--;
  }
  if(confidenceCounter<=0){
    lowConfidence=true;
  }
}



    double shooterRPM=0; 
    double shooterAngle=0;
    if (controllerDriver.getRawButtonPressed(9)){
      lightRing.set(!lightRing.get());
    }
    double targetDistance= kTargetingHeightDiff/ Math.tan( Math.toRadians(verticalAngle));
  SmartDashboard.putNumber("Target Distance", targetDistance);

    //input distance via smartdashboard and then 
    //double test = SmartDashboard.getNumber("Target Distance", 0);
    TargetPoint firingSolution = shooterFormula.calculate(targetDistance);
    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);      
    }else{
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);

    }


    if(controllerDriver.getRawButton(1) && !lowConfidence) {
      double p = 5;
      
      double xAngle=0;
      double currentError = autocorrectTargetAngle-currentAngle;
      //double currentError=xAngle- currentAngle;
      requestedTurn=p*currentError;
      SmartDashboard.putNumber("Auto Angle Correct", requestedTurn); 

    }
    SmartDashboard.putBoolean("lowConfidence",lowConfidence);
       

    SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());
    driveTrain.Drive(requestedSpeed, requestedTurn);

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
