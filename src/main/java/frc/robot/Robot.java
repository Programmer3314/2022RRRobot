// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.kCameraVerticalAngle;
import static frc.robot.Constants.kChassiRadius;
import static frc.robot.Constants.kMaxSpeed;
import static frc.robot.Constants.kMaxTurnRate;
import static frc.robot.Constants.kNeoDriveTrainD;
import static frc.robot.Constants.kNeoDriveTrainF;
import static frc.robot.Constants.kNeoDriveTrainFreeLimit;
import static frc.robot.Constants.kNeoDriveTrainI;
import static frc.robot.Constants.kNeoDriveTrainIZ;
import static frc.robot.Constants.kNeoDriveTrainMax;
import static frc.robot.Constants.kNeoDriveTrainMin;
import static frc.robot.Constants.kNeoDriveTrainP;
import static frc.robot.Constants.kNeoDriveTrainStallLimit;
import static frc.robot.Constants.kRevPerFoot;
import static frc.robot.Constants.kTargetingHeightDiff;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSRXMotorController;
import frc.robot.utility.MMSparkMaxMotorController;

/*
Main TODO List:
- Do CLEANUPS...

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
  public static MMDiffDriveTrain driveTrain;
  Joystick controllerDriver;
  MMJoystickAxis speed, turn;
  MMMotorGroup shooterWheels;
  MMMotorGroup shooterCAM;
  ShooterFormula shooterFormula;
  public static Solenoid lightRing;
  MMJoystickAxis intakeTrigger;
  MMFollowingMotorGroup frontIntake;
  public static NetworkTableInstance nt;
  public static NetworkTable visiontable;
  public static AHRS navx;
  public static double autocorrectTargetAngle;
  public static int confidenceCounter;
  public static double currentAngle;
  public static double verticalAngle;
  public static double horizontalAngle;
  public static MMAutonomous autonomous;
  public static boolean autoBallPickup;
  public static Alliance alliance;
  public static double ballChaseAngle;
  public static MMMotorGroup queueBelt;
  public static MMMotorGroup tunnelBelt;
  public static QueueStateMachine queueStateMachine;
  public static TunnelStateMachine tunnelStateMachine;
  public static ShooterStateMachine shooterStateMachine;
  public static MMMotorGroup tunnelWheels;
  public static ColorSensorV3 frontColorSensor;





  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // TODO CLEANUP Organize the init code to group simillar code
    // like Motor devices together, Human inputs together,
    // Sensors together, Data init, etc.
    nt = NetworkTableInstance.getDefault();
    visiontable = nt.getTable("Retroreflective Tape Target");
    lightRing = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
    navx = new AHRS(SPI.Port.kOnboardCS0);
    navx.reset();

    confidenceCounter = 0;

    frontColorSensor = new ColorSensorV3(Port.kMXP);
    controllerDriver = new Joystick(4);
    speed = new MMJoystickAxis(4, 1, .2, kMaxSpeed);
    turn = new MMJoystickAxis(4, 4, .2, kMaxTurnRate);
    intakeTrigger = new MMJoystickAxis(4, 2, .5, 1);
    frontIntake = new MMFollowingMotorGroup(
        new MMSRXMotorController(20));

    /**
     * shooterCAM = new MMFollowingMotorGroup(
     * new MMSparkMaxMotorController(11, MotorType.kBrushless)
     * .setCurrentLimit(kNeoShooterCAMStallLimit, kNeoShooterCAMFreeLimit)
     * .setInverted(false)
     * .setPIDFParameters(kNeoShooterCAMP, kNeoShooterCAMI, kNeoShooterCAMD,
     * kNeoShooterCAMF, kNeoShooterCAMIZ, kNeoShooterCAMMin, kNeoShooterCAMMax)
     * );
     * 
     * 
     * shooterWheels = new MMFollowingMotorGroup(
     * new MMSparkMaxMotorController(10, MotorType.kBrushless)
     * .setCurrentLimit(kNeoShooterWheelsStallLimit, kNeoShooterWheelsFreeLimit)
     * .setInverted(false)
     * .setPIDFParameters(kNeoShooterWheelsP, kNeoShooterWheelsI,
     * kNeoShooterWheelsD, kNeoShooterWheelsF, kNeoShooterWheelsIZ,
     * kNeoShooterWheelsMin,
     * kNeoShooterWheelsMax)
     * );
     */

    shooterFormula = new ShooterFormula();
    SmartDashboard.putNumber("Target Distance", 0);

    // TODO Move definitions of MotorGroups controlled by 
    // these state machines and pass them in just like 
    // with the diffDriveTrain below. This will prevent 
    // confusion and/or conflict over controlling them. 
    // The same should be done with sensor definitions. 
    queueStateMachine = new QueueStateMachine();
    tunnelStateMachine = new TunnelStateMachine();
    shooterStateMachine = new ShooterStateMachine();

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
        kRevPerFoot, kChassiRadius);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    alliance = DriverStation.getAlliance();
    autonomous = new TwoBallAuto();
  }

  @Override
  public void autonomousPeriodic() {
    commonPeriodic();
    currentAngle = navx.getYaw();

    verticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000) + kCameraVerticalAngle;
    horizontalAngle = (Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000);
    SmartDashboard.putNumber("Vertical Angle", verticalAngle);
    SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);

    

    SmartDashboard.putNumber("RobotDistance", driveTrain.getDistanceFeet());

    autonomous.periodic();
    SmartDashboard.putString("CurrentState", autonomous.currentState.toString());
  }

  @Override
  public void teleopInit() {
    alliance = DriverStation.getAlliance();
  }

  @Override
  public void teleopPeriodic() {
    commonPeriodic();
    currentAngle = navx.getYaw();
    SmartDashboard.putNumber("curentAngle", currentAngle);

    // double vertical = controllerDriver.getRawAxis(1);
    // double horizonal = controllerDriver.getRawAxis(4);

    double requestedSpeed = speed.get();
    double requestedTurn = turn.get();

    
    autoBallPickup = controllerDriver.getRawButton(2);

    double IntakePower = intakeTrigger.get();
    frontIntake.setPower(IntakePower);
    SmartDashboard.putNumber("Intake Power", IntakePower);

    verticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000) + kCameraVerticalAngle;
    horizontalAngle = (Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000);
    SmartDashboard.putNumber("Vertical Angle", verticalAngle);
    SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);

    double shooterRPM = 0;
    double shooterAngle = 0;
    if (controllerDriver.getRawButtonPressed(9)) {
      lightRing.set(!lightRing.get());
    }
    double targetDistance = kTargetingHeightDiff / Math.tan(Math.toRadians(verticalAngle));
    SmartDashboard.putNumber("Target Distance", targetDistance);

    // input distance via smartdashboard and then
    // double test = SmartDashboard.getNumber("Target Distance", 0);
    TargetPoint firingSolution = shooterFormula.calculate(targetDistance);
    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);
    } else {
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);
    }

    if (controllerDriver.getRawButton(1) && confidenceCounter > 0) {
      double p = 5;

      double currentError = autocorrectTargetAngle - currentAngle;
      // double currentError=xAngle- currentAngle;
      requestedTurn = p * currentError;
      SmartDashboard.putNumber("Auto Angle Correct", requestedTurn);

    }

    boolean targetConfidence = nt.getTable("Ball Target").getEntry(alliance==Alliance.Blue?"Blue Target Confidence": "Red Target Confidence").getBoolean(false);
    ballChaseAngle = (Double) nt.getTable("Ball Target").getEntry(alliance==Alliance.Blue?"Blue Angle to Ball": "Red Angle to Ball").getNumber(0);
    SmartDashboard.putNumber("TargetBallAngle", ballChaseAngle);

    if(autoBallPickup /*&& targetConfidence*/){
      double p = 3;
      requestedTurn = p*ballChaseAngle;
      requestedSpeed = -1;
    }
    SmartDashboard.putNumber("ConfidenceCounter", confidenceCounter);

    SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());
    driveTrain.Drive(requestedSpeed, requestedTurn);

    SmartDashboard.putString("Alliance Type", alliance.toString());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    alliance = DriverStation.getAlliance();
    // TODO Move all occurances of new ColorSensorV3 into Robot Init. (And then into the Tunnel constructor)
    frontColorSensor = new ColorSensorV3(Port.kMXP);
  }

  @Override
  public void testPeriodic() {
    commonPeriodic();
    tunnelStateMachine.update();
    SmartDashboard.putBoolean("Desired Ball", tunnelStateMachine.desiredBall);
    SmartDashboard.putBoolean("isRed", tunnelStateMachine.isRed);
    SmartDashboard.putBoolean("isBlue", tunnelStateMachine.isBlue);
    SmartDashboard.putNumber("Counter", tunnelStateMachine.counter);
    SmartDashboard.putString("Tunnel State", tunnelStateMachine.currentState.toString());
    SmartDashboard.putString("Is Running:", "Yes");
    SmartDashboard.putNumber("Amount of Red Detected:", frontColorSensor.getRed());
    SmartDashboard.putNumber("Amount of Blue Detected: ", frontColorSensor.getBlue());
  }

  public void commonPeriodic(){
    if (visiontable.getEntry("Confidence").getBoolean(false)) {
      autocorrectTargetAngle = currentAngle + horizontalAngle;
      confidenceCounter = 500;
    } else {
      if (confidenceCounter > 0) {
        confidenceCounter--;
      }
    }
  }
}