// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFXMotorController;
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
  public static Joystick buttonBox1;
  public static Joystick controllerOperator;
  MMJoystickAxis speedAxis, turnAxis;
  MMMotorGroup shooterWheels;
  MMMotorGroup shooterCAM;
  ShooterFormula shooterFormula;
  public static Solenoid lightRing;
  // public static Solenoid lightRing1;
  // public static Solenoid lightRing2;
  // public static Solenoid lightRing3;
  public static Solenoid lightRing4;
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
  public static QueueStateMachine queueStateMachine;
  public static TunnelStateMachine tunnelStateMachine;
  public static ShooterStateMachine shooterStateMachine;
  public static ClimbStateMachine climbStateMachine;
  public static double targetDistance;
  public static Intake intake;
  public static boolean intakeButton;
  public static boolean ejectButton;
  public static boolean autoLockHoop;
  public static boolean pointBlankButton;
  public static boolean shootOneButton;
  public static boolean shootAllButton;
  public static AimController aimController;
  public static boolean searchButton;
  public static PneumaticsControlModule pneumaticsControlModule;

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
    // TODO Confirm Hardware Sensor requirements.
    // check in each state machine and for the robot in general.
    // TODO Organize all human inputs into a single class with and update() call to
    // get data
    // convert button presses to more meaningful variables.
    // TODO ON-HOLD create custom PIDF controller that includes:
    // - small amount of error around zero to be ignored
    // - minimum correction to apply (if any +/- correction use at least a minimum
    // value)
    // - maximum correction to apply
    // TODO Implement Auto Select Dial
    // TODO Create In/Out ball counter
    // TODO Create Log

    nt = NetworkTableInstance.getDefault();
    visiontable = nt.getTable("Retroreflective Tape Target");
    // lightRing = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);

    // lightRing1 = new Solenoid(1, PneumaticsModuleType.CTREPCM, 4);
    // lightRing2 = new Solenoid(1, PneumaticsModuleType.CTREPCM, 5);
    // lightRing3 = new Solenoid(1, PneumaticsModuleType.CTREPCM, 6);
    // lightRing4 = new Solenoid(1, PneumaticsModuleType.CTREPCM, 7);
    

    navx = new AHRS(Port.kMXP);
    // navx = new AHRS(SerialPort.Port.USB);
    navx.reset();

    confidenceCounter = 0;

    controllerDriver = new Joystick(Constants.kJoystickDriver);
    controllerOperator = new Joystick(Constants.kJoystickOperator);
    speedAxis = new MMJoystickAxis(4, 1, .2, kMaxSpeed);
    turnAxis = new MMJoystickAxis(4, 4, .2, kMaxTurnRate);
    buttonBox1 = new Joystick(1);    

    shooterFormula = new ShooterFormula();
    SmartDashboard.putNumber("Target Distance", 0);

    intake = new Intake();
    queueStateMachine = new QueueStateMachine();
    shooterStateMachine = new ShooterStateMachine();
    tunnelStateMachine = new TunnelStateMachine();

    // climbStateMachine = new ClimbStateMachine();

    aimController = new AimController(/*turret*/);
    // pneumaticsControlModule = new
    // PneumaticsControlModule(Constants.kPneumaticsControlModule);

    driveTrain = new MMDiffDriveTrain(
        new MMFollowingMotorGroup(
            new MMFXMotorController(Constants.kCanMCDriveLeft1)
                .setStatorCurrentLimit(true, 40, 45, .5)
                .setInverted(Constants.kLeftMGInverted)
                .setPIDFParameters(Constants.kfalconDrivetrainKP, Constants.kfalconDrivetrainKI,
                    Constants.kfalconDrivetrainKD, Constants.kfalconDrivetrainKFF),
            new MMFXMotorController(Constants.kCanMCDriveLeft2)
                .setInverted(Constants.kLeftMGInverted) // This MUST MATCH LEAD!
        ),
        new MMFollowingMotorGroup(
            new MMFXMotorController(Constants.kCanMCDriveRight1)
                .setStatorCurrentLimit(true, 40, 45, .5)
                .setInverted(Constants.kRightMGInverted)
                .setPIDFParameters(Constants.kfalconDrivetrainKP, Constants.kfalconDrivetrainKI,
                    Constants.kfalconDrivetrainKD, Constants.kfalconDrivetrainKFF),
            new MMFXMotorController(Constants.kCanMCDriveRight2)
                .setInverted(Constants.kRightMGInverted) // This MUST MATCH LEAD!
        ),
        Constants.kNewRevPerFoot, Constants.kNewChassisRadius);

    // driveTrain = new MMDiffDriveTrain(
    // new MMFollowingMotorGroup(
    // new MMSparkMaxMotorController(kCanMCDriveLeft1, MotorType.kBrushless)
    // .setCurrentLimit(kNeoDriveTrainStallLimit, kNeoDriveTrainFreeLimit)
    // .setInverted(true)
    // .setPIDFParameters(kNeoDriveTrainP, kNeoDriveTrainI, kNeoDriveTrainD,
    // kNeoDriveTrainF, kNeoDriveTrainIZ,
    // kNeoDriveTrainMin, kNeoDriveTrainMax),
    // new MMSparkMaxMotorController(kCanMCDriveLeft2, MotorType.kBrushless),
    // new MMSparkMaxMotorController(kCanMCDriveLeft3, MotorType.kBrushless)),
    // new MMFollowingMotorGroup(
    // new MMSparkMaxMotorController(kCanMCDriveRight1, MotorType.kBrushless)
    // .setCurrentLimit(kNeoDriveTrainStallLimit, kNeoDriveTrainFreeLimit)
    // .setInverted(false)
    // .setPIDFParameters(kNeoDriveTrainP, kNeoDriveTrainI, kNeoDriveTrainD,
    // kNeoDriveTrainF, kNeoDriveTrainIZ,
    // kNeoDriveTrainMin, kNeoDriveTrainMax),
    // new MMSparkMaxMotorController(kCanMCDriveRight2, MotorType.kBrushless),
    // new MMSparkMaxMotorController(kCanMCDriveRight3, MotorType.kBrushless)),
    // kRevPerFoot, kChassiRadius);
    // pneumaticsControlModule.enableCompressorDigital();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    commonInit();
    autonomous = new TwoBallAuto();
  }

  @Override
  public void autonomousPeriodic() {
    commonPeriodic();

    autonomous.periodic();
    commonUpdate();
  }

  @Override
  public void teleopInit() {
    commonInit();
    // lightRing4.set(false);
    // climbStateMachine.currentState = ClimbStates.Start;
    // TODO: Comment these lines before competition
    // climbStateMachine.resetState();
    tunnelStateMachine.resetState();
    queueStateMachine.resetState();
    shooterStateMachine.resetState();
  }

  @Override
  public void teleopPeriodic() {
    commonPeriodic();

    double requestedSpeed = speedAxis.get();
    double requestedTurn = turnAxis.get();
    SmartDashboard.putNumber("Joystick Value", requestedSpeed);

    autoBallPickup = controllerDriver.getRawButton(Constants.kDriverAutoBallPickup);
    intakeButton = controllerDriver.getRawButton(Constants.kDriverIntake);
    ejectButton = controllerDriver.getRawButton(Constants.kDriverEject);
    pointBlankButton = controllerOperator.getPOV(Constants.kOperatorPointBlankPOV) == 0;
    autoLockHoop = controllerDriver.getRawButton(Constants.kDriverAutoTurnToTarget);

    if (intakeButton) {
      intake.intake();
    } else if (ejectButton) {
      intake.eject();
    } else {
      intake.idle();
    }
    if (controllerDriver.getRawButtonPressed(Constants.kDriverToggleBallLight)) {
      lightRing.set(!lightRing.get());
    }

    AimMode aimMode = AimMode.driver;

    if (autoLockHoop && confidenceCounter > 0) {
      // double p = 5;
      // double currentError = autocorrectTargetAngle - currentAngle;
      // // double currentError=xAngle- currentAngle;
      // requestedTurn = p * currentError;
      // SmartDashboard.putNumber("Auto Angle Correct", requestedTurn);
      aimMode = AimMode.robotShoot;
    }

    if (autoBallPickup /* && targetConfidence */) {
      // double p = 3;
      // requestedTurn = p * ballChaseAngle;
      // requestedSpeed = -1;
      aimMode = AimMode.ballChase;
    }

    aimController.setAimMode(aimMode);
    requestedTurn = aimController.calculate(requestedTurn, autocorrectTargetAngle, currentAngle, ballChaseAngle);
    driveTrain.Drive(requestedSpeed, requestedTurn);

    commonUpdate();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    commonInit();
    // tunnelStateMachine = new TunnelStateMachine();
    tunnelStateMachine.resetState();
    // climbStateMachine.resetState();
  }

  @Override
  public void testPeriodic() {
    commonPeriodic();
    commonUpdate();
  }

  public void commonInit() {
    alliance = DriverStation.getAlliance();
    navx.resetDisplacement();
  }

  public void commonPeriodic() {
    searchButton = controllerOperator.getRawButton(Constants.kOperatorSearchButton);

    shootOneButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootOne) > .7;
    shootAllButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootAll) > .7;

    currentAngle = cleanAngle(navx.getYaw());
    verticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000) + kCameraVerticalAngle;
    horizontalAngle = cleanAngle((Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000));
    targetDistance = kTargetingHeightDiff / Math.tan(Math.toRadians(verticalAngle));

    boolean targetConfidence = nt.getTable("Ball Target")
        .getEntry(alliance == Alliance.Blue ? "Blue Target Confidence" : "Red Target Confidence").getBoolean(false);
    ballChaseAngle = (Double) nt.getTable("Ball Target")
        .getEntry(alliance == Alliance.Blue ? "Blue Angle to Ball" : "Red Angle to Ball").getNumber(0);

    if (searchButton) {
      aimController.searchRequest();
    }

    if (visiontable.getEntry("Confidence").getBoolean(false)) {
      autocorrectTargetAngle = cleanAngle(currentAngle + horizontalAngle +
          (aimController.turret.getRevolutions() * Constants.kTurretDegreesPerRev));

      confidenceCounter = 500;
    } else {
      if (confidenceCounter > 0) {
        confidenceCounter--;
      }
    }
    // input distance via smartdashboard and then
    // double test = SmartDashboard.getNumber("Target Distance", 0);
    // TODO add condition for confidence and send inactive firing solution if there
    // is no confidence
    TargetPoint firingSolution = shooterFormula.calculate(pointBlankButton ? 0 : targetDistance);

    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);
    } else {
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);
      
      if (true || confidenceCounter > 0) {
        firingSolution.active = true;
      } else {
        firingSolution.active = false;
      }

      shooterStateMachine.setShootingSolution(firingSolution);
    }

  }

  public void commonUpdate() {
    tunnelStateMachine.update();
    // climbStateMachine.update();
    queueStateMachine.update();
    shooterStateMachine.update();

    SmartDashboard.putNumber("Navx Angle", currentAngle);
    SmartDashboard.putNumber("Position X: ", navx.getDisplacementX());
    SmartDashboard.putNumber("Position Y", navx.getDisplacementY());
    SmartDashboard.putNumber("Vertical Angle", verticalAngle);
    SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);
    SmartDashboard.putNumber("Target Distance", targetDistance);
    SmartDashboard.putNumber("TargetBallAngle", ballChaseAngle);
    SmartDashboard.putNumber("RobotDistance", driveTrain.getDistanceFeet());
    // SmartDashboard.putString("climbStateMachine",
    // climbStateMachine.currentState.toString());
    // SmartDashboard.putNumber("Climb Encoder Value",
    // climbStateMachine.climbMotor.getRevolutions());

    SmartDashboard.putNumber("ConfidenceCounter", confidenceCounter);
    SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());
    SmartDashboard.putString("Alliance Type", alliance.toString());

    // SmartDashboard.putString("CurrentState", autonomous.currentState.toString());
  }

  public static double cleanAngle(double angle) {

    return ((((angle + 180) % 360) + 360) % 360) - 180;
  }
}