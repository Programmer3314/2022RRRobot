// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;

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
  public static Joystick controllerDriver;
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
  public static PneumaticHub pneumaticHub;
  public static Solenoid shootLimeLight;
  public static boolean abortShootButton;
  public static boolean disableCompressor;
  public static boolean tacoBell; // Take out all balls within the robot
  public static PowerDistribution powerDistribution;
  public static boolean resetRobot;
  public static double adjustShooterDistance;
  public static boolean increaseDistance;
  public static boolean decreaseDistance;
  public static boolean teststopTunnel;
  public static double currentShooterAngle;
  public static String lastModeRan;

  /**
   * get joystick value and turn on shoot one or shoot all bool, want tap joystick
   * not hold it
   * if someone holds the shoot all button, shoot one ball then shoot all will be
   * false
   * and shoot one will become true
   * 
   */

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO IMMEDEYIT!!!!!!!!! BEFORE COMP
    // measure and tune cam angle/height and get those values in code
    // practice everything with bot
    //

    // Autonomous Select Button
    // Expanded MMPIDController with minOutput and maxOutput
    // Handle multiple same color balls in ball tracking/only detect same balls as
    // alliance

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
    // TODO Create In/Out ball counter
    // TODO Create Log

    // define variables use throughout code
    nt = NetworkTableInstance.getDefault();
    visiontable = nt.getTable("Retroreflective Tape Target");

    // Define devices that do not belong to a specific system
    powerDistribution = new PowerDistribution(Constants.kCanPowerDistributionBoard, ModuleType.kRev);
    powerDistribution.clearStickyFaults();
    navx = new AHRS(Port.kMXP);
    navx.reset();
    pneumaticHub = new PneumaticHub(Constants.kSolenoidModule);
    shootLimeLight = pneumaticHub.makeSolenoid(Constants.kShooterLimeLight);

    // Human Input devices
    controllerDriver = new Joystick(Constants.kJoystickDriver);
    controllerOperator = new Joystick(Constants.kJoystickOperator);
    speedAxis = new MMJoystickAxis(4, 1, .05, Constants.kMaxSpeed);
    turnAxis = new MMJoystickAxis(4, 4, .05, Constants.kMaxTurnRate);
    buttonBox1 = new Joystick(1);

    // Create Systems
    shooterFormula = new ShooterFormula();
    intake = new Intake();
    queueStateMachine = new QueueStateMachine();
    shooterStateMachine = new ShooterStateMachine();
    tunnelStateMachine = new TunnelStateMachine();
    climbStateMachine = new ClimbStateMachine();
    aimController = new AimController();
    driveTrain = new MMDiffDriveTrain(
        new MMFollowingMotorGroup(
            new MMFXMotorController(Constants.kCanMCDriveLeft1)
                .setStatorCurrentLimit(true, 40, 45, .5)
                .setInverted(Constants.kLeftMGInverted)
                .setPIDFParameters(Constants.kfalconDrivetrainKP, Constants.kfalconDrivetrainKI,
                    Constants.kfalconDrivetrainKD, Constants.kfalconDrivetrainKFF)
                .setBrakeMode(false),
            new MMFXMotorController(Constants.kCanMCDriveLeft2)
                .setInverted(Constants.kLeftMGInverted) // This MUST MATCH LEAD!
                .setBrakeMode(false)),
        new MMFollowingMotorGroup(
            new MMFXMotorController(Constants.kCanMCDriveRight1)
                .setStatorCurrentLimit(true, 40, 45, .5)
                .setInverted(Constants.kRightMGInverted)
                .setPIDFParameters(Constants.kfalconDrivetrainKP, Constants.kfalconDrivetrainKI,
                    Constants.kfalconDrivetrainKD, Constants.kfalconDrivetrainKFF)
                .setBrakeMode(false),
            new MMFXMotorController(Constants.kCanMCDriveRight2)
                .setInverted(Constants.kRightMGInverted)
                .setBrakeMode(false) // This MUST MATCH LEAD!
        ),
        Constants.kNewRevPerFoot, Constants.kNewChassisRadius);

    // Misc that should probably be better organized
    confidenceCounter = 0;
    pneumaticHub.enableCompressorDigital();
    SmartDashboard.putNumber("Target Distance", 0);
  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    commonInit();
    tunnelStateMachine.resetState();
    queueStateMachine.resetState();
    shooterStateMachine.resetState();
    aimController.resetTurret();
    aimController.setAimMode(AimMode.driver);
    autonomous = new TwoBallAuto();
    lastModeRan = "auto";
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

    aimController.setAimMode(AimMode.driver);
    if (lastModeRan == "teleop") {
      climbStateMachine.currentState = ClimbStates.Start;
      climbStateMachine.resetState();
      tunnelStateMachine.resetState();
      queueStateMachine.resetState();
      shooterStateMachine.resetState();
      aimController.resetTurret();
    }
    lastModeRan = "teleop";

  }

  @Override
  public void teleopPeriodic() {
    commonPeriodic();

    shootOneButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootOne) > .7;
    shootAllButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootAll) > .7;

    tacoBell = buttonBox1.getRawButton(Constants.kButtonBoxTacobell);
    double requestedSpeed = speedAxis.get();
    double requestedTurn = turnAxis.get();
    SmartDashboard.putNumber("Joystick Value", requestedSpeed);

    autoBallPickup = controllerDriver.getRawButton(Constants.kDriverAutoBallPickup);
    intakeButton = controllerDriver.getRawButton(Constants.kDriverIntake);
    ejectButton = controllerDriver.getRawButton(Constants.kDriverEject);
    pointBlankButton = controllerOperator.getPOV(Constants.kOperatorPointBlankPOV) == 0;
    autoLockHoop = controllerDriver.getRawButton(Constants.kDriverAutoTurnToTarget);
    increaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxIncreaseDistance);
    decreaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxDecreaseDistance);

    abortShootButton = controllerOperator.getRawButton(Constants.kOperatorAbortShot) ||
        buttonBox1.getRawButton(Constants.kButtonBoxAbortShot);
    if (increaseDistance) {
      adjustShooterDistance++;
    }
    if (decreaseDistance) {
      adjustShooterDistance--;
    }

    if (abortShootButton) {
      shooterStateMachine.abortShot();
    }

    if (shootAllButton) {
      shooterStateMachine.shootAll();

    } else if (shootOneButton) {
      shooterStateMachine.shootOne();
    }

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

    if (buttonBox1.getRawButton(Constants.kTestButtonBoxDisableCompressor)) {
      pneumaticHub.disableCompressor();
    }

    if (controllerDriver.getRawButtonPressed(Constants.kDriverAutoTurnToTarget) && confidenceCounter > 0) {
      aimController.setAimMode(AimMode.robotShoot);
    } else if (controllerDriver.getRawButtonReleased(Constants.kDriverAutoTurnToTarget)) {
      aimController.setAimMode(AimMode.driver);
    } else if (controllerDriver.getRawButtonPressed(Constants.kDriverAutoBallPickup)) {
      aimController.setAimMode(AimMode.ballChase);
    } else if (controllerDriver.getRawButtonReleased(Constants.kDriverAutoBallPickup)) {
      aimController.setAimMode(AimMode.driver);
    }
    requestedTurn = aimController.calculate(requestedTurn, autocorrectTargetAngle, currentAngle, ballChaseAngle);

    driveTrain.Drive(requestedSpeed, requestedTurn);

    SmartDashboard.putNumber("Manual Feed:", 0);

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
    // commonInit();
    // tunnelStateMachine = new TunnelStateMachine();
    // tunnelStateMachine.resetState();
    climbStateMachine.resetState();
    lastModeRan = "test";

  }

  @Override
  public void testPeriodic() {
    // commonPeriodic();
    // commonUpdate();
    // DigitalInput testClimbLimit = new DigitalInput(Constants.kDIOClimbLimit);
    // SmartDashboard.putBoolean("Climb Limit Value: ", testClimbLimit.get());
  }

  public void commonInit() {
    alliance = DriverStation.getAlliance();
    navx.resetDisplacement();
    shootLimeLight.set(true);
    intake.idle();

  }

  public void commonPeriodic() {
    searchButton = controllerOperator.getRawButton(Constants.kOperatorSearchButton);
    resetRobot = buttonBox1.getRawButton(Constants.kButtonBoxResetRobot);

    currentAngle = cleanAngle(navx.getYaw());
    currentShooterAngle = cleanAngle(
        currentAngle + (/* aimController.turret.getRevolutions() */ 0 * Constants.kTurretDegreesPerRev));
    verticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000) + Constants.kCameraVerticalAngle;
    horizontalAngle = cleanAngle((Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000));
    targetDistance = Constants.kTargetingHeightDiff / Math.tan(Math.toRadians(verticalAngle));

    boolean targetConfidence = nt.getTable("Ball Target")
        .getEntry(alliance == Alliance.Blue ? "Blue Target Confidence" : "Red Target Confidence").getBoolean(false);
    ballChaseAngle = (Double) nt.getTable("Ball Target")
        .getEntry(alliance == Alliance.Blue ? "Blue Angle to Ball" : "Red Angle to Ball").getNumber(0);

    if (searchButton) {
      aimController.searchRequest();
    }

    if (resetRobot) {
      shooterStateMachine.resetState();
      tunnelStateMachine.resetState();
      queueStateMachine.resetState();
      climbStateMachine.resetState();
    }
    // TODO Clean up autocorrectTargetAngle/firingSolution with respect to
    // when they are valid. I'm not sure what this looks like but it seems odd
    // that we use .active and confidenceCounter...
    if (visiontable.getEntry("Confidence").getBoolean(false)) {
      autocorrectTargetAngle = cleanAngle(currentAngle + horizontalAngle +
          (/* aimController.turret.getRevolutions() */ 0 * Constants.kTurretDegreesPerRev));

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
    SmartDashboard.putNumber("Adjust Shooter Distance", adjustShooterDistance);
    TargetPoint firingSolution = shooterFormula
        .calculate(pointBlankButton ? 0 : targetDistance + adjustShooterDistance);

    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);
    } else {
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);

      if (confidenceCounter > 0) {
        firingSolution.active = true;
      } else {
        firingSolution.active = false;
      }

      shooterStateMachine.setShootingSolution(firingSolution);
    }

  }

  public void commonUpdate() {
    tunnelStateMachine.update();
    climbStateMachine.update();
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
    SmartDashboard.putString("climbStateMachine",
        climbStateMachine.currentState.toString());
    SmartDashboard.putNumber("Climb Encoder Value",
        climbStateMachine.climbMotor.getRevolutions());

    SmartDashboard.putNumber("ConfidenceCounter", confidenceCounter);
    SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());
    SmartDashboard.putString("Alliance Type", alliance.toString());

    // SmartDashboard.putString("CurrentState", autonomous.currentState.toString());
  }

  public static double cleanAngle(double angle) {

    return ((((angle + 180) % 360) + 360) % 360) - 180;
  }
}