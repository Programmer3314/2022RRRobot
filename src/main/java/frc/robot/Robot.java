// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.utility.MMBCDReturn;
import frc.robot.utility.MMDiffDriveTrain;
import frc.robot.utility.MMEdgeTrigger;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

enum Position {
  Left, Right, Center
}// 13/16-4 3/8= TODO 5.385

enum ShotType {
  PointBlankLow, PointBlankHigh, Vision, OperatorUp, OperatorLeft, OperatorRight, OperatorDown, Blind
}

public class Robot extends TimedRobot {
  public static MMDiffDriveTrain driveTrain;
  public static Joystick controllerDriver;
  public static Joystick buttonBox1;
  public static Joystick buttonBox2;
  public static Joystick controllerOperator;
  MMJoystickAxis speedAxis, turnAxis;
  MMMotorGroup shooterWheels;
  MMMotorGroup shooterCAM;
  ShooterFormula shooterFormula;
  // public static Solenoid lightRing;
  // public static Solenoid lightRing1;
  // public static Solenoid lightRing2;
  // public static Solenoid lightRing3;
  public static Solenoid lightRing4;
  public static NetworkTableInstance nt;
  public static NetworkTable visiontable;
  public static NetworkTable limelightTable;
  public static AHRS navx;
  public static double hubTargetAngle;
  public static int confidenceCounter;
  public static double currentRobotAngle;
  public static double hubVerticalAngle;
  public static double hubHorizontalAngle;
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
  public static boolean pointBlankHigh;
  public static boolean shootOneButton;
  public static boolean povLeftShot;
  public static boolean povRightShot;
  public static boolean shootAllButton;
  public static AimController aimController;
  public static boolean searchButton;
  public static PneumaticsControlModule pneumaticHub;
  public static Solenoid shootLimeLight;
  public static boolean stopWhiteBelt;
  public static boolean abortShootButton;
  public static boolean disableCompressor;
  public static boolean tacoBell; // Take out all balls within the robot
  // public static PowerDistribution powerDistribution;
  public static boolean resetRobot;
  public static double adjustShooterDistance;
  public static boolean increaseDistance;
  public static boolean decreaseDistance;
  public static boolean teststopTunnel;
  public static double currentShooterAngle;
  public static String lastModeRan;
  public static boolean pointBlankLow;
  public static double targetpovdistance;
  public static boolean logEvent;
  public static boolean autoLockBar;
  public static int autoSelect;
  public static MMBCDReturn bcdReturn;
  public static Position tarmacPosition;
  public static NavXRoll navXRoll;
  public static boolean useLimeLight = true;
  public static double snapshotCounterLeft = 32;
  public static boolean useVision;
  public static NetworkTableEntry ledMode;
  public static NetworkTableEntry camMode;
  public static NetworkTableEntry snapshot;
  public static TargetSamples targetSamples;
  // public static boolean driverShootHigh;
  // public static boolean driverShootLow;
  public static boolean povUpShot;
  public static boolean povDownShot;
  public static Number[] llpython;
  public static MMBCDReturn pipelineDial;
  public static boolean targetBallConfidence;
  public static boolean increaseCam;
  public static boolean decreaseCam;
  public static MMEdgeTrigger pointBlanklowgoalAxis;
  public static MMEdgeTrigger pointBlankHighGoalAxis;
  public static MMEdgeTrigger povOperatorLeft;
  public static MMEdgeTrigger povOperatorRight;
  public static MMEdgeTrigger povOperatorUp;
  public static MMEdgeTrigger povOperatorDown;
  public static boolean driverLockHoop;
  public static ShotType shotType;
  public static int noTargetRumble;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    adjustShooterDistance = 0;// .5
    Logger.Enabled = true;
    // TODO IMMEDEYIT!!!!!!!!! BEFORE COMP

    // TODO ON-HOLD create custom PIDF controller that includes:
    // - small amount of error around zero to be ignored
    // - minimum correction to apply (if any +/- correction use at least a minimum
    // value)
    // - maximum correction to apply
    // TODO On-Hold Organize all human inputs into a single class with and update()
    // call to
    // get data
    // convert button presses to more meaningful variables.
    // TODO On-Hold Expanded MMPIDController with minOutput and maxOutput
    // TODO On-Hold CLEANUP Organize the init code to group simillar code
    // like Motor devices together, Human inputs together,
    // Sensors together, Data init, etc.

    // define variables use throughout code
    nt = NetworkTableInstance.getDefault();
    visiontable = nt.getTable("Retroreflective Tape Target");
    limelightTable = nt.getTable("limelight");

    ledMode = limelightTable.getEntry("ledMode");
    camMode = limelightTable.getEntry("camMode");
    snapshot = limelightTable.getEntry("snapshot");

    // Define devices that do not belong to a specific system
    navx = new AHRS(Port.kMXP);
    navx.reset();
    pneumaticHub = new PneumaticsControlModule(Constants.kSolenoidModule);
    shootLimeLight = pneumaticHub.makeSolenoid(Constants.kShooterLimeLight);

    // Human Input devices
    controllerDriver = new Joystick(Constants.kJoystickDriver);
    controllerOperator = new Joystick(Constants.kJoystickOperator);
    speedAxis = new MMJoystickAxis(4, 1, .05, Constants.kMaxSpeed);
    turnAxis = new MMJoystickAxis(4, 4, .05, Constants.kMaxTurnRate);
    buttonBox1 = new Joystick(1);
    buttonBox2 = new Joystick(2);

    pointBlankHighGoalAxis = new MMEdgeTrigger();
    pointBlanklowgoalAxis = new MMEdgeTrigger();
    povOperatorLeft = new MMEdgeTrigger();
    povOperatorRight = new MMEdgeTrigger();
    povOperatorUp = new MMEdgeTrigger();
    povOperatorDown = new MMEdgeTrigger();

    // Create Systems
    shooterFormula = new ShooterFormula();
    navXRoll = new NavXRoll();
    bcdReturn = new MMBCDReturn(buttonBox1, 13, 14, 15, 16);
    pipelineDial = new MMBCDReturn(buttonBox2, 10, 11, 12, -1);
    intake = new Intake();
    targetSamples = new TargetSamples();
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
    snapshotCounterLeft = 32;
    Logger.OpenLog("Auto");
    RobotLogHeader();
    commonInit();
    autoSelect = bcdReturn.GetDial();
    driveTrain.resetEncoders();
    tunnelStateMachine.resetState();
    queueStateMachine.resetState();
    shooterStateMachine.resetState();
    aimController.resetTurret();
    aimController.setAimMode(AimMode.driver);

    lastModeRan = "auto";

    configVision();

    if (buttonBox2.getRawButton(13)) {
      tarmacPosition = Position.Left;
    } else {
      if (buttonBox2.getRawButton(14)) {
        tarmacPosition = Position.Right;
      } else {
        tarmacPosition = Position.Center;
      }
    }
    autonomous = new TwoBallAuto(tarmacPosition, autoSelect);
    // autonomous = new AdditionalAuto(tarmacPosition, autoSelect);
    autonomous.LogHeader();
    Logger.EndLine();
  }

  @Override
  public void autonomousPeriodic() {
    Logger.StartLine();
    commonPeriodic();

    shotType = useVision ? ShotType.Vision : ShotType.Blind;

    RobotLogData();

    autonomous.LogData();

    autonomous.periodic();
    commonUpdate();
    SmartDashboard.putString("NewAutoState", autonomous.currentState.toString());
    SmartDashboard.putNumber("Driver Distance", driveTrain.getDistanceFeet());
    Logger.EndLine();
  }

  @Override
  public void teleopInit() {
    Logger.OpenLog("Tele");
    commonInit();
    RobotLogHeader();

    aimController.setAimMode(AimMode.driver);
    shooterStateMachine.abortShot();
    
    if (lastModeRan == "teleop") {
      climbStateMachine.currentState = ClimbStates.Start;
      climbStateMachine.resetState();
      tunnelStateMachine.resetState();
      queueStateMachine.resetState();
      shooterStateMachine.resetState();
      aimController.resetTurret();
      snapshotCounterLeft = 32;
    }
    lastModeRan = "teleop";
    shooterStateMachine.shootOne = false;
    Logger.EndLine();
  }

  @Override
  public void teleopPeriodic() {
    Logger.StartLine();
    RobotLogData();
    commonPeriodic();
    navXRoll.update(navx.getRoll());
    configVision();

    stopWhiteBelt = buttonBox1.getRawButtonPressed(10);
    // shootOneButton =
    // controllerOperator.getRawAxis(Constants.kOperatorAxisShootOne) > .7;
    // shootAllButton =
    // controllerOperator.getRawAxis(Constants.kOperatorAxisShootAll) > .7;
    logEvent = buttonBox1.getRawButton(Constants.kButtonBoxErrorButton);

    increaseCam = buttonBox1.getRawButton(Constants.kButtonBoxIncreaseCam);
    decreaseCam = buttonBox1.getRawButton(Constants.kButtonBoxDecreaseCam);

    tacoBell = buttonBox1.getRawButton(Constants.kButtonBoxTacobell);
    double requestedSpeed = speedAxis.get();
    // double requestedTurn = turnAxis.get();
    double requestedTurn = turnAxis.getSquared();
    SmartDashboard.putNumber("Joystick Value", requestedSpeed);
    SmartDashboard.putNumber("Returned Speed", driveTrain.getVelocity());

    autoBallPickup = controllerDriver.getRawButton(Constants.kDriverAutoBallPickup);
    intakeButton = controllerDriver.getRawButton(Constants.kDriverIntake);
    ejectButton = controllerDriver.getRawButton(Constants.kDriverEject);
    pointBlankLow = pointBlanklowgoalAxis.update(controllerDriver.getRawAxis(2) > .7).transitionHigh();
    pointBlankHigh = pointBlankHighGoalAxis.update(controllerDriver.getRawAxis(3) > .7).transitionHigh();

    povLeftShot = povOperatorLeft.update(controllerOperator.getPOV(Constants.kOperatorPOV) == 270).transitionHigh();
    povRightShot = povOperatorRight.update(controllerOperator.getPOV(Constants.kOperatorPOV) == 90).transitionHigh();
    povUpShot = povOperatorRight.update(controllerOperator.getPOV(Constants.kOperatorPOV) == 0).transitionHigh();
    povDownShot = povOperatorDown.update(controllerOperator.getPOV(Constants.kOperatorPOV) == 180).transitionHigh();

    // TODO consider rumbling stick if autoLookHoop and no target active (or
    // confidenceCounter==0)
    autoLockHoop = controllerDriver.getRawButtonPressed(Constants.kDriverAutoTurnToTarget);
    increaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxIncreaseDistance);
    decreaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxDecreaseDistance);
    SmartDashboard.putBoolean("Intake Out:", intakeButton);

    if (buttonBox1.getRawButtonPressed(Constants.kButtonBoxErrorButton)
        || autoLockHoop) {
      takeSnapshot();
    }

    abortShootButton = controllerOperator.getRawButton(Constants.kOperatorAbortShot) ||
        buttonBox1.getRawButton(Constants.kButtonBoxAbortShot)
        || controllerDriver.getRawButton(7);
    if (increaseDistance) {
      adjustShooterDistance += .5;
    }
    if (decreaseDistance) {
      adjustShooterDistance -= .5;
    }

    if (abortShootButton) {
      shooterStateMachine.abortShot();
    }

    // if (shootAllButton || driverShootHigh || driverShootLow) {
    // shooterStateMachine.shootAll();
    // } else if (shootOneButton) {
    // shooterStateMachine.shootOne();
    // }

    if (intakeButton) {
      intake.intake();
    } else if (ejectButton) {
      intake.eject();
    } else {
      intake.idle();
    }
    if (controllerDriver.getRawButtonPressed(Constants.kDriverToggleBallLight)) {
      shootLimeLight.set(!shootLimeLight.get());
    }

    // if (buttonBox1.getRawButton(Constants.kTestButtonBoxDisableCompressor)) {
    // pneumaticHub.disableCompressor();
    // }

    if (autoLockHoop && confidenceCounter > 0) {
      aimController.setAimMode(AimMode.robotShoot);
      shooterStateMachine.shootAll();
    } else if (controllerDriver.getRawButtonReleased(Constants.kDriverAutoTurnToTarget)) {
      aimController.setAimMode(AimMode.driver);
    } else if (controllerDriver.getRawButtonPressed(Constants.kDriverAutoBallPickup)) {
      aimController.setAimMode(AimMode.ballChase);
    } else if (controllerDriver.getRawButtonReleased(Constants.kDriverAutoBallPickup)) {
      aimController.setAimMode(AimMode.driver);
    } else if (controllerDriver.getRawButtonPressed(Constants.kDriverAutoBarLock)) {
      aimController.setAimMode(AimMode.lockBar);
    } else if (controllerDriver.getRawButtonReleased(Constants.kDriverAutoBarLock)) {
      aimController.setAimMode(AimMode.driver);
    }

    // if(increaseCam){
    // shooterStateMachine.camAngle.setPower(.1);
    // }else if(decreaseCam){
    // shooterStateMachine.camAngle.setPower(-.1);
    // }else{
    // shooterStateMachine.camAngle.setPower(0);
    // }

    if (stopWhiteBelt) {
      tunnelStateMachine.toggleBelt();
    }

    DriveParameters dp = aimController.calculate(requestedTurn, hubTargetAngle, currentRobotAngle, ballChaseAngle,
        climbStateMachine.leadHookContactLeft, climbStateMachine.leadHookContactRight, requestedSpeed);

    requestedTurn = dp.turn;
    requestedSpeed = dp.drive;
    driveTrain.Drive(requestedSpeed, requestedTurn);

    SmartDashboard.putNumber("Manual Feed:", 0);
    SmartDashboard.putNumber("intake Speed: ", intake.intakeMotor.getVelocity());
    SmartDashboard.putNumber("Requested Turn: ", requestedTurn);
    SmartDashboard.putNumber("Intake RPM", intake.intakeMotor.getVelocity());
    SmartDashboard.putNumber("Yaw", navx.getYaw());
    SmartDashboard.putNumber("Pitch", navx.getPitch());
    SmartDashboard.putNumber("Roll", navx.getRoll());

    // tunnelStateMachine.LogData();

    commonUpdate();
    Logger.EndLine();
  }

  @Override
  public void disabledInit() {
    visiontable.getEntry("Enable Log").setBoolean(false);
    ledMode.setNumber(0);
  }

  @Override
  public void disabledPeriodic() {
    Logger.CloseLog();
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
    useVision = !buttonBox1.getRawButton(Constants.kButtonBoxIgnoreVision);
    shotType = ShotType.Vision;

    if (useLimeLight) {
      ledMode.setNumber(3);
    } else {
      shootLimeLight.set(true);
    }
    visiontable.getEntry("Enable Log").setBoolean(false);
    intake.idle();
  }

  public void commonPeriodic() {
    // searchButton =
    // controllerOperator.getRawButton(Constants.kOperatorSearchButton);
    resetRobot = buttonBox1.getRawButton(Constants.kButtonBoxResetRobot);
    useVision = !buttonBox1.getRawButton(Constants.kButtonBoxIgnoreVision);

    // TODO Let's clean up variable names to make them clearer...
    // There are several angles, so maybe currentAngle should be currentRobotAngle
    currentRobotAngle = cleanAngle(navx.getYaw());
    currentShooterAngle = cleanAngle(
        currentRobotAngle + (/* aimController.turret.getRevolutions() */ 0 * Constants.kTurretDegreesPerRev));
    

    targetBallConfidence = nt.getTable("Ball Target")
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
    boolean haveTarget;
    if (!useLimeLight) {
      haveTarget = visiontable.getEntry("Confidence").getBoolean(false);
      hubVerticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000)
          + Constants.kCameraVerticalAngle;
      hubHorizontalAngle = cleanAngle((Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000));
    } else {
      // verticalAngle = (Double) limelighTable.getEntry("ty").getNumber(-5000) +
      // Constants.kCameraVerticalAngle;
      // horizontalAngle = targetSamples.update(cleanAngle((Double)
      // limelighTable.getEntry("tx").getNumber(-5000)));
      llpython = limelightTable.getEntry("llpython").getNumberArray(new Number[] { 0, 0, 0 });
      if (llpython.length > 2) {
        haveTarget = llpython.length > 0 && (Double) llpython[0] == 1.0;
        hubVerticalAngle = (Double) llpython[2] + Constants.kCameraVerticalAngle;
        hubHorizontalAngle = targetSamples.update((Double) llpython[1]);
      } else {
        haveTarget = false;
        hubVerticalAngle = 0;
        hubHorizontalAngle = 0;
      }
    }
    if (haveTarget) {
      hubTargetAngle = cleanAngle(currentRobotAngle + hubHorizontalAngle +
          (/* aimController.turret.getRevolutions() */ 0 * Constants.kTurretDegreesPerRev));
      targetDistance = Constants.kTargetingHeightDiff / Math.tan(Math.toRadians(hubVerticalAngle));
      confidenceCounter = 200;
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
    SmartDashboard.putBoolean("Point Blank Button", pointBlankHigh);
    SmartDashboard.putBoolean("Bottom Basket Button", pointBlankLow);
    SmartDashboard.putBoolean("HaveBallTarget:", targetBallConfidence);

    // TargetPoint firingSolution = shooterFormula
    // .calculate(pointBlankButton ? 0 : targetDistance + adjustShooterDistance);

    // TODO change the flow of this to use something like "overridenHubDistance"
    // and set it to the automatic distance (from the active vision system +
    // adjustment)
    // then set it to a fixed value it one of the following overrides applies
    // It will have a cleaner feel - I hope.
    if (lastModeRan == "auto" && autoSelect == 3) {
      if (tarmacPosition == Position.Left || tarmacPosition == Position.Right) {
        targetpovdistance = 9;
      } else {
        targetpovdistance = 13;
      }
    }
    if (pointBlankHigh) {
      shotType = ShotType.PointBlankHigh;
      shooterStateMachine.shootAll();
    } else if (pointBlankLow) {
      shotType = ShotType.PointBlankLow;
      shooterStateMachine.shootAll();
    } else if (povRightShot) {
      shotType = ShotType.OperatorRight;
      shooterStateMachine.shootAll();
    } else if (povLeftShot) {
      shotType = ShotType.OperatorLeft;
      shooterStateMachine.shootAll();
    } else if (povUpShot) {
      shotType = ShotType.OperatorUp;
      shooterStateMachine.shootAll();
    } else if (povDownShot) {
      shotType = ShotType.OperatorDown;
      shooterStateMachine.shootAll();
    } else if (autoLockHoop) { // TODO Look at other references. The drive mode only changes when
                               // confidenceCounter is >0 maybe this should be like that too.
      shotType = ShotType.Vision;
      shooterStateMachine.shootAll();
      if (!shooterStateMachine.target.active) {
        controllerDriver.setRumble(RumbleType.kRightRumble, 1);
        noTargetRumble = 50;
      }
    }

    if (noTargetRumble > 0) {
      noTargetRumble--;
    }
    if (noTargetRumble == 1) {
      controllerDriver.setRumble(RumbleType.kRightRumble, 0);
    }
    switch (shotType) {
      case PointBlankHigh:
        targetpovdistance = 0;
        break;
      case PointBlankLow:
        targetpovdistance = -5;
        break;
      case Vision:
        targetpovdistance = targetDistance + adjustShooterDistance;
        break;
      case OperatorUp:
        targetpovdistance = 7.5;
        break;
      case OperatorDown:
        targetpovdistance = -2;
        break;
      case OperatorLeft:
        targetpovdistance = -1;
        break;
      case OperatorRight:
        targetpovdistance = -4;
        break;
    }
    TargetPoint firingSolution = shooterFormula
        .calculate(targetpovdistance);

    // SmartDashboard.putNumber("Target Distance: ", firingsolution.distance);
    if (firingSolution == null) {
      SmartDashboard.putNumber("TargetRPM", -1);
      SmartDashboard.putNumber("TargetAngle", -1);
    } else {
      SmartDashboard.putNumber("TargetRPM", firingSolution.rpm);
      SmartDashboard.putNumber("TargetAngle", firingSolution.angle);

      if (confidenceCounter > 0 || shotType!=ShotType.Vision) {
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

    SmartDashboard.putNumber("Navx Angle", currentRobotAngle);
    SmartDashboard.putNumber("Position X: ", navx.getDisplacementX());
    SmartDashboard.putNumber("Position Y", navx.getDisplacementY());
    SmartDashboard.putNumber("Vertical Angle", hubVerticalAngle);
    SmartDashboard.putNumber("Horizontal Angle", hubHorizontalAngle);
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
    SmartDashboard.putNumber("pipeline", pipelineDial.GetDial());
    // SmartDashboard.putNumber("Time:", value);

    // SmartDashboard.putString("CurrentState", autonomous.currentState.toString());
  }

  public static double cleanAngle(double angle) {

    return ((((angle + 180) % 360) + 360) % 360) - 180;
  }

  public void RobotLogHeader() {
    Logger.Header(
        "EVENT, ShootOne, ShootAll, TACOBELL, AutoPickup, IntakeButton, EjectButton, PointBlank, POVLeft, POVRight, BottomBasket, Aimbot, UpDistance, DownDistance,Yaw,Pitch,Roll,targetPOVData,");
    // aimController.LogHeader();
    driveTrain.LogHeadder();
    climbStateMachine.LogHeader();
    intake.LogHeader();
    queueStateMachine.LogHeader();
    shooterStateMachine.LogHeader();
    tunnelStateMachine.LogHeader();
    aimController.LogHeader();
    navXRoll.LogHeader();
    
  }

  public void RobotLogData() {
    Logger.booleans(logEvent, shootOneButton, shootAllButton, tacoBell);
    Logger.booleans(autoBallPickup, intakeButton, ejectButton, pointBlankHigh);
    Logger.booleans(povLeftShot, povRightShot, pointBlankLow, autoLockHoop, increaseDistance, decreaseDistance);
    Logger.doubles(navx.getYaw(), navx.getPitch(), navx.getRoll(), targetpovdistance);
    driveTrain.LogData();
    climbStateMachine.LogData();
    intake.LogData();
    queueStateMachine.LogData();
    shooterStateMachine.LogData();
    tunnelStateMachine.LogData();
    aimController.LogData();
    navXRoll.LogData();

  }

  public void configVision() {
    if (useVision) {
      ledMode.setNumber(3);
      camMode.setNumber(0);
      int pipeline = pipelineDial.GetDial();
      limelightTable.getEntry("pipeline").setNumber(pipeline);

    } else {
      ledMode.setNumber(1);
      limelightTable.getEntry("pipeline").setNumber(2);
      // camMode.setNumber(1);
    }
  }

  public static void takeSnapshot() {
    if (snapshotCounterLeft > 0) {
      snapshot.setNumber(1);
      snapshotCounterLeft--;
    }
  }

}