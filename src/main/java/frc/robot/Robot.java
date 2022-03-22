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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.utility.MMBCDReturn;
import frc.robot.utility.MMDiffDriveTrain;
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
  public static NetworkTable limelighTable;
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
  public static boolean bottomBasket;
  public static double targetpovdistance;
  public static boolean logEvent;
  public static boolean autoLockBar;
  public static int autoSelect;
  public static MMBCDReturn bcdReturn;
  public static Position position;
  public static NavXRoll navXRoll;
  public static boolean useLimeLight=true;
  public static double snapshotCounterLeft=32;
  public static boolean useVision;
  public static NetworkTableEntry ledMode;
  public static NetworkTableEntry camMode;
  public static NetworkTableEntry snapshot;
  public static TargetSamples targetSamples;
  public static boolean driverShootHigh;
  public static boolean driverShootLow;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    adjustShooterDistance = .5;
    Logger.Enabled = true;
    // TODO IMMEDEYIT!!!!!!!!! BEFORE COMP

    // TODO Transfer Shooting to Driver Control:
    // Driver's A button will align and then shoot all high. (see note below ~line 357)
    // Driver's Left Trigger will do Low Batter Shot
    // Driver's Right Trigger will do High Batter Shot (point blank)

    // TODO On-Hold optimize ball camera
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
    limelighTable = nt.getTable("limelight");

    ledMode = limelighTable.getEntry("ledMode");
    camMode = limelighTable.getEntry("camMode");
    snapshot = limelighTable.getEntry("snapshot");

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

    // Create Systems
    shooterFormula = new ShooterFormula();
    navXRoll = new NavXRoll();
    bcdReturn = new MMBCDReturn();
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
    snapshotCounterLeft=32;
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
      position = Position.Left;
    } else {
      if (buttonBox2.getRawButton(14)) {
        position = Position.Right;
      } else {
        position = Position.Center;
      }
    }
    autonomous = new TwoBallAuto(position, autoSelect);
    Logger.EndLine();
  }

  @Override
  public void autonomousPeriodic() {
    Logger.StartLine();
    RobotLogData();
    commonPeriodic();
    autonomous.periodic();
    commonUpdate();
    SmartDashboard.putNumber("Driver Distance", driveTrain.getDistanceFeet());
    Logger.EndLine();
  }

  @Override
  public void teleopInit() {
    Logger.OpenLog("Tele");
    commonInit();
    RobotLogHeader();

    aimController.setAimMode(AimMode.driver);
    if (lastModeRan == "teleop") {
      climbStateMachine.currentState = ClimbStates.Start;
      climbStateMachine.resetState();
      tunnelStateMachine.resetState();
      queueStateMachine.resetState();
      shooterStateMachine.resetState();
      aimController.resetTurret();
      snapshotCounterLeft=32;
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
    useVision = !buttonBox1.getRawButton(12);
    configVision();

    stopWhiteBelt = buttonBox1.getRawButtonPressed(10);
    shootOneButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootOne) > .7;
    shootAllButton = controllerOperator.getRawAxis(Constants.kOperatorAxisShootAll) > .7;
    logEvent = buttonBox1.getRawButton(Constants.kButtonBoxErrorButton);

    tacoBell = buttonBox1.getRawButton(Constants.kButtonBoxTacobell);
    double requestedSpeed = speedAxis.get();
    double requestedTurn = turnAxis.get();
    SmartDashboard.putNumber("Joystick Value", requestedSpeed);
    SmartDashboard.putNumber("Returned Speed", driveTrain.getVelocity());

    autoBallPickup = controllerDriver.getRawButton(Constants.kDriverAutoBallPickup);
    intakeButton = controllerDriver.getRawButton(Constants.kDriverIntake);
    ejectButton = controllerDriver.getRawButton(Constants.kDriverEject);
    driverShootLow=controllerDriver.getRawAxis(2)>.7;
    driverShootHigh = controllerDriver.getRawAxis(3)>.7;
    pointBlankButton = controllerOperator.getPOV(Constants.kOperatorPOV) == 0||driverShootHigh;
    bottomBasket = controllerOperator.getPOV(Constants.kOperatorPOV) == 180||driverShootLow;

    povLeftShot = controllerOperator.getPOV(Constants.kOperatorPOV) == 270;
    povRightShot = controllerOperator.getPOV(Constants.kOperatorPOV) == 90;

    autoLockHoop = controllerDriver.getRawButtonPressed(Constants.kDriverAutoTurnToTarget);
    increaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxIncreaseDistance);
    decreaseDistance = buttonBox1.getRawButtonPressed(Constants.kButtonBoxDecreaseDistance);
    SmartDashboard.putBoolean("Intake Out:", intakeButton);



    if(buttonBox1.getRawButtonPressed(Constants.kButtonBoxErrorButton) 
    || autoLockHoop){
      takeSnapshot();
    }

    abortShootButton = controllerOperator.getRawButton(Constants.kOperatorAbortShot) ||
        buttonBox1.getRawButton(Constants.kButtonBoxAbortShot);
    if (increaseDistance) {
      adjustShooterDistance += .5;
    }
    if (decreaseDistance) {
      adjustShooterDistance -= .5;
    }

    if (abortShootButton) {
      shooterStateMachine.abortShot();
    }

    if (shootAllButton||driverShootHigh||driverShootLow) {
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
      shootLimeLight.set(!shootLimeLight.get());
    }

    if (buttonBox1.getRawButton(Constants.kTestButtonBoxDisableCompressor)) {
      pneumaticHub.disableCompressor();
    }

    // TODO Make autoLockHoop also trigger a FireAll. 
    // TODO Also make FireAll deal with one ball. 
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

    if(stopWhiteBelt){
     tunnelStateMachine.toggleBelt();
    }

    DriveParameters dp = aimController.calculate(requestedTurn, autocorrectTargetAngle, currentAngle, ballChaseAngle,
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
    useVision = !buttonBox1.getRawButton(12);

    if (useLimeLight){
      ledMode.setNumber(3);

    }
    else{
    shootLimeLight.set(true);
    }
    visiontable.getEntry("Enable Log").setBoolean(false);
    intake.idle();
  }

  public void commonPeriodic() {
    // searchButton =
    // controllerOperator.getRawButton(Constants.kOperatorSearchButton);
    resetRobot = buttonBox1.getRawButton(Constants.kButtonBoxResetRobot);

    // TODO Let's clean up variable names to make them clearer...
    // There are several angles, so maybe currentAngle should be currentRobotAngle
    currentAngle = cleanAngle(navx.getYaw());
    currentShooterAngle = cleanAngle(
        currentAngle + (/* aimController.turret.getRevolutions() */ 0 * Constants.kTurretDegreesPerRev));
    if (!useLimeLight) {
      verticalAngle = (Double) visiontable.getEntry("Vertical Angle").getNumber(-5000) + Constants.kCameraVerticalAngle;
      horizontalAngle = cleanAngle((Double) visiontable.getEntry("Horizontal Angle").getNumber(-5000));
    } else {
      verticalAngle = (Double) limelighTable.getEntry("ty").getNumber(-5000) + Constants.kCameraVerticalAngle;
      horizontalAngle = targetSamples.update(cleanAngle((Double) limelighTable.getEntry("tx").getNumber(-5000)));
    }
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
    boolean haveTarget;
    if (!useLimeLight) {
      haveTarget = visiontable.getEntry("Confidence").getBoolean(false);
    } else {
      haveTarget = (Double) limelighTable.getEntry("tv").getNumber(0) == 1.0;
    }
    if (haveTarget) {
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
    SmartDashboard.putBoolean("Point Blank Button", pointBlankButton);
    SmartDashboard.putBoolean("Bottom Basket Button", bottomBasket);

    // TargetPoint firingSolution = shooterFormula
    // .calculate(pointBlankButton ? 0 : targetDistance + adjustShooterDistance);

    // TODO change the flow of this to use something like "overridenHubDistance"
    // and set it to the automatic distance (from the active vision system +
    // adjustment)
    // then set it to a fixed value it one of the following overrides applies
    // It will have a cleaner feel - I hope.
    if (lastModeRan == "auto" && autoSelect == 3) {
      if (position == Position.Left || position == Position.Right) {
        targetpovdistance = 9;
      } else {
        targetpovdistance = 13;
      }

    }
    if (pointBlankButton) {
      targetpovdistance = 0;
    } else if (bottomBasket) {
      targetpovdistance = -5;
    } else if (povRightShot) {
      targetpovdistance = -2;
    } else if (povRightShot) {
      targetpovdistance = -1;
    } else {
      targetpovdistance = targetDistance + adjustShooterDistance;
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

      if (confidenceCounter > 0 || pointBlankButton || bottomBasket || povRightShot || povLeftShot) {
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
    // SmartDashboard.putNumber("Time:", value);

    // SmartDashboard.putString("CurrentState", autonomous.currentState.toString());
  }

  public static double cleanAngle(double angle) {

    return ((((angle + 180) % 360) + 360) % 360) - 180;
  }

  public void RobotLogHeader() {
    Logger.Header(
        "EVENT, ShootOne, ShootAll, TACOBELL, AutoPickup, IntakeButton, EjectButton, PointBlank, POVLeft, POVRight, BottomBasket, Aimbot, UpDistance, DownDistance,Yaw,Pitch,Roll,");
    // aimController.LogHeader();
    climbStateMachine.LogHeader();
    intake.LogHeader();
    queueStateMachine.LogHeader();
    shooterStateMachine.LogHeader();
    tunnelStateMachine.LogHeader();
    aimController.LogHeader();
    navXRoll.LogHeader();
  }

  public void RobotLogData() {
    Logger.booleans(logEvent, shootOneButton, shootAllButton, tacoBell, autoBallPickup, intakeButton, ejectButton,
        pointBlankButton, povLeftShot, povRightShot,
        bottomBasket, autoLockHoop, increaseDistance, decreaseDistance);
    Logger.doubles(navx.getYaw(), navx.getPitch(), navx.getRoll());
    climbStateMachine.LogData();
    intake.LogData();
    queueStateMachine.LogData();
    shooterStateMachine.LogData();
    tunnelStateMachine.LogData();
    aimController.LogData();
    navXRoll.LogData();
  }

  public void configVision(){
    if (useVision){
      ledMode.setNumber(3);
      camMode.setNumber(0);

    }else{
      ledMode.setNumber(1);
      camMode.setNumber(1);
    }
  }

  public static void takeSnapshot(){
    if(snapshotCounterLeft > 0){
      snapshot.setNumber(1);
      snapshotCounterLeft--;
    }
  }

}