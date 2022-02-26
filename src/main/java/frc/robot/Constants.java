// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root direory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

/** Add your docs here. */
public class Constants {
    public static double kNeoDriveTrainP = 0.000005;
    public static double kNeoDriveTrainI = 0;
    public static double kNeoDriveTrainD = 0;
    public static double kNeoDriveTrainF = 0.0002;
    public static double kNeoDriveTrainIZ = 0;
    public static double kNeoDriveTrainMin = -1;
    public static double kNeoDriveTrainMax = +1;

    public static int kNeoDriveTrainStallLimit = 40;
    public static int kNeoDriveTrainFreeLimit = 40;

    public static final double kfalconDrivetrainKI = 0.000000;
    public static final double kfalconDrivetrainKD = 0.00025 ; 
    public static final double kfalconDrivetrainKP = 0.002500;//5e-5;
    public static final double kfalconDrivetrainKIz = 0; 
    public static final double kfalconDrivetrainKFF = 0.050000; 
    public static final double kfalconDrivetrainKMaxOutput =  0.869990; 
    public static final double kfalconDrivetrainKMinOutput = -0.869990;
    public static final double kfalconTrackingMaxCorrection = 0.2; // = 0.1;
    public static final double kfalconTrackingMinCorrection = 0.04;

    public static double kNeoShooterCAMP = 0;
    public static double kNeoShooterCAMI = 0;
    public static double kNeoShooterCAMD = 0;
    public static double kNeoShooterCAMF = 0.0002;
    public static double kNeoShooterCAMIZ = 0;
    public static double kNeoShooterCAMMin = -1;
    public static double kNeoShooterCAMMax = +1;

    public static int kNeoShooterCAMStallLimit = 40;
    public static int kNeoShooterCAMFreeLimit = 40;

    public static double kFXShooterWheelsP = 0.1;
    public static double kFXShooterWheelsI = 0;
    public static double kFXShooterWheelsD = 1;
    public static double kFXShooterWheelsF = 0.0495;
    public static double kFXShooterWheelsIZ = 0;
    public static double kFXShooterWheelsMin = -1;
    public static double kFXShooterWheelsMax = +1;

    public static double kFXCamP = 0.1;
    public static double kFXCamI = 0;
    public static double kFXCamD = 1;
    public static double kFXCamF = 0.0;
    public static double kFXCamIZ = 0;
    public static double kFXCamMin = -1;
    public static double kFXCamMax = +1;

    public static int kNeoShooterWheelsStallLimit = 40;
    public static int kNeoShooterWheelsFreeLimit = 40;

    public static int kOperatorAxisShootOne = 2;
    public static int kOperatorAxisShootAll = 3;

    public static double kMaxSpeed = -10;
    public static double kMaxTurnRate = 300;

    public static double krpmMargin = 100;
    public static double kangleMargin = 10;

    public static double kCameraVerticalAngle = 35.5;//33.5
    public static double ktargetHeight=104/12.0;
    public static double kTargetCameraHeight=37/12.0;
    public static double kTargetingHeightDiff=ktargetHeight-kTargetCameraHeight;

    public static double kRevPerFoot = 4.67;
    public static double kChassisRadius = 1.04;

    public static double kNewRevPerFoot = 8;
    public static double kNewChassisRadius = 1.04;

    public static int kCanMCDriveRight1 = 1;
    public static int kCanMCDriveRight2 = 2;
//    public static int kCanMCDriveRight3 = 3;

    public static int kCanMCDriveLeft1 = 4;
    public static int kCanMCDriveLeft2 = 5;
  //  public static int kCanMCDriveLeft3 = 6;

    public static int kCanMCTunnelBelt = 10;
    public static int kCanMCTunnelWheels = 11;

    public static int kCanMCQueueBelt = 12;

    //public static int kCanMCShooterTurret = 15;
    public static int kCanMCShooterFeed = 16;
    public static int kCanMCShooterShoot = 17;
    public static int kCanMCShooterCam = 18;

    public static int kCanMCIntake = 20;

  //  public static int kCanMCClimber1 = 30;
  //  public static int kCanMCClimber2 = 31;

    public static int kCanPowerDistributionBoard = 1;
   
    public static int kDIOQueueBreakBeam = 0;
    public static int kDIOTunnelBreakBeam = 1;

    public static int kDIOTurretLimitSwitch = 2;
    public static int kDIOCamLimitSwitch = 8;
    public static int kDIOShooterBallGone = 9;

    public static int kShooterCounter = 10;

    public static double kPRobotBallTurn = 3;

    public static double kPRobotTargetTurn = 5;
    public static double kIRobotTargetTurn = 0;
    public static double kDRobotTargetTurn = 0;
    public static double kRobotAimTolerance = 0;

    public static double kMaxRobotTurn = 300;
    public static double kMinRobotTurn = 22.5;
    public static double kToleranceRobotTurn = 0;

    public static double kPTurretTargetTurn =2;

    public static double kTurretDegreesPerRev = 10;
    public static double kTurretDegreesHome = -170;
    public static double kTurretLowLimit = -170;
    public static double kTurretHighLimit = 170;

    public static int kOperatorSearchButton=7; 
    public static int kOperatorPointBlankPOV = 0;
    public static int kOperatorRaiseHooks = 1;
    public static int kOperatorLowerHooksButton = 8;
    public static int kOperatorClimbButton = 2;
    public static int kOperatorAbortShot = 5;

    public static int kButtonBoxIncreaseDistance = 3;
    public static int kButtonBoxDecreaseDistance = 6;
    public static int kButtonBoxResetRobot = 7;
    public static int kButtonBoxTacobell= 8;
    public static int kButtonBoxAbortShot = 9;

    public static int kShooterLimeLight=4;

    public static int kDriverToggleBallLight=9;
    public static int kDriverAutoBallPickup=2;
    public static int kDriverIntake = 5;
    public static int kDriverEject = 6; 
    public static int kDriverAutoTurnToTarget = 1;

    public static int kPneumaticsHub = 0;
    public static int kEjectPneumaticsHub = 1;

    public static InvertType kLeftMGInverted = InvertType.InvertMotorOutput;//not inverted
    public static InvertType kRightMGInverted = InvertType.None;//not inverted

    public static int kJoystickDriver = 4;
    public static int kJoystickOperator = 5;

    public static int kSolenoidModule = 1;

    public static int kSolenoidIntakeForward = 0;
    public static int kSolenoidIntakeBackward =1;

    public static int kSolenoidClimberForward = 5;
    public static int kSolenoidClimberBackward = 6;

    public static int kTestButtonBoxDisableCompressor = 10;
    // public static int kTestButtonBoxDesiredBall = 2;
    // public static int kTestButtonBoxQueuePosition = 3;
    // public static int kTestButtonBoxAirBall = 4;
}