// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static double kNeoShooterCAMP = 0;
    public static double kNeoShooterCAMI = 0;
    public static double kNeoShooterCAMD = 0;
    public static double kNeoShooterCAMF = 0.0002;
    public static double kNeoShooterCAMIZ = 0;
    public static double kNeoShooterCAMMin = -1;
    public static double kNeoShooterCAMMax = +1;

    public static int kNeoShooterCAMStallLimit = 40;
    public static int kNeoShooterCAMFreeLimit = 40;

    public static double kNeoShooterWheelsP = 0;
    public static double kNeoShooterWheelsI = 0;
    public static double kNeoShooterWheelsD = 0;
    public static double kNeoShooterWheelsF = 0.0002;
    public static double kNeoShooterWheelsIZ = 0;
    public static double kNeoShooterWheelsMin = -1;
    public static double kNeoShooterWheelsMax = +1;

    public static int kNeoShooterWheelsStallLimit = 40;
    public static int kNeoShooterWheelsFreeLimit = 40;

    public static double kMaxSpeed = -10;
    public static double kMaxTurnRate = 180;

    public static double kCameraVerticalAngle = 41;
    public static double ktargetHeight=104/12.0;
    public static double kTargetCameraHeight=6/12.0;
    public static double kTargetingHeightDiff=ktargetHeight-kTargetCameraHeight;

    public static double kRevPerFoot = 4.67;
    public static double kChassiRadius = 1.04;
}
