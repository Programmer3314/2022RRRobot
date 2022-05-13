// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Aim the turret based on the vision software
 * Aim the body of the robot without turret
 * move body and turret
 * Aim the body of the robot towards the ball
 * Set Autonomous shooting and ball location
 * input
 * joystick input
 * Robot NAVx angle
 * turret angle(encoder)
 * ball angle
 * target angle
 * 
 * outputs
 * turn the drivetrain
 * turn the turret
 * set turn value to the turret
 * put turret
 */

enum AimMode {
    turretShoot, robotShoot, turretMixedShoot, ballChase, driver, autonomous, lockBar
}

public class AimController {
    PIDController robotAim;
    AimMode aimMode;
   // MMMotorGroup turret;
    DigitalInput turretLowLimitSwitch;
    boolean turretHomed;
    double desiredTurretPosition;
    boolean searching;
    double searchPower;
    double maxRobotTurn;
    double maxAutoTurn;
    double minRobotTurn;
    double toleranceRobotTurn;
    boolean turretBool;

    public AimController() {
        // turret = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterTurret)
        //         .setPIDFParameters(Constants.kTurretKP, Constants.kTurretKI, Constants.kTurretKD, Constants.kTurretKFF)
        //         .setInverted(InvertType.None)
        //         .setBrakeMode(false)
        //         .setPeakOutput(0.20, -0.20)); // 0.25;-0.25
        searchPower = -.15;
        robotAim = new PIDController(Constants.kPRobotTargetTurn, Constants.kIRobotTargetTurn, Constants.kDRobotTargetTurn);
        robotAim.setTolerance(Constants.kRobotAimTolerance);
        turretHomed = false;
        setAimMode(AimMode.driver);
        maxRobotTurn = Constants.kMaxRobotTurn;
        maxAutoTurn = Constants.kMaxAutoTurn;
        minRobotTurn = Constants.kMinRobotTurn;
        toleranceRobotTurn = Constants.kToleranceRobotTurn;
        turretLowLimitSwitch = new DigitalInput(Constants.kDIOTurretLimitSwitch);
        turretBool = false;
    }

    public void setAimMode(AimMode aimMode) {
        this.aimMode = aimMode;
    }

    public DriveParameters calculate(double DriverTurn, double targetAngle, double currentAngle, double ballAngle, 
    boolean contactLeft, boolean contactRight, double drive) {
        targetAngle-=3;
        double turn = 0;
        switch (aimMode) {
            case autonomous:
                // skip for now
                turn = DriverTurn;
                break;
            case ballChase: {
                turn = Constants.kPRobotBallTurn * ballAngle;
            }
                break;
            case driver:
                double scale = ((Constants.kMaxSpeed-(drive*Constants.kTurnReduction))/Constants.kMaxSpeed);
                turn = DriverTurn*scale;
                break;
            case robotShoot: {
                // double currentError = targetAngle - currentAngle;
                // turn = kPRobotTargetTurn * currentError;
                turn = robotAim.calculate(currentAngle, targetAngle);
                if (turn > maxRobotTurn) {
                    turn = maxRobotTurn;
                }
                if (turn < -maxRobotTurn) {
                    turn = -maxRobotTurn;
                }
                if (Math.abs(turn) < toleranceRobotTurn) {
                    turn = 0;
                } else {
                    if (Robot.confidenceCounter > 0) {
                        double ratio = (Math.abs(turn) - toleranceRobotTurn) / (maxRobotTurn - toleranceRobotTurn);
                        turn = Math.signum(turn) * (ratio * (maxRobotTurn - minRobotTurn) + minRobotTurn);
                    }

                    /**
                     * turn = 0.8
                     * tolerance = 0.2
                     * max = 1
                     * min = 0.2
                     * 0.6/0.8 = 75%
                     * 0.6 +0.2 =0.8
                     */
                }

            }
                break;
            case turretShoot:
                turn = DriverTurn;
                break;
            case lockBar:
            if(!contactLeft && contactRight){
                turn = 50;
            }
            else if (!contactRight && contactLeft){
                turn = -50;
            }
            else if(!contactLeft && !contactRight){
                drive = .75;
            }
            else if(contactLeft && contactRight){
                drive = 0;
            }
            break;
            default:
                turn = DriverTurn;
                break;

        }
        // turretBool = !turretLowLimitSwitch.get();
        // if (!turretHomed) {
        //     turret.setPower(-.12);
        // }
        // if (turretBool) {
        //     turretHomed = true;
        //     turret.setEncoderRevolutions(Constants.kTurretDegreesHome / Constants.kTurretDegreesPerRev);
        // }

        // if (turretHomed) {
        //     if (searching) {
        //         if (turret.getRevolutions() < Constants.kTurretLowLimit / Constants.kTurretDegreesPerRev) {
        //             searchPower = .1;
        //         }
        //         if (turret.getRevolutions() > Constants.kTurretHighLimit / Constants.kTurretDegreesPerRev) {
        //             searchPower = -.1;
        //         }

        //         turret.setPower(searchPower);
        //         if (Robot.confidenceCounter > 0) {
        //             searching = false;
        //             turret.setPower(0);
        //         }

        //     } else {
        //         if (Robot.confidenceCounter > 0) {
        //             desiredTurretPosition = (targetAngle - Robot.currentShooterAngle) / Constants.kTurretDegreesPerRev;
        //             if (desiredTurretPosition >= Constants.kTurretHighLimit) {
        //                 desiredTurretPosition = Constants.kTurretHighLimit;
        //             }
        //             if (desiredTurretPosition <= Constants.kTurretLowLimit) {
        //                 desiredTurretPosition = Constants.kTurretLowLimit;
        //             }
        //             // turret.setPosition(desiredTurretPosition);
        //             turret.setPosition(desiredTurretPosition);
        //             SmartDashboard.putString("Setting Position: ", "Yes");
        //         }
        //     }
        // }
        // SmartDashboard.putNumber("TURTurret Revolutions", turret.getRevolutions());
        // SmartDashboard.putNumber("TURTurret Degrees", turret.getRevolutions() * Constants.kTurretDegreesPerRev);
        SmartDashboard.putNumber("TURTurn", turn);
        SmartDashboard.putString("TURaimMode", aimMode.toString());
        SmartDashboard.putBoolean("TURturret  homed", turretHomed);
        SmartDashboard.putBoolean("TURturret limit", turretBool);
        SmartDashboard.putNumber("TURDesired Turret Position: ", desiredTurretPosition);
        SmartDashboard.putNumber("TURTarget Angle: ", targetAngle);
        SmartDashboard.putNumber("TURcurrent Angle: ", currentAngle);
        SmartDashboard.putNumber("TURshooter Angle:", Robot.currentShooterAngle);
        SmartDashboard.putBoolean("Searching??", searching);

        return new DriveParameters(drive, turn);
    }

    public boolean isHomed() {
        return turretHomed;
    }

    public double turretError() {
        return 0;
        // return desiredTurretPosition - turret.getRevolutions() * Constants.kTurretDegreesPerRev;
    }

    public void searchRequest() {
        searching = true;
    }

    public void resetTurret() {
        turretHomed = false;
    }
    public void LogHeader(){
        Logger.Header("AimMode,");
    }
    public void LogData(){
        Logger.singleEnum(aimMode);
    }

}
