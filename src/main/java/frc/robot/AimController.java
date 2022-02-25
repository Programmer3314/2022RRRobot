// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;

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
    turretShoot, robotShoot, turretMixedShoot, ballChase, driver, autonomous
}

public class AimController {
    PIDController robotAim;
    AimMode aimMode;
    MMMotorGroup turret;
    DigitalInput turretLowLimitSwitch;
    boolean turretHomed;
    double desiredTurretPosition;
    boolean searching;
    double searchPower;
    double maxRobotTurn;
    double minRobotTurn;
    double toleranceRobotTurn;

    public AimController() {
        //turret = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterTurret));
        // this.turret = turret;
        searchPower = -.15;
        robotAim = new PIDController(kPRobotTargetTurn, kIRobotTargetTurn, kDRobotTargetTurn);
        robotAim.setTolerance(kRobotAimTolerance);
        turretHomed = true;
        setAimMode(AimMode.driver);
        maxRobotTurn = Constants.kMaxRobotTurn;
        minRobotTurn = Constants.kMinRobotTurn;
        toleranceRobotTurn = Constants.kToleranceRobotTurn;
    }

    public void setAimMode(AimMode aimMode) {
        this.aimMode = aimMode;
    }

    public double calculate(double DriverTurn, double targetAngle, double currentAngle, double ballAngle) {
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
                turn = DriverTurn;
                break;
            case robotShoot: {
                //double currentError = targetAngle - currentAngle;
                //turn = kPRobotTargetTurn * currentError;
                turn = robotAim.calculate(currentAngle, targetAngle);
                if (turn > maxRobotTurn){
                    turn = maxRobotTurn;
                }
                if (turn < -maxRobotTurn){
                    turn = -maxRobotTurn;
                }
                if (Math.abs(turn) < toleranceRobotTurn){
                    turn = 0;
                } else{
                    double ratio = (Math.abs(turn) - toleranceRobotTurn)/(maxRobotTurn - toleranceRobotTurn);
                        turn = Math.signum(turn)*ratio * (maxRobotTurn - minRobotTurn) + minRobotTurn;
                    
                    
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
            default:
                turn = DriverTurn;
                break;

        }
        // if (!turretHomed) {
        //     // turret.setPower(-.15);
        // }
        // if (turretLowLimitSwitch.get()) {
        //     turretHomed = true;
        //     // turret.setEncoderRevolutions(Constants.kTurretDegreesHome/Constants.kTurretDegreesPerRev);
        // }

        // if (turretHomed) {
        //     if (searching) {
        //         if (turret.getRevolutions() < Constants.kTurretLowLimit / Constants.kTurretDegreesPerRev) {
        //             searchPower = .15;
        //         }
        //         if (turret.getRevolutions() > Constants.kTurretHighLimit / Constants.kTurretDegreesPerRev) {
        //             searchPower = -.15;
        //         }

        //         turret.setPower(searchPower);
        //         if (Robot.confidenceCounter > 0) {
        //             searching = false;
        //             turret.setPower(0);
        //         }

        //     } else {
        //         desiredTurretPosition = (targetAngle - currentAngle) / Constants.kTurretDegreesPerRev;
        //         if (desiredTurretPosition >= Constants.kTurretHighLimit) {
        //             // turretPosition = Constants.kTurretHighLimit;
        //         }
        //         if (desiredTurretPosition <= Constants.kTurretLowLimit) {
        //             // turretPosition = Constants.kTurretHighLimit;
        //         }
        //         // turret.setPosition(turretPosition);
        //     }
        // }
        SmartDashboard.putNumber("Turn", turn);
        return turn;
    }

    public boolean isHomed() {
        return turretHomed;
    }

    public double turretError() {
        return 0;
        //return desiredTurretPosition - turret.getRevolutions() * Constants.kTurretDegreesPerRev;
    }
    public void searchRequest() {
        searching = true;
    }
    
}
