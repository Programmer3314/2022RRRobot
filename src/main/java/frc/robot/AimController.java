// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.Constants.*;

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
    AimMode aimMode;

    public AimController() {

    }

    public void setAimMode(AimMode aimMode) {
        this.aimMode = aimMode;
    }

    public double calculate(double DriverTurn, double targetAngle, double currentAngle, double ballAngle) {
        double turn = 0;
        switch (aimMode) {
            case autonomous:
            // skip for now
                break;
            case ballChase:
               {
                turn = Constants.kPRobotBallTurn * ballAngle;
               }
                break;
            case driver:
                turn = DriverTurn;
                break;
            case robotShoot:
               { 

                double currentError = targetAngle - currentAngle;

                turn = kPRobotTargetTurn * currentError;
               }

                break;
            case turretMixedShoot:
                break;
            case turretShoot:

                break;
            default:
                break;

        }
        return turn;
    }
}
