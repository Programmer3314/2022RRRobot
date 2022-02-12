// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;
import frc.robot.AimController;

enum TBautoStates {
    Start, DriveBack, TurnAway, AutoTarget, Done
};

/**
 * Steps:
 * Drive back certain distance+Stop(3ft in tests)
 * Turn robot away from target(10Degrees), then Autoturn back to the goal
 */
public class TwoBallAuto extends MMAutonomous<TBautoStates> {
    double goalAngle;
    double autoTargetRunCounter;

    public TwoBallAuto() {
        super(TBautoStates.Start);
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        update();
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case AutoTarget:
                if (Robot.confidenceCounter == 0){
                    nextState = TBautoStates.Done;
                }
                if (Math.abs(currentAngle - autocorrectTargetAngle) < 2) {
                    autoTargetRunCounter += 1;
                    if (autoTargetRunCounter >= 50) {
                        nextState = TBautoStates.Done;
                    }
                } else {
                    autoTargetRunCounter = 0;
                }
                break;
            case Done:
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <= -3.0) {
                    nextState = TBautoStates.TurnAway;
                }
                break;
            case Start:
                nextState = TBautoStates.DriveBack;
                break;
            case TurnAway:
                if (currentAngle <= goalAngle) {
                    nextState = TBautoStates.AutoTarget;
                }
                break;
        }
    }

    @Override
    public void doTransition() {
        if (nextState == TBautoStates.AutoTarget) {
            lightRing.set(true);
        }
        if (nextState == TBautoStates.DriveBack) {
            Robot.driveTrain.resetEncoders();
        }
        if (nextState == TBautoStates.TurnAway) {
            goalAngle = currentAngle - 10;
        }
    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case AutoTarget:
                if (confidenceCounter > 0) {
                    Robot.aimController.setAimMode(AimMode.robotShoot);
                    double turn = Robot.aimController.calculate(0, autocorrectTargetAngle, currentAngle, 0);
            

                   // double currentError = autocorrectTargetAngle - currentAngle;
                    // double currentError=xAngle- currentAngle;
                   // double requestedTurn = p * currentError;
                    // SmartDashboard.putNumber("Auto Angle Correct", requestedTurn);

                    // SmartDashboard.putNumber("ConfidenceCounter", confidenceCounter);

                    // SmartDashboard.putNumber("encoder value", driveTrain.getRevolutions());
                    driveTrain.Drive(0, turn);
                }
                break;
            case Done:
                Robot.driveTrain.Drive(0, 0);
                break;
            case DriveBack:
                Robot.driveTrain.Drive(-1, 0);
                break;
            case Start:
                break;
            case TurnAway:
                Robot.driveTrain.Drive(0, -20);
                break;
        }
    }
}
