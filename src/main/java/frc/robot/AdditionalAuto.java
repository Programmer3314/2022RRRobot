// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.currentRobotAngle;
import static frc.robot.Robot.hubTargetAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;

enum additionalAutoStates {
    Start, DriveBack, Buffer, FirstShoot, moveToTarget, SecondShoot, GetBallTwo, Done, MoveBack, RightPullForward,
    RightTurn
};

/** Add your docs here. */
public class AdditionalAuto extends MMAutonomous<additionalAutoStates> {
    Position position;
    int autoDial;
    double StartAngle;
    double StartDistance;
    double desiredCenterForward = 15;
    double desiredMoveBackDistance = -17;
    double desiredPullForward = 1;
    double desiredAngleTurn = 75;
    double StartYaw;
    double autoMoveBack = -3.5;

    public AdditionalAuto(Position position, int autoDial) {
        super(additionalAutoStates.Start);
        this.position = position;
        this.autoDial = autoDial;
    }

    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = additionalAutoStates.DriveBack;
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <= autoMoveBack) {
                    nextState = additionalAutoStates.Buffer;
                }
                break;
            case Buffer:
                if (secondsInState >= 2 || !Robot.useVision) {
                    nextState = additionalAutoStates.RightPullForward;
                }
                break;
            case FirstShoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll) {
                    nextState = additionalAutoStates.RightPullForward;
                }
                break;
            case RightPullForward:
                if (Robot.driveTrain.getDistanceFeet() > StartDistance + desiredPullForward) {
                    nextState = additionalAutoStates.RightTurn;
                }
                break;
            case RightTurn:
                if (Robot.navx.getYaw() > desiredAngleTurn + StartYaw) {
                    nextState = additionalAutoStates.MoveBack;
                }
                break;
            case MoveBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + desiredMoveBackDistance) {
                    nextState = additionalAutoStates.GetBallTwo;
                }
                break;
            case moveToTarget:
                if (Robot.driveTrain.getDistanceFeet() > StartDistance + desiredCenterForward) {
                    nextState = additionalAutoStates.SecondShoot;
                }
                break;
            case SecondShoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll) {
                    nextState = additionalAutoStates.Done;
                }
                break;
            case GetBallTwo:
                if (Robot.tunnelStateMachine.currentState == TunnelStates.BallIsLive) {
                    nextState = additionalAutoStates.moveToTarget;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {
        update();
        SmartDashboard.putString("Auto State", currentState.toString());
    }

    @Override
    public void LogHeader() {
        // TODO Auto-generated method stub

    }

    @Override
    public void LogData() {
        // TODO Auto-generated method stub

    }

    @Override
    public void doTransition() {
        if (isTransitionFrom(additionalAutoStates.moveToTarget)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(additionalAutoStates.MoveBack)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(additionalAutoStates.RightPullForward)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(additionalAutoStates.RightTurn)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(additionalAutoStates.DriveBack)) {
            Robot.driveTrain.Drive(0, 0);
        }

        if (isTransitionTo(additionalAutoStates.SecondShoot, additionalAutoStates.FirstShoot)) {
            // Robot.driveTrain.Drive(0, 0);
            if (!Robot.useVision) {
                Robot.aimController.setAimMode(AimMode.driver);
            } else {
                Robot.aimController.setAimMode(AimMode.robotShoot);
                Robot.takeSnapshot();
            }
            Robot.shooterStateMachine.shootAll();
            // Robot.intake.idle();
            Robot.intake.intake();
        }
        if (isTransitionTo(additionalAutoStates.moveToTarget)) {
            StartDistance = Robot.driveTrain.getDistanceFeet();
            // Robot.driveTrain.Drive(1, 0);
            Robot.aimController.setAimMode(AimMode.robotShoot);
        }
        if (isTransitionTo(additionalAutoStates.GetBallTwo)) {
            // Robot.driveTrain.Drive(-1, 0);
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.aimController.setAimMode(AimMode.ballChase);
            Robot.intake.intake();
        }
        if (isTransitionTo(additionalAutoStates.MoveBack)) {
            Robot.intake.intake();
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-9, 0);
        }
        if (isTransitionTo(additionalAutoStates.RightPullForward)) {
            StartYaw = Robot.navx.getYaw();
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(1, 0);
        }
        if (isTransitionTo(additionalAutoStates.RightTurn)) {
            StartYaw = Robot.navx.getYaw();
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(0, 100);
        }
        if (isTransitionTo(additionalAutoStates.DriveBack)) {
            Robot.driveTrain.resetEncoders();
            Robot.intake.intake();
            Robot.driveTrain.Drive(-2, 0);
            Robot.shooterStateMachine.shootAll();
        }
    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case MoveBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + (desiredMoveBackDistance - 2)) {
                    Robot.driveTrain.Drive(-1, -15);
                }
                break;
            case GetBallTwo: {
                Robot.aimController.setAimMode(AimMode.ballChase);
                DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle,
                        Robot.ballChaseAngle, false,
                        false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(-1.5, turn);
                SmartDashboard.putNumber("GetballTurn", turn);
            }
                break;
            case moveToTarget: {
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + 5) {
                    Robot.driveTrain.Drive(3, -30);
                } else {
                    DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle, 0, false,
                            false, 0);
                    double turn = dp.turn;
                    Robot.driveTrain.Drive(4, turn);
                }
            }
                break;
            case FirstShoot:
            case SecondShoot: {
                DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle, 0, false,
                        false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(0, turn);
                Robot.shooterStateMachine.homed = true;
            }
                break;
            default:
                break;
        }
    }
}
