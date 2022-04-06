// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.hubTargetAngle;

import java.nio.channels.NetworkChannel;

import javax.swing.JSpinner.NumberEditor;

import static frc.robot.Robot.currentRobotAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;

enum TBautoStates {
    Start, DriveBack, Buffer, Shoot, Done, CenterTurn, CenterBack, GetBall, RightGetBall, Delay, RightPullForward,
    RightTurn, DrivePastBall,
    RightPullBack, moveToTarget, secondShoot, RightTerminalTurn, LeftMoveForward
};

/**
 * Steps:
 * Drive back certain distance+Stop(3ft in tests)
 * Turn robot away from target(10Degrees), then Autoturn back to the goal
 */
public class TwoBallAuto extends MMAutonomous<TBautoStates> {
    double autoMoveBack;
    double goalAngle;
    double autoTargetRunCounter;
    Position position;
    int autoDial;
    double StartAngle;
    double StartDistance;
    double desiredCenterTurn = 60;// ACTUALVALUE60
    double desiredCenterBack = -7;// ACTUALVALUE-7
    double desiredRightTurn;
    double desiredRightPastBall;
    double desiredCenterForward = 5;
    double desiredRightForward = 2;
    double desiredRightBack;// ACTUALVALUE-10
    double counter;
    double desiredLeftForward = 1.5;

    public TwoBallAuto(Position position, int autoDial) {
        super(TBautoStates.Start);
        this.position = position;
        this.autoDial = autoDial;
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        if (position == Position.Center) {
            autoMoveBack = -7.5;// ACTUALVALUE-7.5
        } else {
            autoMoveBack = -3.5;
        }
        // autoMoveBack = -5;
        update();
        SmartDashboard.putString("Auto State", currentState.toString());
        SmartDashboard.putString("DialPosition", position.toString());
        SmartDashboard.putNumber("GetBallCounter", counter);
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TBautoStates.DriveBack;
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <= autoMoveBack) {
                    nextState = TBautoStates.LeftMoveForward;
                }
                break;
            case Buffer:
                if (secondsInState >= 2 || !Robot.useVision) {
                    nextState = TBautoStates.Shoot;
                }
                break;
            case Shoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll) {
                    if (autoDial == 0||position == Position.Left){
                        nextState = TBautoStates.Done;
                    }else {
                        if (position == Position.Right) {
                            nextState = TBautoStates.RightPullForward;
                        } else if (position == Position.Center) {
                            nextState = TBautoStates.CenterTurn;

                        }
                    }
                }
                break;
            case LeftMoveForward:
            if(Robot.driveTrain.getDistanceFeet() > StartDistance + desiredLeftForward){
                nextState = TBautoStates.Buffer;
            }
            break;
            case Done:
                break;
            case CenterTurn:
                if (Robot.currentRobotAngle > StartAngle + desiredCenterTurn) {
                    nextState = TBautoStates.CenterBack;
                }
                break;
            case CenterBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + desiredCenterBack) {
                    nextState = TBautoStates.Delay;
                }
                break;

            case Delay:
                if (secondsInState >= 2 || Robot.targetBallConfidence) {
                    if (Robot.targetBallConfidence) {
                        nextState = TBautoStates.GetBall;
                    } else {
                        nextState = TBautoStates.Done;
                    }
                }
                break;
            case GetBall:
                if (Robot.tunnelStateMachine.currentState == TunnelStates.BallIsLive) {
                    nextState = TBautoStates.moveToTarget;
                }
                break;
            case RightGetBall:
                    if(Robot.tunnelStateMachine.currentState == TunnelStates.BallIsLive){
                    nextState = TBautoStates.moveToTarget;
                    }
                    else if(Robot.driveTrain.getDistanceFeet() < StartDistance + desiredRightBack) {
                        nextState = TBautoStates.Done;
                    }
                break;
            case moveToTarget:
                if (Robot.driveTrain.getDistanceFeet() > StartDistance + desiredCenterForward) {
                    nextState = TBautoStates.secondShoot;
                }
                break;
            case secondShoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll) {
                    nextState = TBautoStates.Done;
                }
                break;
            case RightPullForward:
                if (Robot.driveTrain.getDistanceFeet() > StartDistance + desiredRightForward) {
                    if (autoDial == 3) {
                        nextState = TBautoStates.RightTerminalTurn;
                        desiredRightTurn = 75;
                    } else {
                        nextState = TBautoStates.RightTurn;
                        desiredRightTurn = 90;
                    }
                }
                break;

            case RightTurn:
                if (Robot.currentRobotAngle > StartAngle + desiredRightTurn) {
                    // nextState = TBautoStates.RightPullBack;
                    nextState = TBautoStates.RightGetBall;
                    desiredRightBack = -10;
                }
                break;
            case RightTerminalTurn:
                if (Robot.currentRobotAngle > StartAngle + desiredRightTurn) {
                    // nextState = TBautoStates.RightPullBack;
                    nextState = TBautoStates.DrivePastBall;
                    desiredRightPastBall = -14;
                }
                break;
            case DrivePastBall:
            if(Robot.driveTrain.getDistanceFeet() < StartDistance + desiredRightPastBall){
                nextState = TBautoStates.RightGetBall;
            }
            break;
            case RightPullBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + desiredRightBack) {
                    nextState = TBautoStates.secondShoot;// Done
                }
                break;
        }
    }

    @Override
    public void doTransition() {

        if (isTransitionFrom(TBautoStates.DriveBack)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if(isTransitionFrom(TBautoStates.DrivePastBall)){
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightPullBack)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.CenterTurn)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightPullForward)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightTurn)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightTerminalTurn)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.CenterBack, TBautoStates.LeftMoveForward)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.moveToTarget)) {
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionTo(TBautoStates.RightPullForward)) {
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(2, 0);
        }
        if(isTransitionTo(TBautoStates.DrivePastBall)){
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-3.5, 0);
        }
        if(isTransitionTo(TBautoStates.LeftMoveForward)){
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(2, 0);
        }
        if (isTransitionTo(TBautoStates.RightTurn)) {
            StartAngle = Robot.currentRobotAngle;
            Robot.driveTrain.Drive(0, 90);
        }
        if (isTransitionTo(TBautoStates.RightTerminalTurn)) {
            StartAngle = Robot.currentRobotAngle;
            Robot.driveTrain.Drive(0,75);
        }
        if (isTransitionTo(TBautoStates.RightPullBack)) {
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-3, 0);// -2
        }
        if (isTransitionTo(TBautoStates.DriveBack)) {
            Robot.driveTrain.resetEncoders();
            Robot.intake.intake();
            Robot.driveTrain.Drive(-2, 0);
        }
        if (isTransitionTo(TBautoStates.Shoot)) {
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
        if (isTransitionTo(TBautoStates.CenterTurn)) {
            StartAngle = Robot.currentRobotAngle;
            Robot.driveTrain.Drive(0, 45);
        }
        if (isTransitionTo(TBautoStates.CenterBack)) {
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-2, 0);
        }
        if (isTransitionTo(TBautoStates.Delay)) {
            Robot.aimController.setAimMode(AimMode.ballChase);
        }
        if (isTransitionTo(TBautoStates.GetBall, TBautoStates.RightGetBall)) {
            // Robot.driveTrain.Drive(-1, 0);
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.aimController.setAimMode(AimMode.ballChase);
            Robot.intake.intake();
        }
        if (isTransitionTo(TBautoStates.moveToTarget)) {
            StartDistance = Robot.driveTrain.getDistanceFeet();
            // Robot.driveTrain.Drive(1, 0);
            Robot.aimController.setAimMode(AimMode.robotShoot);
        }
        if (isTransitionTo(TBautoStates.secondShoot)) {
            Robot.aimController.setAimMode(AimMode.robotShoot);
            Robot.shooterStateMachine.shootAll();
        }
    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case GetBall:
            case RightGetBall: {
                Robot.aimController.setAimMode(AimMode.ballChase);
                DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle,
                        Robot.ballChaseAngle, false,
                        false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(-3, turn);
                counter++;
                SmartDashboard.putNumber("GetballTurn", turn);
            }
                break;

            case moveToTarget: {
                DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle, 0, false,
                        false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(2, turn);
            }
                break;
            case Shoot: {
                DriveParameters dp = Robot.aimController.calculate(0, hubTargetAngle, currentRobotAngle, 0, false,
                        false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(0, turn);
                Robot.shooterStateMachine.homed = true;
            }
                break;
            case Done:
                Robot.driveTrain.Drive(0, 0);
                break;
            default:
                break;
        }
    }

    public void resetState() {
        currentState = TBautoStates.Start;
    }

    public void LogHeader() {
        Logger.Header("AutoMove,"
                + "AutoState,");
    }

    public void LogData() {
        Logger.doubles(autoMoveBack);
        Logger.singleEnum(currentState);
    }
}
