// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.autocorrectTargetAngle;
import static frc.robot.Robot.currentAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;

enum TBautoStates {
    Start, DriveBack, Buffer, Shoot, Done, CenterTurn, CenterBack, RightPullForward, RightTurn, RightPullBack
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
    double desiredCenterTurn = 60;//30
    double desiredCenterBack = -7;
    double desiredRightTurn = 90;
    double desiredRightForward = 2;
    double desiredRightBack = -10;

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
            autoMoveBack = -7.5;
        } else {
            autoMoveBack = -3.5;
        }
        //autoMoveBack = -5;
        update();
        SmartDashboard.putString("Auto State", currentState.toString());
        SmartDashboard.putString("DialPosition", position.toString());

    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TBautoStates.DriveBack;
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <= autoMoveBack) {
                    nextState = TBautoStates.Buffer;
                }
                break;
            case Buffer:
                if (secondsInState >= 2) {
                    nextState = TBautoStates.Shoot;
                }
                break;
            case Shoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll) {
                    if (autoDial == 0 || position == Position.Left){
                        nextState = TBautoStates.Done;
                    }
                    else {
                        if (position == Position.Right){
                            nextState = TBautoStates.RightPullForward;
                        }
                    else if (position == Position.Center){
                        nextState = TBautoStates.CenterTurn;
                        
                    }
                    }
                    

                }

                break;
            case Done:
                break;

            case CenterTurn:
                if (Robot.currentAngle > StartAngle + desiredCenterTurn){
                    nextState = TBautoStates.CenterBack;
                }
                break;
            case CenterBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + desiredCenterBack){
                    nextState = TBautoStates.Done;
                }
                break;
            case RightPullForward:
                if (Robot.driveTrain.getDistanceFeet() > StartDistance + desiredRightForward){
                    nextState = TBautoStates.RightTurn;
                }
                break;
            
            case RightTurn:
                if (Robot.currentAngle > StartAngle + desiredRightTurn){
                    nextState = TBautoStates.RightPullBack;

                }
                break;
            case RightPullBack:
                if (Robot.driveTrain.getDistanceFeet() < StartDistance + desiredRightBack){
                    nextState = TBautoStates.Done;
                }


        }
    }

    @Override
    public void doTransition() {
      
        if (isTransitionFrom(TBautoStates.DriveBack)) {
            Robot.driveTrain.Drive(0, 0);

        }
        if (isTransitionFrom(TBautoStates.RightPullBack)){
            Robot.driveTrain.Drive(0, 0);
        } 

        if (isTransitionFrom(TBautoStates.CenterTurn)){
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightPullForward)){
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionFrom(TBautoStates.RightTurn)){
            Robot.driveTrain.Drive(0, 0);
        }
      
        if (isTransitionFrom(TBautoStates.CenterBack)){
            Robot.driveTrain.Drive(0, 0);
        }
        if (isTransitionTo(TBautoStates.RightPullForward)){
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(2, 0);
        }
    
        if (isTransitionTo(TBautoStates.RightTurn)){
            StartAngle = Robot.currentAngle;
            Robot.driveTrain.Drive(0, 90);
        }
      
        if (isTransitionTo(TBautoStates.RightPullBack)){
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-2, 0);
        }
     
        if (isTransitionTo(TBautoStates.DriveBack)) {
            Robot.driveTrain.resetEncoders();
            Robot.intake.intake();
            Robot.driveTrain.Drive(-2, 0);
        }
        if (isTransitionTo(TBautoStates.Shoot)) {
            // Robot.driveTrain.Drive(0, 0);
            Robot.aimController.setAimMode(AimMode.robotShoot);

            Robot.shooterStateMachine.shootAll();

            // Robot.intake.idle();
            Robot.intake.intake();
        }
        if (isTransitionTo(TBautoStates.CenterTurn)){
            StartAngle = Robot.currentAngle;
            Robot.driveTrain.Drive(0, 45);
        }
        if (isTransitionTo(TBautoStates.CenterBack)){
            StartDistance = Robot.driveTrain.getDistanceFeet();
            Robot.driveTrain.Drive(-2, 0);
        }

    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case Shoot:
                DriveParameters dp = Robot.aimController.calculate(0, autocorrectTargetAngle, currentAngle, 0, false,false, 0);
                double turn = dp.turn;
                Robot.driveTrain.Drive(0, turn);
                Robot.shooterStateMachine.homed = true;
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
