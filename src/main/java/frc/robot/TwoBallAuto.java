// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.autocorrectTargetAngle;
import static frc.robot.Robot.currentAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;

enum TBautoStates {
    Start, DriveBack, Buffer, Shoot, Done
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

    public TwoBallAuto() {
        super(TBautoStates.Start);
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        if(Robot.autoChangeDistance){
            autoMoveBack = -7.5;
        }else{
            autoMoveBack = -3.5;
        }
        update();
        SmartDashboard.putString("Auto State", currentState.toString());
        
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TBautoStates.DriveBack;
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <=autoMoveBack) {
                    nextState = TBautoStates.Buffer;
                }
                break;
            case Buffer:
                if (secondsInState >= 2){
                    nextState = TBautoStates.Shoot;
                } 
                break;
            case Shoot:
                if (!Robot.shooterStateMachine.shootOne && !Robot.shooterStateMachine.shootAll){
                    nextState = TBautoStates.Done;
                }
                break;
            case Done:
                break;
            

        }
    }

    @Override
    public void doTransition() {
        if (isTransitionTo(TBautoStates.DriveBack)){
            Robot.driveTrain.resetEncoders();
            Robot.intake.intake();
            Robot.driveTrain.Drive(-2, 0);
        }

        if (isTransitionFrom(TBautoStates.DriveBack)){
            Robot.driveTrain.Drive(0, 0);

        }

        if (isTransitionTo(TBautoStates.Shoot)){
            //Robot.driveTrain.Drive(0, 0);
            Robot.aimController.setAimMode(AimMode.robotShoot);

            Robot.shooterStateMachine.shootAll();

            Robot.intake.idle();
        }
    }

    @Override
    public void doCurrentState() {     
        switch (currentState){
            case Shoot:
                double turn = Robot.aimController.calculate(0, autocorrectTargetAngle, currentAngle, 0);
                Robot.driveTrain.Drive(0, turn);
                Robot.shooterStateMachine.homed = true;
                break;
            case Done:
            Robot.driveTrain.Drive(0, 0);
            break;
        }
    }
    public void resetState(){
        currentState = TBautoStates.Start;
    }
}
