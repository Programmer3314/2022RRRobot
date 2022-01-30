// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utility.MMStateMachine;

enum ShooterStates {
    Start, Home, Idle, Preparing, Shooting
};

/** Add your docs here. */
public class ShooterStateMachine extends MMStateMachine<ShooterStates> {

    boolean homed;
    TargetPoint target;
    boolean ballGone;

    public ShooterStateMachine() {
        super(ShooterStates.Start);
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = ShooterStates.Home;
                break;
            case Home:
                if (homed) {
                    nextState = ShooterStates.Idle;
                }
                break;
            case Idle:
                if (target.active) {
                    nextState = ShooterStates.Preparing;
                }
                break;
            case Preparing:
                if (target.active && Robot.queueStateMachine.isFull()) {
                    nextState = ShooterStates.Shooting;
                } else if (!target.active) {
                    nextState = ShooterStates.Idle;
                }
                break;
            case Shooting:
                if (ballGone) {
                    nextState = ShooterStates.Idle;
                }
                break;
        }
    }

    @Override
    public void doTransition() {
    if(isTransitionTo(ShooterStates.Shooting)){
        Robot.queueStateMachine.shooterBallRequest();
    }
    }

    @Override
    public void doCurrentState() {
        switch(currentState){
            case Home:
            homed = true;
            break;
            case Preparing:
            //get motors to correct velocity and position based on shooting solution
            break;
        }

    }

    public void setShootingSolution(TargetPoint target) {
        this.target = target;
    }
}
