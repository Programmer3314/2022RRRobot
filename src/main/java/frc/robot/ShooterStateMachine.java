// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSRXMotorController;
import frc.robot.utility.MMStateMachine;

/**
 * Expected Hardware configuration (No Turret):
 * Break Beam sensor in ball path to detect when ball is ready/gone
 * Shoot motor (implement motor group anyway) 
 * Feed motor
 * CAM motor
 * CAM Home Limit Switch
 * Existing Motor Encoders
 * 
 */

/**
 * Look at 3dPaint for description of statemachine
 */

 // TODO Add motors and check for actually attaining the firing solution
 

enum ShooterStates {
    Start, Home, Idle, Preparing, Shooting
};

/** Add your docs here. */
public class ShooterStateMachine extends MMStateMachine<ShooterStates> {

    boolean homed;
    TargetPoint target;
    boolean ballGone;
    MMMotorGroup shooter;
    MMMotorGroup camAngle;
    MMMotorGroup feed;


    public ShooterStateMachine() {
        super(ShooterStates.Start);
        shooter = new MMFollowingMotorGroup(new MMSRXMotorController(Constants.kCanMCShooterShoot));
        camAngle = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterCam));
        feed = new MMFollowingMotorGroup(new MMSRXMotorController(Constants.kCanMCShooterFeed));
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
            camAngle.setPower(-.2);
            //home turret
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
