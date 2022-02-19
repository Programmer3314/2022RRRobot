// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSRXMotorController;
import frc.robot.utility.MMStateMachine;
import frc.robot.Constants.*;

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
// TODO when to clear passThroughCounter
// TODO make sure that we are still on the target, even after 2+ shots in
// succession(with aiming state machine)

enum ShooterStates {
    Start, Home, Idle, Preparing, Shooting1, Shooting2
};

/** Add your docs here. */
public class ShooterStateMachine extends MMStateMachine<ShooterStates> {

    boolean homed;
    TargetPoint target;
    boolean airBall;// true when beam is not broken, ball not there
    MMMotorGroup shooter;
    MMMotorGroup camAngle;
    MMMotorGroup feed;
    boolean camhomed;
    DigitalInput camlimitswitch;
    DigitalInput ballGoneBreakBeam;
    int passThroughCounter;

    public ShooterStateMachine() {
        super(ShooterStates.Start);
        shooter = new MMFollowingMotorGroup(new MMSRXMotorController(Constants.kCanMCShooterShoot)
        .setInverted(InvertType.InvertMotorOutput));
        camAngle = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterCam));
        feed = new MMFollowingMotorGroup(new MMSRXMotorController(Constants.kCanMCShooterFeed));
        camlimitswitch = new DigitalInput(Constants.kDIOCamLimitSwitch);
        ballGoneBreakBeam = new DigitalInput(Constants.kDIOShooterBallGone);

        
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = ShooterStates.Home;
                break;
            case Home:
                if (homed && Robot.aimController.isHomed()) {
                    nextState = ShooterStates.Idle;
                }
                break;
            case Idle:
                if (target.active && (Robot.shootOneButton || Robot.shootAllButton)) {
                    nextState = ShooterStates.Preparing;
                }
                break;
            case Preparing:
                if (target.active && Robot.queueStateMachine.isFull()
                        && closeEnough(camAngle.getRevolutions(), target.angle, Constants.kangleMargin)
                        && closeEnough(shooter.getVelocity(), target.rpm, Constants.krpmMargin)
                        && closeEnough(feed.getVelocity(), target.feedrpm, Constants.krpmMargin)
                        && closeEnough(Robot.aimController.turretError(), 0, target.turretMargin)) {
                    passThroughCounter++;

                    if (passThroughCounter > Constants.kShooterCounter) {
                        nextState = ShooterStates.Shooting1;
                    }
                } else if (!target.active) {
                    nextState = ShooterStates.Idle;
                }
                break;
            case Shooting1:
                if (!airBall) {
                    nextState = ShooterStates.Shooting2;
                }
                break;
            case Shooting2:
                if (airBall) {
                    if (Robot.shootAllButton) {
                        nextState = ShooterStates.Preparing;
                    }
                    if (Robot.shootOneButton) {
                        nextState = ShooterStates.Idle;
                    }
                }
        }
    }

    @Override
    public void doTransition() {
        if (isTransitionTo(ShooterStates.Home)) {
            camAngle.setPower(-.2);
            // move turret
        }
        if (isTransitionTo(ShooterStates.Preparing)) {
            //shooter.setVelocity(target.rpm);
            //camAngle.setPosition(target.angle);
            //feed.setVelocity(target.feedrpm);
            shooter.setPower(target.rpm/6000);
            feed.setPower(target.feedrpm/6000);


            passThroughCounter = 0;
        }
        if (isTransitionTo(ShooterStates.Shooting1)) {
            Robot.queueStateMachine.shooterBallRequest();
        }
        if (isTransitionTo(ShooterStates.Idle)) {
            shooter.setPower(0);
            camAngle.setPower(0);
            feed.setPower(0);
        }
    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case Home:
                camhomed = true;
                if (camhomed) {
                    camAngle.setPower(0);
                }
                homed = camhomed;
                break;
        }

    }

    public void setShootingSolution(TargetPoint target) {
        this.target = target;
        passThroughCounter = 0;
    }

    @Override
    public void update() {
        camhomed = camlimitswitch.get();
        airBall = Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxAirBall);
        //airBall = ballGoneBreakBeam.get();
        super.update();

        SmartDashboard.putString("Shooter State", currentState.toString());
    }

    public void resetState(){
        currentState = ShooterStates.Start;
    }

    public boolean closeEnough(double value1, double value2, double margin) {
        return Math.abs(value1 - value2) <= margin;
    }
}
