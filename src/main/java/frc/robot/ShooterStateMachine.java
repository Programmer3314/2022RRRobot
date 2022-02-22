// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Queue;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;
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
    MMJoystickAxis camPower;
    boolean shootAll;
    boolean shootOne;
    boolean abortShot;
    double shooterSpeed;
    double camControl;


    public ShooterStateMachine() {
        super(ShooterStates.Start);
        shooter = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterShoot)
                .setInverted(InvertType.InvertMotorOutput)
                .setPIDFParameters(Constants.kFXShooterWheelsP, Constants.kFXShooterWheelsI,
                        Constants.kFXShooterWheelsD,
                        Constants.kFXShooterWheelsF));

        camAngle = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterCam)
                .setPIDFParameters(Constants.kFXCamP, Constants.kFXCamI, Constants.kFXCamD,
                        Constants.kFXCamF)
                .setInverted(InvertType.InvertMotorOutput));
        feed = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterFeed)
                .setPIDFParameters(Constants.kFXShooterWheelsP, Constants.kFXShooterWheelsI,
                        Constants.kFXShooterWheelsD,
                        Constants.kFXShooterWheelsF));

        camlimitswitch = new DigitalInput(Constants.kDIOCamLimitSwitch);
        ballGoneBreakBeam = new DigitalInput(Constants.kDIOShooterBallGone);
        camPower = new MMJoystickAxis(Constants.kJoystickOperator, 5, .1, 1);
        shootAll = false;
        shootOne = false;
    }

    @Override
    public void CalcNextState() {

        // if (abortShot && currentState != ShooterStates.Home && currentState !=
        // ShooterStates.Start) {
        // nextState = ShooterStates.Idle;
        // } else {
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
                if (target != null && target.active && (shootOne || shootAll)) {
                    nextState = ShooterStates.Preparing;
                }
                break;
            case Preparing:
                // if (target.active && Robot.queueStateMachine.isFull()
                // && closeEnough(camAngle.getRevolutions(), target.angle,
                // Constants.kangleMargin)
                // && closeEnough(shooter.getVelocity(), target.rpm, Constants.krpmMargin)
                // && closeEnough(feed.getVelocity(), target.feedrpm, Constants.krpmMargin)
                // && closeEnough(Robot.aimController.turretError(), 0, target.turretMargin)) {

                SmartDashboard.putString("In Preparing ", "Yessir");
                SmartDashboard.putBoolean("close to shooter velocity",
                        closeEnough(shooter.getVelocity(), target.rpm, Constants.krpmMargin));
                SmartDashboard.putBoolean("close to feed velocity",
                        closeEnough(feed.getVelocity(), target.feedrpm, Constants.krpmMargin));
                SmartDashboard.putBoolean("Close to cam angle", closeEnough(camAngle.getRevolutions(), target.angle,
                Constants.kangleMargin));

                SmartDashboard.putBoolean("Target Active", target == null ? false : target.active);
                if (target.active && Robot.queueStateMachine.isFull()
                && closeEnough(camAngle.getRevolutions(), target.angle,
                Constants.kangleMargin)
                        && closeEnough(shooter.getVelocity(), target.rpm, Constants.krpmMargin)
                        && closeEnough(feed.getVelocity(), target.feedrpm, Constants.krpmMargin)
                // && closeEnough(Robot.aimController.turretError(), 0, target.turretMargin)
                ) {
                    passThroughCounter++;
                    SmartDashboard.putString("In Preparing/ passed If: ", "Yessir");
                    if (true || passThroughCounter > Constants.kShooterCounter) {
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
                    if (shootAll) {
                        nextState = ShooterStates.Preparing;
                        shootAll = false;
                        shootOne = true;
                    } else if (shootOne) {
                        shootOne = false;
                        shootAll =false;
                        nextState = ShooterStates.Idle;
                    }
                }
        }
        // }

    }

    @Override
    public void doTransition() {
        if (isTransitionTo(ShooterStates.Home)) {
            camAngle.setPower(-.2);
            target = null;
            // move turret
        }
        if (isTransitionTo(ShooterStates.Preparing)) {
            shooter.setVelocity(target.rpm);
            camAngle.setPosition(target.angle);
            feed.setVelocity(target.feedrpm);
            Robot.aimController.setAimMode(AimMode.robotShoot);
            // shooter.setVelocity(shooterSpeed);
            // camAngle.setPosition(camControl);
            // feed.setVelocity(shooterSpeed);
            // shooter.setPower(target.rpm / 6000);
            // feed.setPower(target.feedrpm / 6000);

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
        if (isTransitionFrom(ShooterStates.Shooting2)){
            Robot.aimController.setAimMode(AimMode.driver);
        }
    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case Home:
                // camhomed = true;
                if (camhomed) {
                    camAngle.setPower(0);
                    camAngle.resetEncoders();
                }
                homed = camhomed;
                break;
            case Preparing:
                shooter.setVelocity(target.rpm);
                camAngle.setPosition(target.angle);
                feed.setVelocity(target.feedrpm);
                break;
        }

    }

    public void setShootingSolution(TargetPoint target) {
        this.target = target;
        // passThroughCounter = 0;
    }

    @Override
    public void update() {
        camhomed = !camlimitswitch.get();
        // airBall = Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxAirBall);
        airBall = !ballGoneBreakBeam.get();
        SmartDashboard.putBoolean("shootAll", shootAll);
        SmartDashboard.putBoolean("ShootOne", shootOne);
        if (target != null) {
            // target.rpm = shooterSpeed;
            // target.feedrpm = shooterSpeed;
            // // target.angle = camControl;
            // SmartDashboard.putNumber("Manual Shooter", shooterSpeed);
            SmartDashboard.putNumber("Requested Shooter", target.rpm);
            SmartDashboard.putNumber("Requested Cam", target.angle);
            //SmartDashboard.putNumber("Manual Cam", camControl);
        }

        // if (Robot.controllerDriver.getPOV() == 0) {
        //     shooterSpeed += 200;
        // }
        // if (Robot.controllerDriver.getPOV() == 180) {
        //     shooterSpeed -= 200;
        // }

        // if (Robot.controllerDriver.getPOV() == 90) {
        //     camControl += 5;
        // }
        // if (Robot.controllerDriver.getPOV() == 270) {
        //     camControl -= 5;
        // }
        super.update();

        
        // camAngle.setPower(camPower.get());
        // camAngle.setPosition(camControl);
        // shooter.setVelocity(shooterSpeed);
        // feed.setVelocity(manualFeedPower);

        SmartDashboard.putBoolean("Air Ball", airBall);
        SmartDashboard.putString("Shooter SM", currentState.toString());
        
        SmartDashboard.putBoolean("CamLimitSwitch", camlimitswitch.get());
        SmartDashboard.putNumber("Returned Feed result", feed.getVelocity());
        SmartDashboard.putNumber("Returned Shooter", shooter.getVelocity());
        SmartDashboard.putNumber("Returned Cam", camAngle.getRevolutions());
        SmartDashboard.putNumber("Pass Through Counter", passThroughCounter);

        SmartDashboard.putBoolean("Queue Full", Robot.queueStateMachine.isFull());
        SmartDashboard.putBoolean("Shooter Abort", abortShot);
    }

    public void resetState() {
        currentState = ShooterStates.Start;
        shootAll = false;
        shootOne = false;
    }

    public boolean closeEnough(double value1, double value2, double margin) {
        return Math.abs(value1 - value2) <= margin;
    }

    public void shootAll() {
        shootAll = true;
    }

    public void shootOne() {
        shootOne = true;
    }

    public void abortShot(boolean abortshot) {
        abortshot = true;
        shootAll = false;
        shootOne = false;
    }
}
