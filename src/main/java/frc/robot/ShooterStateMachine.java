// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMJoystickAxis;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMRollingAverage;
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
// TODO when to clear passThroughCounter
// TODO make sure that we are still on the target, even after 2+ shots in
// succession(with aiming state machine)

enum ShooterStates {
    Start, Home, Idle, Preparing, Shooting1, Shooting2, RejectBall
};

/** Add your docs here. */
public class ShooterStateMachine extends MMStateMachine<ShooterStates> {

    boolean homed;
    TargetPoint target;
    TargetPoint prepTarget;
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

    double shooterRPM;
    double feedRPM;
    double camRevs;
    boolean queueIsFull;
    double smoothedShooterRPM;
    double smoothedFeedRPM;
    MMRollingAverage rollingShooterRPM;
    MMRollingAverage rollingFeedRPM;

    public ShooterStateMachine() {
        super(ShooterStates.Start);
        shooter = new MMFollowingMotorGroup(
                new MMFXMotorController(Constants.kCanMCShooterShoot)
                        .setInverted(InvertType.None)
                        .setPIDFParameters(Constants.kFXShooterWheelsP, Constants.kFXShooterWheelsI,
                                Constants.kFXShooterWheelsD,
                                Constants.kFXShooterWheelsF,
                                Constants.kFXShooterWheelsIzone),
                new MMFXMotorController(Constants.kCanMCShooterTwo)
                        .setInverted(InvertType.OpposeMaster));

        camAngle = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterCam)
                .setPIDFParameters(Constants.kFXCamP, Constants.kFXCamI, Constants.kFXCamD,
                        Constants.kFXCamF)
                .setInverted(InvertType.InvertMotorOutput));
        feed = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCShooterFeed)
                .setInverted(InvertType.InvertMotorOutput)
                .setPIDFParameters(Constants.kFXFeedWheelsP, Constants.kFXFeedWheelsI,
                        Constants.kFXFeedWheelsD,
                        Constants.kFXFeedWheelsF,
                        Constants.kFXFeedWheelsIzone));

        camlimitswitch = new DigitalInput(Constants.kDIOCamLimitSwitch);
        ballGoneBreakBeam = new DigitalInput(Constants.kDIOShooterBallGone);
        camPower = new MMJoystickAxis(Constants.kJoystickOperator, 5, .1, 1);
        shootAll = false;
        shootOne = false;
        rollingFeedRPM = new MMRollingAverage(10);
        rollingShooterRPM = new MMRollingAverage(10);
    }

    @Override
    public void update() {
        camhomed = !camlimitswitch.get();
        queueIsFull = Robot.queueStateMachine.isFull();
        // airBall = Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxAirBall);
        airBall = !ballGoneBreakBeam.get() || Robot.controllerOperator.getRawButton(3);// manual override for airball
        SmartDashboard.putBoolean("shootAll", shootAll);
        SmartDashboard.putBoolean("ShootOne", shootOne);
        SmartDashboard.putBoolean("CamHOMED", homed);
        SmartDashboard.putBoolean("HOMEAimController", Robot.aimController.isHomed());
        if (target != null) {
            // target.rpm = shooterSpeed;
            // target.feedrpm = shooterSpeed;
            // // target.angle = camControl;
            // SmartDashboard.putNumber("Manual Shooter", shooterSpeed);
            SmartDashboard.putNumber("Requested Shooter", target.rpm);
            SmartDashboard.putNumber("Requested Cam", target.angle);
            // SmartDashboard.putNumber("Manual Cam", camControl);
        }

        // if (Robot.controllerDriver.getPOV() == 0) {
        // shooterSpeed += 200;
        // }
        // if (Robot.controllerDriver.getPOV() == 180) {
        // shooterSpeed -= 200;
        // }

        // if (Robot.controllerDriver.getPOV() == 90) {
        // camControl += 5;
        // }
        // if (Robot.controllerDriver.getPOV() == 270) {
        // camControl -= 5;
        // }
        shooterRPM = shooter.getVelocity();
        feedRPM = feed.getVelocity();
        camRevs = camAngle.getRevolutions();

        smoothedFeedRPM = rollingFeedRPM.update(feedRPM);
        smoothedShooterRPM = rollingShooterRPM.update(shooterRPM);

        super.update();

        // camAngle.setPower(camPower.get());
        // camAngle.setPosition(camControl);
        // shooter.setVelocity(shooterSpeed);
        // feed.setVelocity(manualFeedPower);

        SmartDashboard.putBoolean("Air Ball", airBall);
        SmartDashboard.putString("Shooter SM", currentState.toString());

        SmartDashboard.putBoolean("CamLimitSwitch", camlimitswitch.get());
        SmartDashboard.putNumber("Returned Feed result", feedRPM);
        SmartDashboard.putNumber("Returned Shooter", shooterRPM);
        SmartDashboard.putNumber("Returned Cam", camRevs);
        SmartDashboard.putNumber("Pass Through Counter", passThroughCounter);

        SmartDashboard.putBoolean("Queue Full", queueIsFull);
        SmartDashboard.putBoolean("Shooter Abort", abortShot);
    }

    @Override
    public void CalcNextState() {
        if (Robot.tacoBell) {
            nextState = ShooterStates.RejectBall;
        } else {
            // if (abortShot && currentState != ShooterStates.Home && currentState !=
            // ShooterStates.Start) {
            // nextState = ShooterStates.Idle;
            // } else {
            switch (currentState) {
                case Start:
                    nextState = ShooterStates.Home;
                    break;
                case Home:
                    if (homed/** && Robot.aimController.isHomed() */
                    ) {
                        nextState = ShooterStates.Idle;
                    }
                    break;
                case Idle:
                    if (((target != null && target.active) || !Robot.useVision) && (shootOne || shootAll)) {
                        nextState = ShooterStates.Preparing;
                    }
                    break;
                case Preparing:
                    SmartDashboard.putNumber("Auto correct Target angle", Robot.hubTargetAngle);
                    SmartDashboard.putString("ShotType", Robot.shotType.toString());
                    if (target != null) {
                        // added check just in case...
                        // SmartDashboard.putString("In Preparing ", "Yessir");
                        SmartDashboard.putBoolean("close to shooter velocity",
                                closeEnough(shooterRPM, target.rpm, Constants.krpmMargin));
                        SmartDashboard.putBoolean("close to feed velocity",
                                closeEnough(feedRPM, target.feedrpm, Constants.krpmFeedMargin));
                        SmartDashboard.putBoolean("Close to cam angle",
                                closeEnough(camRevs, target.angle, Constants.kangleMargin));
                        SmartDashboard.putBoolean("Close to target Angle",
                                closeEnough(Robot.currentShooterAngle, Robot.hubTargetAngle, target.turretMargin));
                        SmartDashboard.putBoolean("Target Active", target.active);
                        SmartDashboard.putNumber("Target Margin WW", target.turretMargin);

                        // double marginScale=1;
                        // if(Robot.shotType==ShotType.PointBlankLow){
                        // marginScale=1.5;
                        // }

                        if (queueIsFull
                                && closeEnough(camRevs, target.angle, Constants.kangleMargin)
                                && (/*this.cyclesInStates>80 ||*/closeEnough(shooterRPM, target.rpm, Constants.krpmMargin))
                                && (/*this.cyclesInStates>80 ||*/closeEnough(feedRPM, target.feedrpm, Constants.krpmFeedMargin))
                                && ((target.active && closeEnough(Robot.currentShooterAngle, Robot.hubTargetAngle,
                                        target.turretMargin))
                                        || Robot.shotType != ShotType.Vision
                                        || !Robot.useVision)) {
                            passThroughCounter++;
                            // SmartDashboard.putString("In Preparing/ passed If: ", "Yessir");
                            if (passThroughCounter > Constants.kShooterCounter) {
                                nextState = ShooterStates.Shooting1;
                            }
                        } else {
                            if (!target.active && Robot.shotType == ShotType.Vision) {
                                nextState = ShooterStates.Idle;
                            }
                            passThroughCounter = 0;
                        }
                    } else {
                        passThroughCounter = 0;
                        SmartDashboard.putBoolean("close to shooter velocity", false);
                        SmartDashboard.putBoolean("close to feed velocity", false);
                        SmartDashboard.putBoolean("Close to cam angle", false);
                        SmartDashboard.putBoolean("Close to target Angle", false);
                        SmartDashboard.putBoolean("Target Active", false);
                        SmartDashboard.putNumber("Target Margin WW", 0);
                    }

                    if (abortShot) {
                        nextState = ShooterStates.Idle;
                        abortShot = false;
                    }
                    break;
                case Shooting1:
                    if (!airBall) {
                        nextState = ShooterStates.Shooting2;
                    }
                    if (abortShot) {
                        nextState = ShooterStates.Idle;
                        abortShot = false;
                    }
                    break;
                case Shooting2:
                    if (airBall) {
                        if (shootAll) {
                            nextState = ShooterStates.Preparing;
                            shootAll = false;
                            if (Robot.tunnelStateMachine.currentState == TunnelStates.BallDetected
                                    || Robot.tunnelStateMachine.currentState == TunnelStates.MoveToQueue) {
                                shootOne = true;
                            } else {
                                shootOne = false;
                                nextState = ShooterStates.Idle;
                            }
                        } else if (shootOne) {
                            shootOne = false;
                            shootAll = false;
                            nextState = ShooterStates.Idle;

                        }
                    }
                    if (abortShot) {
                        nextState = ShooterStates.Idle;
                        abortShot = false;
                    }
                    break;
                case RejectBall:
                    if (!Robot.tacoBell) {
                        nextState = ShooterStates.Idle;

                    }
            }
        }
        // }
    }

    @Override
    public void doTransition() {
        if (isTransitionFrom(ShooterStates.Shooting2)) {
            // Robot.aimController.setAimMode(AimMode.driver);
        }
        if (isTransitionTo(ShooterStates.Home)) {
            camAngle.setPower(-.1);
            target = null;
            // move turret
        }
        if (isTransitionTo(ShooterStates.Preparing)) {
            shooter.setVelocity(target.rpm);
            camAngle.setPosition(target.angle);
            feed.setVelocity(target.feedrpm);
            // Robot.aimController.setAimMode(AimMode.robotShoot);

            // shooter.setVelocity(shooterSpeed);
            // camAngle.setPosition(camControl);
            // feed.setVelocity(shooterSpeed);
            // shooter.setPower(target.rpm / 6000);
            // feed.setPower(target.feedrpm / 6000);

            passThroughCounter = 0;
        }

        if (isTransitionTo(ShooterStates.Shooting1)) {
            Robot.queueStateMachine.shooterBallRequest();
            Robot.takeSnapshot();
        }

        if (isTransitionTo(ShooterStates.Idle)) {
            shooter.setPower(0);
            camAngle.setPower(0);
            feed.setPower(0);
        }

        if (isTransitionTo(ShooterStates.RejectBall)) {
            shooter.setPower(-.3);
            feed.setPower(-.3);
            shootOne = false;
            shootAll = false;
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
                // SmartDashboard.putString("HOMECheck:", "YESSIRE");
                homed = camhomed;
                break;
            case Preparing:
                shooter.setVelocity(target.rpm);
                camAngle.setPosition(target.angle);
                feed.setVelocity(target.feedrpm);
                SmartDashboard.putNumber("secondsInPreparing", secondsInState);
                break;
            default:
                break;
        }

    }

    public void setShootingSolution(TargetPoint target) {
        this.target = target;
        // passThroughCounter = 0;
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

    public void abortShot() {
        abortShot = true;
        shootAll = false;
        shootOne = false;
    }

    public void LogHeader() {
        Logger.Header(
                "FeedRPM, ShooterRPM, CamAngle,CurrentShooterAngle,TargetFeed,TargetShooter,TargetAngle,TargetActive,"
                        + "HubTargetAngle,PassThroughCounter,smoothedFeedRPM,smoothedShooterRPM,"
                        + "BallGone, camhomed, QueueFull,pointBlankButton,bottomBasket,povLeft, povRight, shootOne, shootAll,"
                        + "ShooterState,ShotType,");
    }

    public void LogData() {
        Logger.doubles(feedRPM, shooterRPM, camRevs, Robot.currentShooterAngle);// target.feedrpm,target.rpm,target.angle);
        if (target == null) {
            Logger.doubles(0.0, 0.0, 0.0);
            Logger.booleans(false);
        } else {
            Logger.doubles(target.feedrpm, target.rpm, target.angle);
            Logger.booleans(target.active);
        }
        Logger.doubles(Robot.hubTargetAngle, passThroughCounter);
        Logger.doubles(smoothedFeedRPM, smoothedShooterRPM);
        Logger.booleans(airBall, camhomed, queueIsFull, Robot.pointBlankHigh, Robot.pointBlankLow, Robot.povLeftShot,
                Robot.povRightShot, shootOne, shootAll);
        Logger.singleEnum(currentState);
        Logger.singleEnum(Robot.shotType);
    }
}
