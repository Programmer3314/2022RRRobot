// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Start
 * Idle: Tunnelbelt moves
 * 
 */

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMStateMachine;

/**
 * Expected Hardware configuration:
 * Break Beam sensors in ball path to detect entry/exit
 * Break Beam sensor(s) to position ball at green wheels
 * Color Sensor V3
 * Belt motor (implement motor group anyway)
 * Green Wheel motor
 * Existing Motor Encoders
 */

enum TunnelStates {
    Start, Idle, BallIsLive, EncoderDelay, BallInPosition, BallDetected, MoveToQueue, RejectBall
};

/** Add your docs here. */
public class TunnelStateMachine extends MMStateMachine<TunnelStates> {
    public boolean desiredBall;
    public int counter;
    public boolean isRed;
    public boolean isBlue;
    MMMotorGroup tunnelBelt;
    MMMotorGroup tunnelWheels;
    ColorSensorV3 frontColorSensor;
    DigitalInput breakBeamOne;
    DigitalInput tunnelBreakInput;
    int baseBlue;
    int baseRed;
    double tunnelBeltRPM;
    double tunnelWheelsRPM;
    boolean queueIsFull;
    int red, blue;
    public boolean climbing;
    public boolean tunnelBreakBeamBroken;
    double whiteBeltCurrent;
    double whiteBeltOffset = 2.0 - 1.61;
    double whiteBeltSensorOffset = 1.61;
    double whiteBeltGoal;
    boolean ignoreColorSensor;
    double whiteBeltNormalSpeed = 0.275;
    boolean trident;

    public TunnelStateMachine() {
        super(TunnelStates.Start);
        tunnelWheels = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelWheels));
        tunnelBelt = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelBelt)
                .setBrakeMode(true));
        // breakBeamOne = new DigitalInput(Constants.kDIOTunnelBreakBeam);
        frontColorSensor = new ColorSensorV3(Port.kMXP);
        tunnelBreakInput = new DigitalInput(Constants.kDIOTunnelBreakBeam);
    }

    @Override
    public void update() {
        red = frontColorSensor.getRed();
        blue = frontColorSensor.getBlue();
        tunnelBreakBeamBroken = !tunnelBreakInput.get();
        isRed = red > blue * 1.75;//deux
        isBlue = blue > red * 1.75;//dos
        whiteBeltCurrent = tunnelBelt.getRevolutions();
        ignoreColorSensor = Robot.buttonBox1.getRawButton(Constants.kButtonBoxIgnoreColorSensor);
        // isRed = red > baseRed*1.05;
        // isBlue = blue > baseBlue*1.05;
        // desiredBall = ((Robot.alliance == Alliance.Blue && isBlue) || (Robot.alliance
        // == Alliance.Red && isRed));
        // desiredBall = ((Robot.alliance == Alliance.Blue && isBlue && !isRed)
        // || (Robot.alliance == Alliance.Red && isRed && !isBlue) ||
        // ignoreColorSensor); // && !breakBeamOne.get();
        // desiredBall =
        // Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxDesiredBall);
        tunnelBeltRPM = tunnelBelt.getVelocity();
        tunnelWheelsRPM = tunnelWheels.getVelocity();
        queueIsFull = Robot.queueStateMachine.isFull();
        climbing = Robot.climbStateMachine.isClimbing();

        SmartDashboard.putBoolean("Ignore Color Sensor", ignoreColorSensor);
        SmartDashboard.putBoolean("Desired Ball", desiredBall);
        SmartDashboard.putBoolean("isRed", isRed);
        SmartDashboard.putBoolean("isBlue", isBlue);
        SmartDashboard.putString("Tunnel State", currentState.toString());
        SmartDashboard.putNumber("Amount of Red Detected:", red);
        SmartDashboard.putNumber("Amount of Blue Detected: ", blue);
        // SmartDashboard.putBoolean("breakBeamOne", breakBeamOne.get());
        SmartDashboard.putNumber("Green Tunnel Wheels", tunnelWheelsRPM);
        SmartDashboard.putBoolean("TunnelBreakBeam", tunnelBreakBeamBroken);
        SmartDashboard.putBoolean("RawTunnelBreakBeam", tunnelBreakInput.get());
        SmartDashboard.putBoolean("ColorSensorFailed", turnOffSensor());

        super.update();
    }

    @Override
    public void CalcNextState() {
        if (Robot.tacoBell) {
            nextState = TunnelStates.RejectBall;
        } else {
            switch (currentState) {
                case Start:
                    nextState = TunnelStates.Idle;
                    break;
                case Idle:
                    if (tunnelBreakBeamBroken) {
                        // if(desiredBall){
                        // nextState = TunnelStates.BallDetected;
                        nextState = TunnelStates.BallIsLive;
                    }
                    break;
                case BallIsLive:
                    if (whiteBeltCurrent >= whiteBeltGoal) {
                        nextState = TunnelStates.EncoderDelay;
                    }
                    break;
                case EncoderDelay:
                    if (whiteBeltCurrent >= whiteBeltGoal) {
                        nextState = TunnelStates.BallInPosition;
                    }
                    break;
                case BallInPosition:
                    if (desiredBall) {
                        nextState = TunnelStates.BallDetected;
                    } else {
                        nextState = TunnelStates.Idle;
                    }
                    break;
                case BallDetected:
                    if (!queueIsFull) {
                        nextState = TunnelStates.MoveToQueue;
                    }
                    break;
                case MoveToQueue:
                    if (queueIsFull) {
                        nextState = TunnelStates.Idle;
                    }
                    break;
                case RejectBall:
                    if (!Robot.tacoBell) {
                        nextState = TunnelStates.Idle;
                    }
                    break;
            }
        }
    }

    @Override
    public void doTransition() {
        if (isTransitionFrom(TunnelStates.BallDetected)) {
            Robot.queueStateMachine.takeBallFromTunnel();
            tunnelWheels.setPower(0.6);
        }
        if (isTransitionFrom(TunnelStates.BallInPosition)) {
            desiredBall = false;
        }

        if (isTransitionTo(TunnelStates.BallIsLive)) {
            whiteBeltGoal = whiteBeltCurrent + whiteBeltSensorOffset;
        }
        if (isTransitionTo(TunnelStates.EncoderDelay)) {
            whiteBeltGoal = whiteBeltCurrent + whiteBeltOffset;
            desiredBall = ((Robot.alliance == Alliance.Blue && isBlue && !isRed)
                    || (Robot.alliance == Alliance.Red && isRed && !isBlue) || ignoreColorSensor||turnOffSensor());
        }
        if (isTransitionTo(TunnelStates.Idle)) {
            tunnelWheels.setPower(0);
            counter++;
            tunnelBelt.resetEncoders();
        }
        if (isTransitionTo(TunnelStates.RejectBall)) {
            tunnelWheels.setPower(-.4);
            tunnelBelt.setPower(.4);
        }
    }

    @Override
    public void doCurrentState() {
        if (!climbing&&!trident) {
            switch (currentState) {
                case Start:
                    baseRed = red;
                    baseBlue = blue;
                case Idle:
                    tunnelBelt.setPower(whiteBeltNormalSpeed);
                    break;
                case BallDetected:
                    tunnelBelt.setPower(0.0);
                    break;
                default:
                    break;
            }
        } else {
            tunnelBelt.setPower(0);
        }
    }

    public void resetState() {
        desiredBall = false;
        currentState = TunnelStates.Start;
        trident = false;
    }

    public void LogHeader() {
        Logger.Header("TunnelBeltRPM,tunnelWheelsRPM, red, blue, TunnelEncoder,"
                + "isRed, isBlue,desiredBall,tunnelbreakbeam,"
                + "TunnelState,");
    }

    public void LogData() {
        Logger.doubles(tunnelBeltRPM, tunnelWheelsRPM, red, blue, tunnelBelt.getRevolutions());
        Logger.booleans(isRed, isBlue, desiredBall, tunnelBreakBeamBroken);
        Logger.singleEnum(currentState);
    }

    public void toggleBelt() {
        trident = !trident;
    }
    boolean turnOffSensor(){
        return red==0&&blue==0;
    }
}
