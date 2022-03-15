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
    Start, Idle, BreakBeamTrue, DistancePause1, DistancePause2, BallDetected, MoveToQueue, RejectBall
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
    public boolean tunnelBreakBeam;

    public TunnelStateMachine() {
        super(TunnelStates.Start);
        tunnelWheels = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelWheels));
        tunnelBelt = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelBelt));
        // breakBeamOne = new DigitalInput(Constants.kDIOTunnelBreakBeam);
        frontColorSensor = new ColorSensorV3(Port.kMXP);
        tunnelBreakInput = new DigitalInput(Constants.kDIOTunnelBreakBeam);
    }

    @Override
    public void update() {
        // TODO make desired ball laggy
        red = frontColorSensor.getRed();
        blue = frontColorSensor.getBlue();
        tunnelBreakBeam = !tunnelBreakInput.get();
        isRed = red > blue * 2;
        isBlue = blue > red * 2;
        // isRed = red > baseRed*1.05;
        // isBlue = blue > baseBlue*1.05;
        // desiredBall = ((Robot.alliance == Alliance.Blue && isBlue) || (Robot.alliance
        // == Alliance.Red && isRed));
        desiredBall = ((Robot.alliance == Alliance.Blue && isBlue && !isRed)
                || (Robot.alliance == Alliance.Red && isRed && !isBlue)); // && !breakBeamOne.get();
        // desiredBall =
        // Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxDesiredBall);
        tunnelBeltRPM = tunnelBelt.getVelocity();
        tunnelWheelsRPM = tunnelWheels.getVelocity();
        queueIsFull = Robot.queueStateMachine.isFull();
        climbing = Robot.climbStateMachine.isClimbing();

        SmartDashboard.putBoolean("Desired Ball", desiredBall);
        SmartDashboard.putBoolean("isRed", isRed);
        SmartDashboard.putBoolean("isBlue", isBlue);
        SmartDashboard.putString("Tunnel State", currentState.toString());
        SmartDashboard.putNumber("Amount of Red Detected:", red);
        SmartDashboard.putNumber("Amount of Blue Detected: ", blue);
        // SmartDashboard.putBoolean("breakBeamOne", breakBeamOne.get());
        SmartDashboard.putNumber("Green Tunnel Wheels", tunnelWheelsRPM);
        SmartDashboard.putBoolean("TunnelBreakBeam", tunnelBreakBeam);
        SmartDashboard.putBoolean("RawTunnelBreakBeam", tunnelBreakInput.get());

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
                    if (tunnelBreakBeam) {
                        // if(desiredBall){
                        // nextState = TunnelStates.BallDetected;
                        nextState = TunnelStates.DistancePause1;
                    }
                    break;
                case DistancePause1:
                if(!tunnelBreakBeam){
                    nextState = TunnelStates.DistancePause2;
                }
                break;
                case DistancePause2:
                if(cyclesInStates>5){
                    nextState = TunnelStates.BreakBeamTrue;
                }
                break;
                case BreakBeamTrue:
                    if (desiredBall) {
                        nextState=TunnelStates.BallDetected;
                    }
                    else{
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
            }
        }
    }

    @Override
    public void doTransition() {
        // The FROM on the next line was correct. The Green wheels don't turn on until
        // the Queue is ready. The problem seems to be that we're not moving to
        // MoveToQueue
        // or we are but there is another problem
        if (isTransitionFrom(TunnelStates.BallDetected)) {
            Robot.queueStateMachine.takeBallFromTunnel();
            tunnelWheels.setPower(0.6);
        }
        if (isTransitionTo(TunnelStates.Idle)) {
            tunnelWheels.setPower(0);
            counter++;
        }
        if (isTransitionTo(TunnelStates.RejectBall)) {
            tunnelWheels.setPower(-.4);
            tunnelBelt.setPower(.4);
        }

    }

    @Override
    public void doCurrentState() {
        if (!climbing) {
            switch (currentState) {
                case Start:
                    baseRed = red;
                    baseBlue = blue;
                case Idle:
                    tunnelBelt.setPower(0.15);

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
    }

    public void LogHeader() {
        Logger.Header("TunnelBeltRPM,tunnelWheelsRPM, red, blue"
                + "colorSensorRed, colorSensorBlue,desiredBall,tunnelbreakbeam,"
                + "TunnelState,");
    }

    public void LogData() {
        Logger.doubles(tunnelBeltRPM, tunnelWheelsRPM, red, blue);
        Logger.booleans(isRed, isBlue, desiredBall, tunnelBreakBeam);
        Logger.singleEnum(currentState);
    }
}
