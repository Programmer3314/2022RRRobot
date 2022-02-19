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
import frc.robot.Constants.*;

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
    Start, Idle, BallDetected, MoveToQueue
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
    int baseBlue;
    int baseRed;
    


    public TunnelStateMachine() {
        super(TunnelStates.Start);
        tunnelWheels = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelWheels));
        tunnelBelt = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCTunnelBelt));
        breakBeamOne = new DigitalInput(Constants.kDIOTunnelBreakBeam);
        frontColorSensor = new ColorSensorV3(Port.kMXP);
    }

    @Override
    public void update() {
        //TODO make desired ball laggy
        int red = frontColorSensor.getRed();
        int blue = frontColorSensor.getBlue();
        isRed = red > blue * 2;
        isBlue = blue > red * 2;
        // isRed = red > baseRed*1.05;
        // isBlue = blue > baseBlue*1.05;
        desiredBall = ((Robot.alliance == Alliance.Blue && isBlue && !isRed) || (Robot.alliance == Alliance.Red && isRed && !isBlue)); //&& !breakBeamOne.get();
        //desiredBall = Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxDesiredBall);

        SmartDashboard.putBoolean("Desired Ball", desiredBall);
        SmartDashboard.putBoolean("isRed", isRed);
        SmartDashboard.putBoolean("isBlue", isBlue);
        SmartDashboard.putString("Tunnel State", currentState.toString());
        SmartDashboard.putNumber("Amount of Red Detected:", frontColorSensor.getRed());
        SmartDashboard.putNumber("Amount of Blue Detected: ", frontColorSensor.getBlue());
        SmartDashboard.putBoolean("breakBeamOne", breakBeamOne.get());
        SmartDashboard.putNumber("Green Tunnel Wheels", tunnelWheels.getVelocity());

        super.update();
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TunnelStates.Idle;
                break;
            case Idle:                                                                                                          
                if (desiredBall) { 
                    nextState = TunnelStates.BallDetected;
                }
                break;
            case BallDetected:
                if (!Robot.queueStateMachine.isFull()) {
                    nextState = TunnelStates.MoveToQueue;
                }
                break;
            case MoveToQueue:
                if (Robot.queueStateMachine.isFull()) {
                    nextState = TunnelStates.Idle;
                }
                break;

        }
    }

    @Override
    public void doTransition() {
        // The FROM on the next line was correct. The Green wheels don't turn on until
        // the Queue is ready. The problem seems to be that we're not moving to MoveToQueue
        // or we are but there is another problem 
        if (isTransitionFrom(TunnelStates.BallDetected)) {
            Robot.queueStateMachine.takeBallFromTunnel();
            tunnelWheels.setPower(0.6);
        }
        if (isTransitionTo(TunnelStates.Idle)) {
            tunnelWheels.setPower(0);
            counter++;
        }

    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case Start:
                baseRed = frontColorSensor.getRed();
                baseBlue =  frontColorSensor.getBlue();
            case Idle:
                tunnelBelt.setPower(0.15);

                break;
            case BallDetected:
                tunnelBelt.setPower(0.0);
                break;
        }

    }
    public void resetState(){
        currentState = TunnelStates.Start;
    }
}
