// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utility.MMStateMachine;

enum TunnelStates {
    Start, Idle, BallDetected, MoveToQueue
};

/** Add your docs here. */
public class TunnelStateMachine extends MMStateMachine<TunnelStates> {
    public boolean desiredBall;
    public int counter;
    public boolean isRed;
    public boolean isBlue;

    public TunnelStateMachine() {
        super(TunnelStates.Start);

    }

    @Override
    public void update() {
        int red = Robot.frontColorSensor.getRed();
        int blue = Robot.frontColorSensor.getBlue();
        isRed = red > blue * 2;
        isBlue = blue > red * 2;
        desiredBall = ((Robot.alliance == Alliance.Blue && isBlue) || (Robot.alliance == Alliance.Red && isRed));

        super.update();
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TunnelStates.Idle;
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
        if (isTransitionTo(TunnelStates.MoveToQueue)) {
            Robot.queueStateMachine.takeBallFromTunnel();
            // Robot.tunnelWheels.setVelocity(500);
        }
        if (isTransitionTo(TunnelStates.Idle)) {
            // Robot.tunnelWheels.setVelocity(0);
            counter++;
        }

    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case Idle:
                // Robot.tunnelBelt.setVelocity(500);
                break;
            case BallDetected:
                // Robot.tunnelBelt.setVelocity(0);
                break;
        }

    }

}
