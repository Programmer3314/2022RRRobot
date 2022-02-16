// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//The WPILib advisor declares that Queueueueue.giveBallToShooter = ShooterBallRequest

package frc.robot;

import frc.robot.utility.MMStateMachine;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.Constants.*;

/**
 * Expected Hardware configuration:
 * Break Beam sensor in ball path to detect when ball is ready/gone
 * Single motor (implement motor group anyway)
 */

enum QueueStates {
    Start, WaitForBall, DrawBallIn, GotBall, SendingBall
};

/** Add your docs here. */
public class QueueStateMachine extends MMStateMachine<QueueStates> {

    MMMotorGroup queueBelt;
    boolean takeBallFromTunnel = false;
    boolean ballPositionInQueue = false;
    boolean shooterBallRequest = false;
    public static boolean queueFull = false;
    DigitalInput breakBeamOne;

    public QueueStateMachine() {
        super(QueueStates.Start);
        breakBeamOne = new DigitalInput(Constants.kDIOQueueBreakBeam);
        queueBelt = new MMFollowingMotorGroup(new MMFXMotorController(Constants.kCanMCQueueBelt));
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = QueueStates.WaitForBall;
                break;
            case WaitForBall:
                if (takeBallFromTunnel) {
                    nextState = QueueStates.DrawBallIn;
                }
                break;
            case DrawBallIn:
                if (ballPositionInQueue) {
                    nextState = QueueStates.GotBall;
                }
                break;
            case GotBall:
                if (shooterBallRequest) {
                    nextState = QueueStates.SendingBall;
                }
                break;
            case SendingBall:
                if (!ballPositionInQueue) {
                    nextState = QueueStates.WaitForBall;
                }
                break;
        }

    }

    @Override
    public void doTransition() {
        if (isTransitionTo(QueueStates.DrawBallIn)) {
            queueBelt.setVelocity(200);
            takeBallFromTunnel = false;

        }
        if (isTransitionTo(QueueStates.GotBall)) {
            queueBelt.setVelocity(0);
            queueFull = true;

        }
        if (isTransitionTo(QueueStates.SendingBall)) {
            queueBelt.setVelocity(300);
            shooterBallRequest = false;

        }
        if (isTransitionTo(QueueStates.WaitForBall)) {
            queueBelt.setVelocity(0);
            queueFull = false;
        }

    }

    @Override
    public void doCurrentState() {
    }

    public void takeBallFromTunnel() {
        takeBallFromTunnel = true;
    }

    public void shooterBallRequest() {
        shooterBallRequest = true;

    }

    public boolean isFull() {
        return queueFull;
    }
    
    public void resetState(){
        currentState = QueueStates.Start;
    }

    @Override
    public void update(){
        //ballPositionInQueue = breakBeamOne.get();
        ballPositionInQueue = Robot.buttonBox1.getRawButton(Constants.kTestButtonBoxQueuePosition);
        super.update();
    }
}