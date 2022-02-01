// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//The WPILib advisor declares that Queueueueue.giveBallToShooter = ShooterBallRequest

package frc.robot;

import frc.robot.utility.MMStateMachine;

enum QueueStates {
    Start, WaitForBall, DrawBallIn, GotBall, SendingBall
};

/** Add your docs here. */
public class QueueStateMachine extends MMStateMachine<QueueStates> {

    boolean takeBallFromTunnel = false;
    boolean ballPositionInQueue = false;
    boolean shooterBallRequest = false;
    boolean queueFull = false;

    public QueueStateMachine() {
        super(QueueStates.Start);

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
        if(isTransitionTo(QueueStates.DrawBallIn)){
            Robot.queueBelt.setVelocity(200);
            takeBallFromTunnel = false;
        
        }
        if(isTransitionTo(QueueStates.GotBall)){
            Robot.queueBelt.setVelocity(0);
            queueFull = true;
            
        }
        if (isTransitionTo(QueueStates.SendingBall)){
            Robot.queueBelt.setVelocity(300);
            shooterBallRequest = false;
            
        }
        if(isTransitionTo(QueueStates.WaitForBall)){
            Robot.queueBelt.setVelocity(0);
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
    public boolean isFull(){
        return queueFull;
    }
}