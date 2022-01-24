// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

// TODO Add initial call to doCurrentState() on first update
// TODO Add secondsInState and cyclesInState variables

/** 
 * Generic State Machine 
 */
public abstract class MMStateMachine<T> {
    public T currentState;
    public T nextState;

    public MMStateMachine(T initState) {
        currentState = initState;
    }

    public void update() {
        nextState = currentState;
        CalcNextState();
        if (nextState != currentState) {
            doTransition();
            currentState = nextState;
        }
        doCurrentState();
    }

    public abstract void CalcNextState();

    public abstract void doTransition();

    public abstract void doCurrentState();
}