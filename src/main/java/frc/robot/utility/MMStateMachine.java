// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

// TODO Add secondsInState and cyclesInState variables
// TODO add non-abstract methods isTransition, isTransitionFrom, isTransitionTo
// these should take these should check From&To, From, To states respectively.

/**
 * Generic State Machine
 */
public abstract class MMStateMachine<T> {
    public T currentState;
    public T nextState;
    public boolean firstTimeRun;

    public MMStateMachine(T initState) {
        currentState = initState;
        firstTimeRun = true;
    }

    public void update() {
        if (!firstTimeRun) {
            nextState = currentState;
            CalcNextState();
            if (nextState != currentState) {
                doTransition();
                currentState = nextState;
            }
        }
        doCurrentState();
        firstTimeRun = false;
    }

    public abstract void CalcNextState();

    public abstract void doTransition();

    public abstract void doCurrentState();

    public boolean isTransitionFrom(T state) {
        return state == currentState;
    }

    public boolean isTransitionTo(T state) {
        return state == nextState;
    }

    public boolean isTransition(T from, T to) {
        return currentState == from && nextState == to;
    }
}