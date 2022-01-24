// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

// TODO Make sure that the intiial state runs on first update
// TODO Add secondsInState and cyclesInState variables
// TODO add non-abstract methods isTransition, isTransitionFrom, isTransitionTo
// these should take these should check From&To, From, To states respectively. 

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