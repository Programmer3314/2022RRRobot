// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;

/**
 * Generic State Machine
 */
public abstract class MMStateMachine<T> {
    public T currentState;
    public T nextState;
    public boolean firstTimeRun;
    public double secondsInState;
    public double cyclesInStates;
    public double startCurrentStateTime;

    public MMStateMachine(T initState) {
        currentState = initState;
        firstTimeRun = true;
        cyclesInStates = 0;
        startCurrentStateTime = Timer.getFPGATimestamp();
    }

    public void update() {
        if (!firstTimeRun) {
            nextState = currentState;
            CalcNextState();
            if (nextState != currentState) {
                doTransition();
                currentState = nextState;
                cyclesInStates = 0;
                startCurrentStateTime = Timer.getFPGATimestamp();
            }
        }
        // TODO should this be the first line of the update method?
        secondsInState = Timer.getFPGATimestamp() - startCurrentStateTime;
        doCurrentState();
        cyclesInStates++;
        firstTimeRun = false;
    }

    public abstract void CalcNextState();

    public abstract void doTransition();

    public abstract void doCurrentState();

    public boolean isTransitionFrom(T... states) {
        for (T state : states) {
            if (state == currentState) {
                return true;
            }
        }
        return false;
    }

    public boolean isTransitionTo(T... states) {
        for (T state : states) {
            if (state == nextState) {
                return true;
            }
        }
        return false;
    }

    public boolean isTransition(T from, T to) {
        return currentState == from && nextState == to;
    }
}