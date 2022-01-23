// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public abstract class MMAutonomous<T> extends MMStateMachine<T> {
    public abstract void init();
    public abstract void periodic();
    public MMAutonomous (T initState){
        super(initState);
    }
}
