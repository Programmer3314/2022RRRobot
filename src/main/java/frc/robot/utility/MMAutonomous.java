// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public abstract class MMAutonomous<T> extends MMStateMachine<T> {

    public MMAutonomous (T initState){
        super(initState);
    }

    public abstract void init();
    public abstract void periodic();
    public abstract void LogHeader();
    public abstract void LogData();
}
