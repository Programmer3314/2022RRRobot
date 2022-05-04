// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public class MMEdgeTrigger {
    boolean lastValue;
    boolean currentValue;

    public MMEdgeTrigger update(boolean value){
        lastValue= currentValue;
        currentValue = value;
        return this;
    }
    public boolean transitionHigh(){
        return lastValue ==false && currentValue == true;
    }
    public boolean transitionLow(){
        return lastValue == true && currentValue == false;
    }
}
