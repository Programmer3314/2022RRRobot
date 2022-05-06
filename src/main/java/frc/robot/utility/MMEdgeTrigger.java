// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;


// TODO: rename this to MMBoolean and add additional functionality:
// Add Debouncing - this would ignore "twitchy" rapid transisssions
//  requiring that a value change for some amount of time/cycles so 
//  that inputs such as sensors which might make/break/make/break/make 
//  will not be seen as making or breaking until they stabalize. 
//  This would be helpful in cases like the prox sensors on the climber
//

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
    public boolean value(){
        return currentValue;
    }
}
