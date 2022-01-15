// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class TargetPoint {
    public double distance;
    public double angle;
    public double rpm;

    /**
     * Creating a target point w/ values
     * @param distance in feet
     * @param rpm in revolutions per minutes
     * @param angle in revolutions/rotations
     */
    public TargetPoint(double distance, double rpm, double angle){
        this.distance = distance;
        this.rpm = rpm;
        this.angle = angle;
    }
}
