// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class TargetSamples {

    public double[] targetSamples = new double[20];
    public int counter;
    public double sum;
    public double average;

    public TargetSamples() {
        counter = 0;
    }

    public double update(double newHValue) {
        sum -= targetSamples[counter];
        targetSamples[counter] = newHValue;
        counter = (counter + 1) % 20;
        sum += newHValue;
        average = sum / 20.0;
        return average;
    }
}
