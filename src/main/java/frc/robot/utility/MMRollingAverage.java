// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public class MMRollingAverage {
    public double[] samples;
    public int pointer;
    public double sum;
    public double average;
    public int count;

    public MMRollingAverage(int sampleCount) {
        count = sampleCount;
        samples = new double[sampleCount];
    }

    public double update(double value) {
        sum -= samples[pointer];
        sum += value;
        samples[pointer] = value;
        pointer = (pointer + 1) % count;
        average = sum / count;
        return average;
    }
}
