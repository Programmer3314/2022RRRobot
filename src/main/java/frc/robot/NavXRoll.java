// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class NavXRoll {
    double lastRoll;
    int downcount;
    double centerBarAng = 60;
    int cyclesAbove = 10;
    double[] samples = new double[25];
    int pointer;
    double minsample;
    double maxsample;
    boolean calm;
    double margin = 1;
    int downCountTwo;

    public NavXRoll() {
        pointer = 0;
    }

    public void update(double roll) {
        if (roll > lastRoll && roll < centerBarAng) {
            downcount++;
        } else {
            downcount = 0;
        }
        if (roll > lastRoll) {
            downCountTwo++;
        } else {
            downCountTwo = 0;
        }
        samples[pointer]= roll;
        minsample = roll;
        maxsample = roll;
        for(double sample:samples){
            if(sample<minsample){
                minsample = sample;
            }
            if(sample>maxsample){
                maxsample = sample;
            }
        }
        lastRoll = roll;
        pointer=(pointer+1)%samples.length;
        smartdashboards(roll);
    }

    public boolean isApproachingBar() {
        return downcount > cyclesAbove;
    }
    public boolean isGoingDown(){
        return downCountTwo > cyclesAbove;
    }
    public boolean isBelowBar(double centerBarAngle){
        return lastRoll>centerBarAngle;
    }
    public boolean navxCalm(){
        return Math.abs(minsample -maxsample)<margin;
    }
    public void smartdashboards(double roll){
        SmartDashboard.putNumber("NavXRoll", roll);
        SmartDashboard.putBoolean("isApproachingBar:", isApproachingBar());
        SmartDashboard.putBoolean("isBelowBar(60): ", isBelowBar(60));
        SmartDashboard.putBoolean("isBelowBar(57): ", isBelowBar(60));
        SmartDashboard.putBoolean("NavXCalm: ", navxCalm());
        SmartDashboard.putNumber("MinValue", minsample);
        SmartDashboard.putNumber("MaxValue", maxsample);

    }
    public void LogHeader(){
        Logger.Header("ApproachBar, BelowBar60, BelowBar57 ,NavXCalm, "
        +"minValue, maxValue,");
    }
    public void LogData(){
        Logger.booleans(isApproachingBar(), isBelowBar(60), isBelowBar(57), navxCalm());
        Logger.doubles(minsample, maxsample);
    }

}