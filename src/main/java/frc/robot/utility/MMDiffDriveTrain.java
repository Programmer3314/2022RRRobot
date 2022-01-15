// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/**
 * Differential Drive Train
 */
public class MMDiffDriveTrain {
    MMMotorGroup leftMG;
    MMMotorGroup rightMG;
    double revPerFoot;
    double chassisRadius;
    double feetPerDeg;

    /**
     * Differential Drive Train
     * 
     * @param leftMG left side motor group
     * @param rightMG right side motor group
     * @param revPerFoot motor revolutions per linear foot of chassis movement
     * @param chassisRadius radius of robot chassis in Feet
     */
    public MMDiffDriveTrain(MMMotorGroup leftMG, MMMotorGroup rightMG, double revPerFoot, double chassisRadius) {
        this.leftMG = leftMG;
        this.rightMG = rightMG;
        this.revPerFoot = revPerFoot;
        this.chassisRadius = chassisRadius;
        feetPerDeg = (chassisRadius * Math.PI) / 180;
    }

    /**
     * Drive
     * 
     * @param speed in feet/sec
     * @param turn  in degrees/sec
     */
    public void Drive(double speed, double turn) {
        // TODO the calculation for degToFeet has "turn" in it and so does the turnRPM line
        // This is a mistake. "turn" should only be in one of them. 
        // degToFeet should probably just be the conversion and should be calculated once in the 
        // constructor. It's not a lot of fun, but maybe we should add UOM (Units of Measure)
        // to variable names that are used in anything but trivial calculations.
        // degToFeet should maybe be called feetPerDeg (and not include turn) 
        // 60 could be kSecPerMin
        // turn could be turnDegPerSec
        // then we'd have turnRPM = turnDegPerSec * kSecPerMin * revPerFoot * feetPerDeg;
        // or we just need to be more carefull
        double speedRPM = speed * 60 * revPerFoot;
        double turnRPM = turn * 60 * revPerFoot * feetPerDeg;
        leftMG.setVelocity(speedRPM + turnRPM);
        rightMG.setVelocity(speedRPM - turnRPM);
    }

    /**
     * Get (average) Revolutions from each motor group
     * 
     * @return revolutions
     */
    public double getRevolutions() {
        return (leftMG.getRevolutions() + rightMG.getRevolutions()) / 2.0;
    };

    // TODO add a funtion that returns distance in feet (based on getRevolutions)

}
