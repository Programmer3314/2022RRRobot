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

    /**
     * Differential Drive Train
     * 
     * @param leftMG
     * @param rightMG
     * @param revPerFoot
     * @param chassisRadius
     */
    public MMDiffDriveTrain(MMMotorGroup leftMG, MMMotorGroup rightMG, double revPerFoot, double chassisRadius) {
        this.leftMG = leftMG;
        this.rightMG = rightMG;
        this.revPerFoot = revPerFoot;
        this.chassisRadius = chassisRadius;
    }

    /**
     * Drive
     * 
     * @param speed in feet/sec
     * @param turn  in feet/sec
     */
    public void Drive(double speed, double turn) {
        double degToFeet = (turn * chassisRadius * Math.PI) / 180;
        double speedRPM = speed * 60 * revPerFoot;
        double turnRPM = turn * 60 * revPerFoot * degToFeet;
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
