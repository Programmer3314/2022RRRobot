// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/**
 * Differential Drive Train
 */
public class MMDiffDriveTrain {
    double ChassiRadius;
    MMMotorGroup leftMG;
    MMMotorGroup rightMG;
    double revPerFoot;

    /**
     * Differential Drive Train
     * 
     * @param leftMG     left side motor group
     * @param rightMG    right side motor group
     * @param revPerFoot motor revolutions/foot of drivetrain travel
     */
    public MMDiffDriveTrain(MMMotorGroup leftMG, MMMotorGroup rightMG, double revPerFoot, double ChassiRadius) {
        this.leftMG = leftMG;
        this.rightMG = rightMG;
        this.revPerFoot = revPerFoot;
        this.ChassiRadius = ChassiRadius;
    }

    /**
     * Drive
     * 
     * @param speed in feet/sec
     * @param turn  in feet/sec
     */
    public void Drive(double speed, double turn) {
        double DegToFeet = (turn*ChassiRadius*Math.PI)/180;
        double speedRPM = speed * 60 * revPerFoot;
        double turnRPM = turn * 60 * revPerFoot;
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
