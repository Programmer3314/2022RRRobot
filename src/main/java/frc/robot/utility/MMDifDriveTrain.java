// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public class MMDifDriveTrain {
    MMMotorGroup leftMG;
    MMMotorGroup rightMG;
    double revPerFoot;

    public MMDifDriveTrain(MMMotorGroup leftMG, MMMotorGroup rightMG, double revPerFoot){
        this.leftMG = leftMG;
        this.rightMG = rightMG;
        this.revPerFoot = revPerFoot;
    }

    public void Drive(double speed, double turn){
        
        double speedRPM = speed*60*revPerFoot;
        double turnRPM = turn*60*revPerFoot;
        leftMG.setVelocity(speedRPM+turnRPM);
        rightMG.setVelocity(speedRPM-turnRPM);
        
    } 

    

}
