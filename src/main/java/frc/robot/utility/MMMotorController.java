// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public abstract class MMMotorController {

    /**
     * Set the Motor Power on a scale from -1 to 1
     * @param power
     */
    public abstract void setPower(double power);

    /**
     * Set Velocity in RPM
     * @param rpm
     */
    public abstract void setVelocity(double rpm);

    /**
     * Get Velocity in RPM
     * @return 
     */
    public abstract double getVelocity();

    /**
     * Get Revolutions
     */
    public abstract double getRevolutions();

    /**
     * Set the lead for this controller to follow.
     * @param lead
     */
    public abstract void follow(MMMotorController lead);

    /**
     * Reset then controller's encoder
     */
    public abstract void resetEncoder();
}

