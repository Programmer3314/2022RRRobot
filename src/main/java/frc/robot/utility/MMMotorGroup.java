// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/**
 * Motor Group consisting of one feedback device
 * and a group of motor.
 */
public abstract class MMMotorGroup {

    /**
     * Set motor power ranging from -1 to 1
     * 
     * @param power
     */
    public abstract void setPower(double power);

    /**
     * Set the motor velocity in RPM
     * 
     * @param rpm
     */
    public abstract void setVelocity(double rpm);

    /**
     * Set Position in Revolutions
     * 
     * 
     * @param position
     */
    public abstract void setPosition(double position);

    /**
     * Get the Revolutions since last Reset
     * 
     * @return
     */
    public abstract double getRevolutions();

    /**
     * Reset Encoders
     */
    public abstract void resetEncoders();

    public abstract void setEncoder(double ticks);

    /**
     * Get velocity in RPM
     * 
     * @return
     */
    public abstract double getVelocity();

    public abstract void setEncoderRevolutions(double revs);

    public abstract double getCurrent();
}

