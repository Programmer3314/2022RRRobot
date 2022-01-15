// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

//import static frc.robot.Constant.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class MMSparkMaxMotorController extends MMMotorController {
    public CANSparkMax mc;
    public SparkMaxPIDController pid;
    public RelativeEncoder enc;

    public MMSparkMaxMotorController(int canid, MotorType type) {
        mc = new CANSparkMax(canid, type);
        mc.restoreFactoryDefaults();
        pid = mc.getPIDController();
        enc = mc.getEncoder();
    }

    public MMSparkMaxMotorController setInverted(Boolean inverted) {
        mc.setInverted(inverted);
        return this;
    }

    public MMSparkMaxMotorController setPIDFParameters(double p, double i, double d, double f, double iz, double min,
            double max) {
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setFF(f);
        pid.setIZone(iz);
        pid.setOutputRange(min, max);
        return this;
    }

    public MMSparkMaxMotorController setCurrentLimit(int stallLimit, int freeLimit) {
        mc.setSmartCurrentLimit(stallLimit, freeLimit);
        return this;
    }

    @Override
    public void setPower(double power) {
        mc.set(power);
    }

    @Override
    public void setVelocity(double rpm) {
        pid.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public double getVelocity() {
        return enc.getVelocity();
    }

    @Override
    public void follow(MMMotorController lead) {
        mc.follow(((MMSparkMaxMotorController) lead).mc);
    }

    @Override
    public double getRevolutions() {
        return enc.getPosition();
    }

    @Override
    public void resetEncoder() {
        enc.setPosition(0);
    }
}
