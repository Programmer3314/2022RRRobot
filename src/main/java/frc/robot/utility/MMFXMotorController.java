// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import static frc.robot.utility.MMConstants.kMMFalconTicksPerRev;
import static frc.robot.utility.MMConstants.kMMTimeoutMs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * TalonFX Motor Controller built into a Falcon500 motor
 */
public class MMFXMotorController extends MMMotorController {

    public TalonFX mc;
    // public TalonFXConfiguration config;

    /**
     * Create a TalonFX Motor Controller built into a Falcon500 motor
     * 
     * @param canid
     */
    public MMFXMotorController(int canid) {
        mc = new TalonFX(canid);
        mc.configFactoryDefault();
        // mc.getAllConfigs(config, 100);

        /**
         * Phase sensor accordingly.
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
        mc.setSensorPhase(true);

        /* Config the peak and nominal outputs */
        mc.configNominalOutputForward(0, kMMTimeoutMs);
        mc.configNominalOutputReverse(0, kMMTimeoutMs);
        mc.configPeakOutputForward(1, kMMTimeoutMs);
        mc.configPeakOutputReverse(-1, kMMTimeoutMs);

    }
    public MMFXMotorController setNominalOutput(double forward, double reverse){
        mc.configNominalOutputForward(forward, kMMTimeoutMs);
        mc.configNominalOutputReverse(reverse, kMMTimeoutMs);
        return this;
    }

    public MMFXMotorController setPeakOutput(double forward, double reverse){
        mc.configPeakOutputForward(forward, kMMTimeoutMs);
        mc.configPeakOutputReverse(reverse, kMMTimeoutMs);
        return this;
    }

    public MMFXMotorController setInverted(InvertType invertType) {
        mc.setInverted(invertType);
        return this;
    }

    
    public MMFXMotorController setPIDFParameters(double p, double i, double d, double f) {
        setPIDFParameters(p, i, d, f, 0);
        return this;
    }

    public MMFXMotorController setPIDFParameters(double p, double i, double d, double f, double izone) {
        /* Config the Velocity closed loop gains in slot0 */
        mc.config_kF(0, f, kMMTimeoutMs);
        mc.config_kP(0, p, kMMTimeoutMs);
        mc.config_kI(0, i, kMMTimeoutMs);
        mc.config_kD(0, d, kMMTimeoutMs);
        mc.config_IntegralZone(0, izone, kMMTimeoutMs);
        return this;
    }

    @Override
    public void setPower(double power) {
        mc.set(TalonFXControlMode.PercentOutput, power);
    }

    @Override
    public void setVelocity(double rpm) {
        // controller specific RPM to velocity conversion
        double velocity = rpm / 600 * kMMFalconTicksPerRev;
        mc.set(TalonFXControlMode.Velocity, velocity);
    }

    @Override
    public double getVelocity() {

        return mc.getSelectedSensorVelocity() * 600.0 / kMMFalconTicksPerRev;
    }

    @Override
    public void follow(MMMotorController lead) {
        mc.follow(((MMFXMotorController) lead).mc);
    }

    @Override
    public double getRevolutions() {
        return mc.getSelectedSensorPosition() / kMMFalconTicksPerRev;
    }

    @Override
    public void resetEncoder() {
        mc.setSelectedSensorPosition(0);
    }

    @Override
    public void setPosition(double positionRevs) {
        double ticks = kMMFalconTicksPerRev * positionRevs;
        mc.set(ControlMode.Position, ticks);

    }

    @Override
    public void setEncoder(double ticks) {
        mc.setSelectedSensorPosition(ticks);
    }

    public void setEncoderRevolution(double revs) {
        setEncoder(revs * kMMFalconTicksPerRev);
    }

    public MMFXMotorController setBrakeMode(boolean brakeMode) {
        if (brakeMode) {
            mc.setNeutralMode(NeutralMode.Brake);
        } else {
            mc.setNeutralMode(NeutralMode.Coast);
        }
        return this;
    }

    @Override
    public void setEncoderRevolutions(double revs) {
        setEncoder(revs * kMMFalconTicksPerRev);
    }

    @Override
    public double getCurrent() {
        return mc.getSupplyCurrent();
    }

    public MMFXMotorController setStatorCurrentLimit(boolean enabled, double currentAmpLimit,
            double triggerAmpThreshold, double triggerThresholdSeconds) {
        mc.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(enabled, currentAmpLimit, triggerAmpThreshold,
                triggerThresholdSeconds));
        return this;
    }

    public MMFXMotorController setSupplyCurrentLimit(boolean enabled, double currentAmpLimit,
            double triggerAmpThreshold, double triggerThresholdSeconds) {
        mc.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enabled, currentAmpLimit, triggerAmpThreshold,
                triggerThresholdSeconds));
        return this;
    }

    public MMFXMotorController setNominal(double forward, double reverse){
        mc.configNominalOutputForward(forward, kMMTimeoutMs);
        mc.configNominalOutputReverse(reverse, kMMTimeoutMs);
        return this;
    }

}
