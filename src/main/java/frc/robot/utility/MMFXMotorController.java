// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import static frc.robot.utility.MMConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * TalonFX Motor Controller built into a Falcon500 motor
 */
public class MMFXMotorController extends MMMotorController {

    public TalonFX mc;

    /**
     * Create a TalonFX Motor Controller built into a Falcon500 motor
     * 
     * @param canid
     */
    public MMFXMotorController(int canid) {
        mc = new TalonFX(canid);
        mc.configFactoryDefault();

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

    public MMFXMotorController setInverted(InvertType invertType) {
        mc.setInverted(invertType);
        return this;
    }

    public MMFXMotorController setPIDFParameters(double p, double i, double d, double f) {
        /* Config the Velocity closed loop gains in slot0 */
        mc.config_kF(0, f, kMMTimeoutMs);
        mc.config_kP(0, p, kMMTimeoutMs);
        mc.config_kI(0, i, kMMTimeoutMs);
        mc.config_kD(0, d, kMMTimeoutMs);
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
        return mc.getSelectedSensorVelocity() * 600 / kMMFalconTicksPerRev;
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

}
