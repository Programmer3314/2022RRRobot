// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.utility.MMConstants.*;

/** Add your docs here. */
public class MMSRXMotorController extends MMMotorController {

    public TalonSRX mc;
    double encoderTicksPerRev = 1;

    public MMSRXMotorController(int canid) {
        mc = new TalonSRX(canid);
        mc.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        mc.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kMMTimeoutMs);

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

    public MMSRXMotorController setTicksPerRev(double ticksPerRev) {
        this.encoderTicksPerRev = ticksPerRev;
        return this;
    }

    public MMSRXMotorController setInverted(InvertType invertType) {
        mc.setInverted(invertType);
        return this;
    }

    public MMSRXMotorController setPIDFParameters(double p, double i, double d, double f) {
        /* Config the Velocity closed loop gains in slot0 */
        mc.config_kF(0, f, kMMTimeoutMs);
        mc.config_kP(0, p, kMMTimeoutMs);
        mc.config_kI(0, i, kMMTimeoutMs);
        mc.config_kD(0, d, kMMTimeoutMs);
        return this;
    }

    @Override
    public void setPower(double power) {
        mc.set(TalonSRXControlMode.PercentOutput, power);
    }

    @Override
    public void setVelocity(double rpm) {
        double velocity = rpm / 600.0 * encoderTicksPerRev;
        mc.set(TalonSRXControlMode.Velocity, velocity);
    }

    @Override
    public double getVelocity() {
        return mc.getSelectedSensorVelocity() * 600.0 / encoderTicksPerRev;
    }

    @Override
    public void follow(MMMotorController lead) {
        mc.follow(((MMSRXMotorController) lead).mc);
    }

    @Override
    public double getRevolutions() {
        return mc.getSelectedSensorPosition() / encoderTicksPerRev;
    }

    @Override
    public void resetEncoder() {
        mc.setSelectedSensorPosition(0);
    }

    @Override
    public void setPosition(double position) {
        double ticks = encoderTicksPerRev*position;
        mc.set(TalonSRXControlMode.Position, ticks);
        
    }

}
