// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMSRXMotorController;

/** Add your docs here. */
public class Intake {

    MMMotorGroup intakeMotor;
    DoubleSolenoid intakePosition;

    public Intake() {
        intakeMotor = new MMFollowingMotorGroup(new MMSRXMotorController(20));
        intakePosition = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 2);
    }

    public void intake() {
        intakePosition.set(Value.kForward);
        intakeMotor.setPower(1);
    }

    public void eject() {
        intakeMotor.setPower(-1);
        intakePosition.set(Value.kForward);
    }

    public void idle() {
        intakeMotor.setPower(0);
        intakePosition.set(Value.kReverse);
    }
}
