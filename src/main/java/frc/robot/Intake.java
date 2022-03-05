// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;

/** Add your docs here. */
public class Intake {

    MMMotorGroup intakeMotor;
    DoubleSolenoid intakePosition;

    public Intake() {
        intakeMotor = new MMFollowingMotorGroup(
                new MMFXMotorController(Constants.kCanMCIntake)
                        .setBrakeMode(false));
        intakePosition = Robot.pneumaticHub.makeDoubleSolenoid(
                Constants.kSolenoidIntakeForward,
                Constants.kSolenoidIntakeBackward);
    }

    public void intake() {
        intakePosition.set(Value.kForward);
        intakeMotor.setPower(.5);
    }

    public void eject() {
        intakeMotor.setPower(-.5);
        intakePosition.set(Value.kForward);
    }

    public void idle() {
        intakeMotor.setPower(0);
        intakePosition.set(Value.kReverse);
    }
}
