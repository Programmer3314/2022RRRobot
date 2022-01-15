// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;

/**
 * Motor group in which all motors follow the first
 */
public class MMFollowingMotorGroup extends MMMotorGroup {

    ArrayList<MMMotorController> motors = new ArrayList<>();
    MMMotorController lead;
    MMMotorController enc;

    /**
     * Create a motor group in which all motors follow the first.
     * Fully spec the lead motor, others can generally be minimally spec'd.
     * 
     * @param motors list of motors
     */
    public MMFollowingMotorGroup(MMMotorController... motors) {
        boolean first = true;
        for (MMMotorController m : motors) {
            this.motors.add(m);
            if (first) {
                lead = m;
                enc = m;
            } else {
                m.follow(lead);
            }
            first = false;
        }
    }

    @Override
    public void setPower(double power) {
        lead.setPower(power);
    }

    @Override
    public void setVelocity(double rpm) {
        lead.setVelocity(rpm);
    }

    @Override
    public double getRevolutions() {
        return enc.getRevolutions();
    }

    @Override
    public void resetEncoders() {
        enc.resetEncoder();
    }

    @Override
    public double getVelocity() {
        return enc.getVelocity();
    }

    @Override
    public void setPosition(double position) {
        lead.setPosition(position);
        
    }
}
