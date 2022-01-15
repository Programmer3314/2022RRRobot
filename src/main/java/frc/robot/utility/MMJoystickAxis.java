// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class MMJoystickAxis {
    Joystick joystick;
    int axis;
    double deadzone;
    double scale;

    public MMJoystickAxis(int joystickIndex, int axisIndex, double deadzone, double scale) {
        this.joystick = new Joystick(joystickIndex);
        this.axis = axisIndex;
        this.deadzone = deadzone;
        this.scale = scale;
    }

    public double get() {
        double rawAxis = joystick.getRawAxis(axis);
        double result;

        if (deadzone <= Math.abs(rawAxis)) {
            double temp = Math.signum(rawAxis) * (Math.abs(rawAxis) - deadzone) / (1 - deadzone);
            result = temp * scale;
        } else {
            result = 0;
        }
        return result;
    }
} 
