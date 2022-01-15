// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class MMJoystickAxis {
    Joystick joystick;
    int Axis;
    double Deadzone;
    double scale;
    public MMJoystickAxis(int JoystickIndx, int AxisIndx, double deadzone, double Scale){
        this.joystick = new Joystick(JoystickIndx);
        this.Axis = AxisIndx;
        this.Deadzone = deadzone;
        this.scale = Scale;
    }
    public double get(){
        double rawAxis = joystick.getRawAxis(Axis);
        double result;
        if (Deadzone<=Math.abs(rawAxis)){
            double temp = Math.signum(rawAxis)*(Math.abs(rawAxis)-Deadzone)/(1-Deadzone);
            result = temp*scale;
        }else{
            result = 0;;
        }
        return result;
    }
}
