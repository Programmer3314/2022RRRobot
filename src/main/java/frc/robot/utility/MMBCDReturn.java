// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import frc.robot.Robot;

/** Add your docs here. */


public class MMBCDReturn {

    boolean bit1, bit2, bit3, bit4;
    public MMBCDReturn(){
  
    }
    public int GetDial(){
        //13, 14, 15,16
        int returnedBinary = 0;
        bit1 = Robot.buttonBox1.getRawButton(13);
        bit2 = Robot.buttonBox1.getRawButton(14);
        bit3 = Robot.buttonBox1.getRawButton(15);
        bit4 = Robot.buttonBox1.getRawButton(16);

        if(bit1){
            returnedBinary+=1;
        }
        if(bit2){
            returnedBinary+=2;
        }
        if(bit3){
            returnedBinary+=4;
        }
        if(bit4){
            returnedBinary+=8;
        }

        return returnedBinary;
    }
}
