// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */

public class MMBCDReturn {

    int dialButton1, dialButton2, dialButton3, dialButton4;
    Joystick bb;

    boolean bit1, bit2, bit3, bit4;

    public MMBCDReturn(Joystick ButtonBox, int button1, int button2, int button3, int button4) {
        bb = ButtonBox;
        dialButton1 = button1;
        dialButton2 = button2;
        dialButton3 = button3;
        dialButton4 = button4;

    }

    public int GetDial() {
        // 13, 14, 15,16
        int returnedBinary = 0;
        bit1 = dialButton1 > 0 && bb.getRawButton(dialButton1);
        bit2 = dialButton2 > 0 && bb.getRawButton(dialButton2);
        bit3 = dialButton3 > 0 && bb.getRawButton(dialButton3);
        bit4 = dialButton4 > 0 && bb.getRawButton(dialButton4);

        if (bit1) {
            returnedBinary += 1;
        }
        if (bit2) {
            returnedBinary += 2;
        }
        if (bit3) {
            returnedBinary += 4;
        }
        if (bit4) {
            returnedBinary += 8;
        }

        return returnedBinary;
    }
}
