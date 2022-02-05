// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Expected Hardware configuration:
 * Deflection Sensor (1 or 2) on lower hooks
 * Contact Sensors on top hooks (proximity or wobble)
 * Maybe: Pocketted Sensors on top hooks
 * Single motor (implement motor group anyway) 
 * Existing motor encoder(s)
 * Existing IMU in NavX
 */

// TODO Add a Pneumatic to shift the climber when extending the upper hooks
// I've been told that there will be a pneumatic to shif the angle of the lift and the robot.
// It's expected that the climber will start out leaning back and will need to be pushed forward 
// durring "homing".  
// TODO Add second motor (irl) and set it inverted to make sure that the inverted motor runs opposite
// to the lead.
// TODO Comment out the setVelocity calls and replace (for now at least) with set Power. 
// We will probably want the velocity calls back in, but not knowing when we'll be tuning the lift,
// I want to be safe by using power. Distance is critical, speed is not critical.  

/**
 * Steps
 * 1) Start
 * 2) Home its motors so its in the right position
 * 3) Idle
 *  -----Button Hit-------------raise lead hooks------
 * 4) First Extend- extends lead hook until it is fully extended
 *  -----Button Hit--------start climb-------
 * 4.5) Move into bar
 * 5) First Pullup - Lead hook pulls robot up, thus bringing the stationary hooks up
 * 6) Check - Check the stationary hook moved out and back on bar; check for secure grip on bar
 * 7) Second Extend- extends lead hook after a said amount of encoders, make sure the lead hook overshoots 
 * the second bar
 * 8) Second Pullup - Lead hook pulls the robot up, which will result it in swinging on the second bar
 * 9) Check - Check the stationary hook moved out and back on bar; check for secure grip on bar
 * 10) Third Extend Part One - extend the lead hook a little bit due to the robot swinging a lot, wait 
 * until robot swing calms down
 * 11) Third Extend Part Two - extend lead hook until it overshoots the third bar, which should be 
 * under the third bar
 * 12) Third Pullup - Lead hook pulls the robot until the stationary hook is off the second bar
 * 
 * 
 */

package frc.robot;

import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMStateMachine;

enum ClimbStates {
    Start, Home, Idle, ExtendToBar1, DriveToBar1, PullBar1ToStatHooks, PullBar1PastStatHooks,
    ExtendToBar2, ExtendToBar2Check, PullBar2ToStatHooks, PullBar2PastStatHooks, ExtendBar3Swing, WaitForCalm,
    ExtendBar3Calm, ExtendToBar3Check, PullupBar3, Done
}

/** Add your docs here. */
public class ClimbStateMachine extends MMStateMachine<ClimbStates> {

    boolean lowLimitSwitch;
    boolean highLimitSwitch;
    boolean statHookDeflectionLeft;// true if not deflected
    boolean statHookDeflectionRight;// true if not deflected
    boolean leadHookContactLeft;
    boolean leadHookContactRight;
    boolean leadHookPocketLeft;
    boolean leadHookPocketRight;
    MMMotorGroup climbMotor;
    boolean raiseLeadHooks;
    boolean startClimb;
    double revolutionsToBar1 = 10;
    double revolutionsToBar2 = 10;
    double revolutionsNearBar3 = 5;
    double revolutionsToBar3 = 10;
    double revolutionsToBar3Final = 7;
    boolean navxCalm = true;
    double rpmForBarExtend = 100;
    double rpmForBarPull = -200;
    double rpmForPullPast = -150;
    double homePower = -0.15;

    public ClimbStateMachine() {
        super(ClimbStates.Start);
        climbMotor = new MMFollowingMotorGroup(
                new MMFXMotorController(30)
                        .setPIDFParameters(.15, 0, 1, 0));
    }

    @Override
    public void update() {
        // TODO Read Sensor Values
        leadHookContactLeft = Robot.buttonBox1.getRawButton(2);
        leadHookContactRight = Robot.buttonBox1.getRawButton(3);
        statHookDeflectionLeft = !Robot.buttonBox1.getRawButton(5);
        statHookDeflectionRight = !Robot.buttonBox1.getRawButton(6);
        lowLimitSwitch = Robot.buttonBox1.getRawButton(9);
        raiseLeadHooks = Robot.buttonBox1.getRawButton(4);
        startClimb = Robot.buttonBox1.getRawButton(7);

        super.update();
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = ClimbStates.Home;
                break;
            case Home:
                if (lowLimitSwitch) {
                    nextState = ClimbStates.Idle;
                }
                break;
            case Idle:
                if (raiseLeadHooks) {
                    nextState = ClimbStates.ExtendToBar1;
                }
                break;
            case ExtendToBar1:
                if (climbMotor.getRevolutions() >= revolutionsToBar1) {
                    nextState = ClimbStates.DriveToBar1;
                }
                break;
            case DriveToBar1:
                if (leadHookContactLeft && leadHookContactRight && startClimb) {
                    nextState = ClimbStates.PullBar1ToStatHooks;
                }
                break;
            case PullBar1ToStatHooks:
                if (!statHookDeflectionLeft || !statHookDeflectionRight) {
                    nextState = ClimbStates.PullBar1PastStatHooks;
                }
                break;
            case PullBar1PastStatHooks:
                if (statHookDeflectionLeft && statHookDeflectionRight) {
                    nextState = ClimbStates.ExtendToBar2;
                }
                break;
            case ExtendToBar2:
                if (climbMotor.getRevolutions() >= revolutionsToBar2) {
                    nextState = ClimbStates.ExtendToBar2Check;
                }
                break;
            case ExtendToBar2Check:
                if (leadHookContactLeft && leadHookContactRight) {
                    nextState = ClimbStates.PullBar2ToStatHooks;
                }
                break;
            case PullBar2ToStatHooks:
                if (!statHookDeflectionLeft || !statHookDeflectionRight) {
                    nextState = ClimbStates.PullBar2PastStatHooks;
                }
                break;
            case PullBar2PastStatHooks:
                if (statHookDeflectionLeft && statHookDeflectionRight) {
                    nextState = ClimbStates.ExtendBar3Swing;
                }
                break;
            case ExtendBar3Swing:
                if (climbMotor.getRevolutions() >= revolutionsNearBar3) {
                    nextState = ClimbStates.WaitForCalm;
                }
                break;
            case WaitForCalm:
                if (navxCalm) {
                    nextState = ClimbStates.ExtendBar3Calm;
                }
                break;
            case ExtendBar3Calm:
                if (climbMotor.getRevolutions() >= revolutionsToBar3) {
                    nextState = ClimbStates.ExtendToBar3Check;
                }
                break;
            case ExtendToBar3Check:
                if (leadHookContactRight && leadHookContactLeft) {
                    nextState = ClimbStates.PullupBar3;
                }
                break;
            case PullupBar3:
                if (climbMotor.getRevolutions() <= revolutionsToBar3Final) {
                    nextState = ClimbStates.Done;
                }
                break;
            case Done:
                break;
        }

    }

    @Override
    public void doTransition() {
        if (isTransitionTo(ClimbStates.Home)) {
            climbMotor.setPower(homePower);
        }
        if (isTransitionFrom(ClimbStates.Home)) {
            climbMotor.setPower(0);
            climbMotor.resetEncoders();
        }
        if (isTransitionTo(ClimbStates.ExtendToBar1)) {
            climbMotor.setVelocity(rpmForBarExtend);
        }
        if (isTransitionFrom(ClimbStates.ExtendToBar1)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.PullBar1ToStatHooks)) {
            climbMotor.setVelocity(rpmForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullBar1PastStatHooks)) {
            climbMotor.setVelocity(rpmForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendToBar2)) {
            climbMotor.setVelocity(rpmForBarExtend);
        }
        if (isTransitionFrom(ClimbStates.ExtendToBar2)) {
            climbMotor.setVelocity(0);
        }
        if (isTransitionTo(ClimbStates.PullBar2ToStatHooks)) {
            climbMotor.setVelocity(rpmForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullBar2PastStatHooks)) {
            climbMotor.setVelocity(rpmForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendBar3Swing)) {
            climbMotor.setVelocity(rpmForBarExtend);
        }
        if (isTransitionFrom(ClimbStates.ExtendBar3Swing)) {
            climbMotor.setVelocity(0);
        }
        if (isTransitionTo(ClimbStates.ExtendBar3Calm)) {
            climbMotor.setVelocity(rpmForBarExtend);
        }
        if (isTransitionFrom(ClimbStates.ExtendBar3Calm)) {
            climbMotor.setVelocity(0);
        }
        if (isTransitionTo(ClimbStates.PullupBar3)) {
            climbMotor.setVelocity(rpmForBarPull);
        }
        if (isTransitionFrom(ClimbStates.PullupBar3)) {
            climbMotor.setVelocity(0);
        }
    }

    @Override
    public void doCurrentState() {
    }

}
