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

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMFXMotorController;
import frc.robot.utility.MMFollowingMotorGroup;
import frc.robot.utility.MMMotorGroup;
import frc.robot.utility.MMStateMachine;

enum ClimbStates {
    Start, Home, Idle, ExtendToBar1fast, ExtendToBar1slow, DriveToBar1,
    PullBar1ToStatHooksC, PullBar1PastStatHooksC, ExtendtoHookA, ExtendPastA, PulltoHookB, PullPastHookB,
    Pause1, ExtendToBar2, ExtendToBar2Check, PullBar2ToStatHooksC, PullBar2PastStatHooksC, ExtendtoHookA2, ExtendPastA2,
    PulltoHookB2, PullPastHookB2,
    ExtendBar3Swing, WaitForCalm, ExtendBar3Calm, ExtendToBar3Check, PullupBar3, Done, Pause, Manual
}

/** Add your docs here. */
public class ClimbStateMachine extends MMStateMachine<ClimbStates> {

    boolean lowLimitSwitch;
    boolean highLimitSwitch;
    boolean leadHookContactLeft;
    boolean leadHookContactRight;
    boolean leadHookPocketLeft;
    boolean leadHookPocketRight;
    MMMotorGroup climbMotor;
    boolean raiseLeadHooks;
    boolean lowerLeadHooks;
    boolean startClimb;
    double revolutionsToBar1 = 74;
    double revolutionsToBar1slow = 65;
    double revolutionsToBar2 = 70;
    double revolutionsNearBar3 = 60;
    double revolutionsToBar3 = 70;
    double revolutionsToBar3Final = 7;
    boolean navxCalm = true;
    double rpmForBarExtend = 100;
    double pwrForBarExtend = .2;
    double pwrForBarExtendslow = .1;
    double rpmForBarPull = -200;
    double pwrForBarPull = -.4;
    double rpmForPullPast = -150;
    double pwrForPullPast = -.5;
    double homePower = -0.15;
    // DoubleSolenoid climberPosition;
    ClimbStates pauseState;
    boolean manualMoveClimberUp;
    boolean manualMoveClimberDown;
    double manualClimbPosition;
    boolean manualHome;
    boolean climberHomed;
    DigitalInput climblimit;
    boolean climbbool;
    boolean prevLimitSwitch;
    DigitalInput leadcontactLeft;
    DigitalInput leadcontactRight;
    DigitalInput deflectionCRed;
    DigitalInput deflectionCWhite;
    DigitalInput deflectionAred;
    boolean deflectionA;
    DigitalInput deflectionBRed;
    DigitalInput deflectionBWhite;
    boolean deflectionRedB;
    boolean deflectionWhiteB;
    boolean deflectionWhiteC;
    boolean deflectionRedC;
    boolean overrideLeadHooks;
    double climbMotorRevs;
    double climbMotorRPM;

    /*
     * go up to 72 revs
     * bring top hook between hooks 1 and 2
     * 72
     * Hook 1
     * 72
     * (lower)Hook 1 =
     * Hook 2 =
     */
    public ClimbStateMachine() {
        super(ClimbStates.Start);
        climbMotor = new MMFollowingMotorGroup(// TODO 73 is our highest point
                new MMFXMotorController(Constants.kCanMCClimber1)
                        .setPIDFParameters(Constants.kfalconClimbKP, Constants.kfalconClimbKI,
                                Constants.kfalconClimbKD, Constants.kfalconClimbKFF)
                        .setBrakeMode(true)
                        .setInverted(Constants.kClimberInvert),
                new MMFXMotorController(Constants.kCanMCClimber2)
                        .setPIDFParameters(Constants.kfalconClimbKP, Constants.kfalconClimbKI,
                                Constants.kfalconClimbKD, Constants.kfalconClimbKFF)
                        .setBrakeMode(true)
                        .setInverted(InvertType.OpposeMaster));
        leadcontactLeft = new DigitalInput(Constants.kDIOBarWhite);
        leadcontactRight = new DigitalInput(Constants.kNAVXBarRed);
        deflectionCRed = new DigitalInput(Constants.kDIOCRed);
        deflectionCWhite = new DigitalInput(Constants.kNAVXCWhite);
        deflectionAred = new DigitalInput(Constants.kDIOARed);
        deflectionBRed = new DigitalInput(Constants.kDIOBRed);
        deflectionBWhite = new DigitalInput(Constants.kNAVXBWhite);

        climblimit = new DigitalInput(Constants.kDIOClimbLimit);
        // climberPosition = new DoubleSolenoid(Constants.kSolenoidModule,
        // PneumaticsModuleType.CTREPCM,
        // Constants.kSolenoidClimberBackward, Constants.kSolenoidClimberForward);
        climberHomed = false;

    }

    @Override
    public void update() {
        // Manual buttons to move climber
        // manualMoveClimberUp = Robot.buttonBox1.getRawButtonPressed(2);
        // manualMoveClimberDown = Robot.buttonBox1.getRawButtonPressed(5);

        climbMotorRevs = climbMotor.getRevolutions();
        climbMotorRPM = climbMotor.getVelocity();

        manualHome = Robot.buttonBox1.getRawButton(Constants.kButtonBoxManualHome);

        leadHookContactLeft = !leadcontactLeft.get();
        leadHookContactRight = !leadcontactRight.get();

        lowLimitSwitch = !climblimit.get();

        deflectionA = deflectionAred.get();
        deflectionWhiteB = deflectionBWhite.get();
        deflectionRedB = deflectionBRed.get();
        deflectionWhiteC = deflectionCWhite.get();
        deflectionRedC = deflectionCRed.get();

        overrideLeadHooks = Robot.buttonBox1.getRawButton(5);

        // TODO: double check this code for errors
        raiseLeadHooks = Robot.controllerOperator.getRawButton(Constants.kOperatorRaiseHooks);
        lowerLeadHooks = Robot.controllerOperator.getRawButton(Constants.kOperatorLowerHooksButton);
        startClimb = Robot.controllerOperator.getRawButton(Constants.kOperatorClimbButton);

        super.update();

        SmartDashboard.putNumber("Desired Climber Position: ", manualClimbPosition);
        SmartDashboard.putNumber("Returned Climber Position: ", climbMotor.getRevolutions());
        // SmartDashboard.putBoolean("Climber UP Value: ", manualMoveClimberUp);
        // SmartDashboard.putBoolean("Climber UP Value: ", manualMoveClimberDown);
        SmartDashboard.putBoolean("Climber Limit: ", lowLimitSwitch);
        SmartDashboard.putBoolean("RaiseLeadHooks : ", raiseLeadHooks);
        SmartDashboard.putBoolean("LowerLeadHooks : ", lowerLeadHooks);
        SmartDashboard.putBoolean("Lead Contact Left: ", leadHookContactLeft);
        SmartDashboard.putBoolean("Lead Contact Right: ", leadHookContactRight);
        SmartDashboard.putBoolean("Deflection A", deflectionA);
        SmartDashboard.putBoolean("Deflection White B", deflectionWhiteB);
        SmartDashboard.putBoolean("Deflection Red B", deflectionRedB);
        SmartDashboard.putBoolean("Deflection White C", deflectionWhiteC);
        SmartDashboard.putBoolean("Deflection Red C", deflectionRedC);
        SmartDashboard.putBoolean("Deflection Red B", deflectionRedB);
        SmartDashboard.putString("Climb State: ", currentState.toString());
        SmartDashboard.putBoolean("Manual Lead Hook Override", overrideLeadHooks);
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
            case Manual:
            case Home:
            case Idle:
            case ExtendToBar1fast:
            case ExtendToBar1slow:
            case DriveToBar1:
            case Done:
            case Pause:
                break;
            default:
                if (!startClimb) {
                    pauseState = currentState;
                    nextState = ClimbStates.Pause;
                    return;
                }
                break;
        }
        switch (currentState) {
            case Start:
                nextState = ClimbStates.Home;
                // nextState = ClimbStates.Manual;
                break;
            case Home:
                if (lowLimitSwitch) {
                    nextState = ClimbStates.Idle;
                }
                break;
            case Idle:
                if (raiseLeadHooks) {
                    nextState = ClimbStates.ExtendToBar1fast;
                }
                break;
            case ExtendToBar1fast:
                if (climbMotorRevs >= revolutionsToBar1slow) {
                    nextState = ClimbStates.ExtendToBar1slow;
                    // nextState = ClimbStates.DriveToBar1;
                    // nextState = ClimbStates.Home;
                }
                if (lowerLeadHooks) {
                    nextState = ClimbStates.Home;
                }
                break;
            case ExtendToBar1slow:
                if (climbMotorRevs >= revolutionsToBar1) {
                    nextState = ClimbStates.DriveToBar1;
                    // nextState = ClimbStates.DriveToBar1;
                    // nextState = ClimbStates.Home;
                }
                if (lowerLeadHooks) {
                    nextState = ClimbStates.Home;
                }
                break;
            case DriveToBar1:
                if (((leadHookContactLeft && leadHookContactRight) || overrideLeadHooks) && startClimb) {
                    nextState = ClimbStates.PullBar1ToStatHooksC;
                }
                if (lowerLeadHooks) {
                    nextState = ClimbStates.Home;
                }
                break;
            case PullBar1ToStatHooksC:
                if (deflectionRedC && deflectionWhiteC) {
                    nextState = ClimbStates.PullBar1PastStatHooksC;
                }
                break;
            case PullBar1PastStatHooksC:
                if (!deflectionRedC && !deflectionWhiteC) {
                    // nextState = ClimbStates.ExtendToBar2;
                    nextState = ClimbStates.ExtendtoHookA;
                }
                break;

            case ExtendtoHookA:
                if (deflectionA) {
                    nextState = ClimbStates.ExtendPastA;
                }
                break;
            case ExtendPastA:
                if (!deflectionA) {
                    nextState = ClimbStates.PulltoHookB;
                }
                break;
            case PulltoHookB:
                if (deflectionWhiteB && deflectionRedB) {
                    nextState = ClimbStates.PullPastHookB;
                }
                break;
            case PullPastHookB:
                if (!deflectionWhiteB && !deflectionRedB) {
                    nextState = ClimbStates.Pause1;
                }
                break;

            case Pause1:
                nextState = ClimbStates.ExtendToBar2;
                break;
            case ExtendToBar2:
                if (climbMotorRevs >= revolutionsToBar2) {
                    // nextState = ClimbStates.ExtendToBar2Check;
                    nextState = ClimbStates.PullBar2ToStatHooksC;
                }
                break;
            case ExtendToBar2Check:
                if ((leadHookContactLeft && leadHookContactRight) || overrideLeadHooks) {
                    nextState = ClimbStates.PullBar2ToStatHooksC;
                }
                break;
            case PullBar2ToStatHooksC:
                if (deflectionWhiteC && deflectionRedC) {
                    nextState = ClimbStates.PullBar2PastStatHooksC;
                }
                break;
            case PullBar2PastStatHooksC:
                if (!deflectionWhiteC && !deflectionRedC) {
                    nextState = ClimbStates.Done;
                } // ExtendtoHookA2, ExtendPastA2, PulltoHookB2, PullPastHookB2
                break;

            case ExtendtoHookA2:
                if (deflectionA) {
                    nextState = ClimbStates.ExtendPastA2;
                }
                break;
            case ExtendPastA2:
                if (!deflectionA) {
                    nextState = ClimbStates.PulltoHookB2;
                }
                break;
            case PulltoHookB2:
                if (deflectionWhiteB && deflectionRedB) {
                    nextState = ClimbStates.PullPastHookB2;
                }
                break;
            case PullPastHookB2:
                if (!deflectionWhiteB && !deflectionRedB) {
                    nextState = ClimbStates.ExtendBar3Swing;
                }
                break;

            case ExtendBar3Swing:
                if (climbMotorRevs >= revolutionsNearBar3) {
                    nextState = ClimbStates.WaitForCalm;
                }
                break;
            case WaitForCalm:
                if (navxCalm) {
                    nextState = ClimbStates.ExtendBar3Calm;
                }
                break;
            case ExtendBar3Calm:
                if (climbMotorRevs >= revolutionsToBar3) {
                    nextState = ClimbStates.ExtendToBar3Check;
                }
                break;
            case ExtendToBar3Check:
                if ((leadHookContactRight && leadHookContactLeft) || overrideLeadHooks) {
                    nextState = ClimbStates.PullupBar3;
                }
                break;
            case PullupBar3:
                if (climbMotorRevs <= revolutionsToBar3Final) {
                    nextState = ClimbStates.Done;
                }
                break;
            case Done:
                break;
            case Pause:
                if (startClimb) {
                    nextState = pauseState;
                }
                break;
            case Manual:

                break;
        }
    }

    @Override
    public void doTransition() {
        if (isTransitionTo(ClimbStates.Home)) {
            climbMotor.setPower(homePower);
            // climberPosition.set(Value.kForward);
        }
        if (isTransitionFrom(ClimbStates.Home)) {
            climbMotor.setPower(0);
            climbMotor.resetEncoders();
        }
        if (isTransitionTo(ClimbStates.ExtendToBar1fast)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.ExtendToBar1slow)) {
            climbMotor.setPower(pwrForBarExtendslow);
        }
        if (isTransitionFrom(ClimbStates.ExtendToBar1slow)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.PullBar1ToStatHooksC)) {
            // climbMotor.setVelocity(rpmForBarPull);
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullBar1PastStatHooksC)) {
            // climbMotor.setVelocity(rpmForPullPast);
            climbMotor.setPower(pwrForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendtoHookA)) {
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.ExtendPastA)) {
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.PulltoHookB)) {
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullPastHookB)) {
            climbMotor.setPower(pwrForPullPast);
        }
        if (isTransitionFrom(ClimbStates.PullPastHookB)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.Pause1)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.ExtendToBar2)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtend);
            // climberPosition.set(Value.kReverse);
        }
        if (isTransitionFrom(ClimbStates.ExtendToBar2)) {
            // climbMotor.setVelocity(0);
            climbMotor.setPower(0);
            // climberPosition.set(Value.kForward);
        }
        if (isTransitionTo(ClimbStates.PullBar2ToStatHooksC)) {
            // climbMotor.setVelocity(rpmForBarPull);
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullBar2PastStatHooksC)) {
            // climbMotor.setVelocity(rpmForPullPast);
            climbMotor.setPower(pwrForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendtoHookA2)) {
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.ExtendPastA2)) {
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.PulltoHookB2)) {
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullPastHookB2)) {
            climbMotor.setPower(pwrForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendBar3Swing)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtend);
            // climberPosition.set(Value.kReverse);
        }
        if (isTransitionFrom(ClimbStates.ExtendBar3Swing)) {
            // climbMotor.setVelocity(0);
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.ExtendBar3Calm)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionFrom(ClimbStates.ExtendBar3Calm)) {
            // climbMotor.setVelocity(0);
            climbMotor.setPower(0);
            // climberPosition.set(Value.kForward);
        }
        if (isTransitionTo(ClimbStates.PullupBar3)) {
            // climbMotor.setVelocity(rpmForBarPull);
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionFrom(ClimbStates.PullupBar3)) {
            // climbMotor.setVelocity(0);
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.Pause)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.Manual)) {
            climbMotor.setPower(0);
        }
        if (isTransitionTo(ClimbStates.Done)) {
            climbMotor.setPower(0);
        }
    }

    @Override
    public void doCurrentState() {
        // switch (currentState) {
        // case Manual:
        // if (manualMoveClimberUp) {
        // manualClimbPosition += 3;
        // }
        // if (manualMoveClimberDown) {
        // manualClimbPosition -= 3;
        // }
        // if (lowLimitSwitch) {
        // climbMotor.resetEncoders();
        // if(lowLimitSwitch && !prevLimitSwitch){
        // manualClimbPosition = 0;

        // }
        // climberHomed = true;
        // }
        // prevLimitSwitch= lowLimitSwitch;
        // if(climberHomed){
        // climbMotor.setPosition(manualClimbPosition);}
        // break;
        // }
    }

    public void resetState() {
        currentState = ClimbStates.Start;
    }

    public void LogHeader() {
        Logger.Header("climbMotorRevs,climbMotorRPM,"
                + "climberHomed,startClimb,overrideLeadHooks,manualHome,lowLimitSwitch,RaiseLeadH, lowerLeadH, LeftLead, RightLead, DEFA, RedDEFB, WhiteDEFB, RedDEFC, WhiteDEFC"
                + "ClimbState");
    }

    public void LogData() {
        Logger.doubles(climbMotorRevs, climbMotorRPM);
        Logger.booleans(climberHomed,startClimb,overrideLeadHooks,manualHome,lowLimitSwitch, raiseLeadHooks, lowerLeadHooks, leadHookContactLeft, leadHookContactRight,
                deflectionA, deflectionRedB, deflectionWhiteB, deflectionRedC, deflectionWhiteC);
        Logger.singleEnum(currentState);
    }

}
