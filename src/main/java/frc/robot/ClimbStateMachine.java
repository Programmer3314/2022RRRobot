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
    PullBar1ToStatHooksC, PullBar1PastStatHooksC, ExtendtoHookA, ExtendPastA, AnglePause1, PulltoHookB, PullPastHookB,
    Pause1, ExtendNearBar2, ExtendToBar2CheckPressed, ExtendToBar2CheckReleased, ExtendPastBar2,
    PullToBar2CheckPresssed, PullToBar2CheckReleased, PullBar2ToStatHooksC,
    PullBar2PastStatHooksC, Bar2SafetyExtend, ExtendtoHookA2, ExtendPastA2, AnglePause2,
    PulltoHookB2, PullPastHookB2, Pause2,
    ExtendNearBar3, ExtendToBar3CheckPressed, ExtendToBar3CheckReleased, ExtendPastBar3,
    PullToBar3CheckPresssed, PullToBar3CheckReleased, PullBar3ToStatHooksC,
    PullBar3PastStatHooksC, Bar3SafetyExtend, extendNearNearBar2, extendNearNearBar3,
    // ExtendBar3Swing, WaitForCalm, ExtendBar3Calm, ExtendToBar3Check, PullupBar3,
    Done, Pause, Manual
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
    double revolutionsToBar1slow = 67;//70
    double revolutionsToBar2 = 55;// 51
    double revolutionsToSafeBar2 = 11;
    double revolutionsNearBar2 = 28;
    double revolutionsNearBar3 = 28;
    double revolutionsToBar3 = 55;// 51
    double revolutionsPastHookA = 44;
    double revolutionsToHookA = 31;
    double revolutionsToHookC = 8;
    double revolutionsToHookB = 15;
    double revolutionsPastHookB = 3.25;// 3.25
    double revolutionsToSafeBar3 = 11;
    boolean navxCalm = true;
    double rpmForBarExtend = 100;
    double pwrForBarExtend = .3;//.3
    double pwrForBarExtendslow = .1;
    double pwrForBarExtendFast = .5;//.5
    double rpmForBarPull = -200;
    double pwrForBarPull = -.7;
    double pwrForBarFastPull = -.7;
    double rpmForPullPast = -150;
    double pwrForPullPast = -.4;//-.5
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
    boolean highBarPressed;
    boolean highBarReleased;
    boolean climbing;
    double revolutionsNearNearBar2 = 25;
    double revolutionsNearNearBar3 = 25;

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
        climbing = false;
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
        highBarPressed = Robot.controllerOperator.getRawButtonPressed(Constants.kOperatorHighBarResume);
        highBarReleased = Robot.controllerOperator.getRawButtonReleased(Constants.kOperatorHighBarResume);

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
        SmartDashboard.putBoolean("StopWhiteBelt", climbing);
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
                // if (deflectionRedC && deflectionWhiteC) {
                // nextState = ClimbStates.PullBar1PastStatHooksC;
                // }
                if (climbMotorRevs <= revolutionsToHookC) {
                    nextState = ClimbStates.PullBar1PastStatHooksC;
                }
                // break;
                break;
            case PullBar1PastStatHooksC:
                if (!deflectionRedC && !deflectionWhiteC) {
                    // nextState = ClimbStates.ExtendToBar2;
                    nextState = ClimbStates.ExtendtoHookA;
                    climbing = true;
                }
                break;
            case ExtendtoHookA:
                // if (deflectionA) {
                // nextState = ClimbStates.ExtendPastA;
                // }
                // break;
                if (climbMotorRevs >= revolutionsToHookA) {
                    nextState = ClimbStates.ExtendPastA;
                }
                // nextState = ClimbStates.ExtendPastA;
                break;
            case ExtendPastA:
                if (!deflectionA) {
                    // nextState = ClimbStates.PulltoHookB;
                    nextState = ClimbStates.AnglePause1;
                }
                if (climbMotorRevs >= revolutionsPastHookA) {
                    nextState = ClimbStates.Done;
                }
                // if(climbMotorRevs>= revolutionsToHookA){
                // if (!deflectionA) {
                // nextState = ClimbStates.PulltoHookB;
                // }
                // else{
                // nextState = ClimbStates.Done;
                // }
                // }

                break;
            case AnglePause1:
                if (Robot.navx.getRoll() > 40) {
                    nextState = ClimbStates.PulltoHookB;
                }
                break;
            case PulltoHookB:
                // if (deflectionWhiteB && deflectionRedB) {
                // nextState = ClimbStates.PullPastHookB;
                // }
                // break;
                if (climbMotorRevs <= revolutionsToHookB) {
                    nextState = ClimbStates.PullPastHookB;
                }
                break;
            case PullPastHookB:
                if (climbMotorRevs <= revolutionsPastHookB) {
                    if (!deflectionWhiteB && !deflectionRedB) {
                        nextState = ClimbStates.Pause1;
                    } else {
                        nextState = ClimbStates.Done;
                    }
                }
                break;
            case Pause1:
                // if(Robot.navx.getRoll()<40){
                nextState = ClimbStates.extendNearNearBar2;
                // }
                break;
            case extendNearNearBar2:
                if (climbMotorRevs >= revolutionsNearNearBar2){
                    nextState = ClimbStates.ExtendNearBar2;
                }
                break;

            case ExtendNearBar2:
                if (climbMotorRevs >= revolutionsNearBar2) {
                    // nextState = ClimbStates.ExtendToBar2Check;
                    // nextState = ClimbStates.PullBar2ToStatHooksC;
                    nextState = ClimbStates.ExtendToBar2CheckPressed;
                }
                break;
            case ExtendToBar2CheckPressed:
                if (/* highBarPressed */Robot.navXRoll.isApproachingBar()) {
                    nextState = ClimbStates.ExtendToBar2CheckReleased;
                }
                break;
            case ExtendToBar2CheckReleased:
                if (/* highBarReleased */Robot.navXRoll.isBelowBar(60)) {
                    nextState = ClimbStates.ExtendPastBar2;
                }
                break;
            case ExtendPastBar2:
                if (climbMotorRevs >= revolutionsToBar2) {
                    nextState = ClimbStates.PullToBar2CheckPresssed;
                }
                break;
            case PullToBar2CheckPresssed:
                // if (/* highBarPressed */secondsInState >= 2) {
                    nextState = ClimbStates.PullToBar2CheckReleased;
                // }
                break;
            case PullToBar2CheckReleased:
                if (/* highBarReleased */Robot.navXRoll.navxCalm()) {
                    nextState = ClimbStates.PullBar2ToStatHooksC;
                }
                break;
            case PullBar2ToStatHooksC:
                // if (deflectionWhiteC && deflectionRedC) {
                // nextState = ClimbStates.PullBar2PastStatHooksC;
                // }
                if (climbMotorRevs <= revolutionsToHookC) {
                    nextState = ClimbStates.PullBar2PastStatHooksC;
                }
                break;
            case PullBar2PastStatHooksC:
                if (!deflectionWhiteC && !deflectionRedC) {
                    // nextState = ClimbStates.Bar2SafetyExtend;
                    // nextState = ClimbStates.Done;
                    nextState = ClimbStates.ExtendtoHookA2;
                } // ExtendtoHookA2, ExtendPastA2, PulltoHookB2, PullPastHookB2
                break;
            case Bar2SafetyExtend:
                if (climbMotorRevs >= revolutionsToSafeBar2) {
                    nextState = ClimbStates.Done;
                }
                break;
            case ExtendtoHookA2:
                // if (deflectionA) {
                // nextState = ClimbStates.ExtendPastA2;
                // }
                if (climbMotorRevs >= revolutionsToHookA) {
                    nextState = ClimbStates.ExtendPastA2;
                }
                break;
            case ExtendPastA2:
                if (!deflectionA) {
                    // nextState = ClimbStates.PulltoHookB2;
                    nextState = ClimbStates.AnglePause2;
                }
                if (climbMotorRevs >= revolutionsPastHookA) {
                    nextState = ClimbStates.Done;
                }
                break;
            case AnglePause2:
                if (Robot.navx.getRoll() > 40) {
                    nextState = ClimbStates.PulltoHookB2;
                }
                break;
            case PulltoHookB2:
                // if (deflectionWhiteB && deflectionRedB) {
                // nextState = ClimbStates.PullPastHookB2;
                // }
                if (climbMotorRevs <= revolutionsToHookB) {
                    nextState = ClimbStates.PullPastHookB2;
                }
                break;
            case PullPastHookB2:
                if (climbMotorRevs <= revolutionsPastHookB) {
                    if (!deflectionWhiteB && !deflectionRedB) {
                        nextState = ClimbStates.Pause2;
                    } else {
                        nextState = ClimbStates.Done;
                    }
                }
                break;

            case Pause2:
                // if(Robot.navx.getRoll()<40){
                nextState = ClimbStates.extendNearNearBar3;
                // }
                break;
            case extendNearNearBar3:
                if(climbMotorRevs >= revolutionsNearNearBar3){
                    nextState = ClimbStates.ExtendNearBar3;

            }
            break;
            // if (!deflectionWhiteB && !deflectionRedB) {
            // nextState = ClimbStates.ExtendNearBar3;
            // }
            // break;

            // case ExtendBar3Swing:
            // if (climbMotorRevs >= revolutionsNearBar3) {
            // nextState = ClimbStates.WaitForCalm;
            // }
            // break;
            // case WaitForCalm:
            // if (navxCalm) {
            // nextState = ClimbStates.ExtendBar3Calm;
            // }
            // break;
            // case ExtendBar3Calm:
            // if (climbMotorRevs >= revolutionsToBar3) {
            // nextState = ClimbStates.ExtendToBar3Check;
            // }
            // break;
            // case ExtendToBar3Check:
            // if ((leadHookContactRight && leadHookContactLeft) || overrideLeadHooks) {
            // nextState = ClimbStates.PullupBar3;
            // }
            // break;
            // case PullupBar3:
            // if (climbMotorRevs <= revolutionsToBar3Final) {
            // nextState = ClimbStates.Done;
            // }
            // break;
            case ExtendNearBar3:
                if (climbMotorRevs >= revolutionsNearBar3) {
                    // nextState = ClimbStates.ExtendToBar2Check;
                    // nextState = ClimbStates.PullBar2ToStatHooksC;
                    nextState = ClimbStates.ExtendToBar3CheckPressed;
                }
                break;
            case ExtendToBar3CheckPressed:
                if (/* highBarPressed */Robot.navXRoll.isApproachingBar()){
                //|| (Robot.navXRoll.isGoingDown() && Robot.navx.getRoll() > 70)) {
                    nextState = ClimbStates.ExtendToBar3CheckReleased;
                }
                break;
            case ExtendToBar3CheckReleased:
                if (/* highBarReleased */Robot.navXRoll.isBelowBar(57)) {
                    nextState = ClimbStates.ExtendPastBar3;
                }
                break;
            case ExtendPastBar3:
                if (climbMotorRevs >= revolutionsToBar3) {
                    nextState = ClimbStates.PullToBar3CheckPresssed;
                }
                break;
            case PullToBar3CheckPresssed:
                // if (/* highBarPressed */secondsInState >= 2) {
                    nextState = ClimbStates.PullToBar3CheckReleased;
                // }
                break;
            case PullToBar3CheckReleased:
                if (/* highBarReleased */Robot.navXRoll.navxCalm()) {
                    nextState = ClimbStates.PullBar3ToStatHooksC;
                }
                break;
            case PullBar3ToStatHooksC:
                // if (deflectionWhiteC && deflectionRedC) {
                // nextState = ClimbStates.PullBar3PastStatHooksC;
                // }
                // break;
                if (climbMotorRevs <= revolutionsToHookC) {
                    nextState = ClimbStates.PullBar3PastStatHooksC;
                }
                break;
            case PullBar3PastStatHooksC:
                if (!deflectionWhiteC && !deflectionRedC) {
                    nextState = ClimbStates.Bar3SafetyExtend;
                    // nextState = ClimbStates.Done;
                } // ExtendtoHookA2, ExtendPastA2, PulltoHookB2, PullPastHookB2
                break;
            case Bar3SafetyExtend:
                if (climbMotorRevs >= revolutionsToSafeBar3) {
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
        if (isTransitionFrom(ClimbStates.Home)) {
            climbMotor.setPower(0);
            climbMotor.resetEncoders();
        }
        if (isTransitionFrom(ClimbStates.ExtendToBar1slow)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.PullPastHookB)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.extendNearNearBar2, ClimbStates.extendNearNearBar3)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.ExtendNearBar2, ClimbStates.ExtendNearBar3)) {
            // climbMotor.setVelocity(0);
            climbMotor.setPower(0);
            // climberPosition.set(Value.kForward);
        }
        // if (isTransitionFrom(ClimbStates.ExtendBar3Swing)) {
        // // climbMotor.setVelocity(0);
        // climbMotor.setPower(0);
        // }
        // if (isTransitionFrom(ClimbStates.ExtendBar3Calm)) {
        // // climbMotor.setVelocity(0);
        // climbMotor.setPower(0);
        // // climberPosition.set(Value.kForward);
        // }
        // if (isTransitionFrom(ClimbStates.PullupBar3)) {
        // // climbMotor.setVelocity(0);
        // climbMotor.setPower(0);
        // }
        // PullBar3PastStatHooksC, ExtendPastBar3,PullBar3PastStatHooksC,
        // Bar3SafetyExtend,
        if (isTransitionFrom(ClimbStates.ExtendPastBar2, ClimbStates.ExtendPastBar3)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.Bar2SafetyExtend, ClimbStates.Bar3SafetyExtend)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.ExtendPastA2, ClimbStates.ExtendPastA)) {
            climbMotor.setPower(0);
        }
        if (isTransitionFrom(ClimbStates.PullPastHookB2, ClimbStates.PullPastHookB)) {
            climbMotor.setPower(0);
        }

        if (isTransitionTo(ClimbStates.Home)) {
            climbMotor.setPower(homePower);
            // climberPosition.set(Value.kForward);
        }
        // if(isTransitionTo(ClimbStates.AnglePause1, ClimbStates.AnglePause2)){
        // climbMotor.setPower(0);
        // }
        if (isTransitionTo(ClimbStates.Bar2SafetyExtend, ClimbStates.Bar3SafetyExtend)) {
            climbMotor.setPower(pwrForBarExtendslow);
        }
        if (isTransitionTo(ClimbStates.ExtendPastBar2, ClimbStates.ExtendPastBar3)) {
            // climbMotor.setPower(pwrForBarExtendslow);
            // climbMotor.setPower(pwrForBarExtend);
            climbMotor.setPower(pwrForBarExtendFast);
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
        // if (isTransitionTo(ClimbStates.PullupBar3)) {
        // // climbMotor.setVelocity(rpmForBarPull);
        // climbMotor.setPower(pwrForBarPull);
        // }
        if (isTransitionTo(ClimbStates.PullBar2ToStatHooksC, ClimbStates.PullBar3ToStatHooksC)) {
            // climbMotor.setVelocity(rpmForBarPull);
            // climbMotor.setPower(pwrForBarPull);
            climbMotor.setPower(pwrForBarFastPull);
        }
        if (isTransitionTo(ClimbStates.PullBar2PastStatHooksC, ClimbStates.PullBar3PastStatHooksC)) {
            // climbMotor.setVelocity(rpmForPullPast);
            climbMotor.setPower(pwrForPullPast);
        }
        if (isTransitionTo(ClimbStates.ExtendtoHookA2)) {
            climbMotor.setPower(pwrForBarExtend);
        }
        // if (isTransitionTo(ClimbStates.ExtendPastA2)) {
        // climbMotor.setPower(pwrForBarExtend);
        // }
        if (isTransitionTo(ClimbStates.PulltoHookB2)) {
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.extendNearNearBar2, ClimbStates.extendNearNearBar3)){
            climbMotor.setPower(pwrForBarExtend);
        }
        if (isTransitionTo(ClimbStates.PullPastHookB2)) {
            climbMotor.setPower(pwrForPullPast);
        }
        // if (isTransitionTo(ClimbStates.ExtendBar3Swing)) {
        // // climbMotor.setVelocity(rpmForBarExtend);
        // climbMotor.setPower(pwrForBarExtend);
        // // climberPosition.set(Value.kReverse);
        // }
        // if (isTransitionTo(ClimbStates.ExtendBar3Calm)) {
        // // climbMotor.setVelocity(rpmForBarExtend);
        // climbMotor.setPower(pwrForBarExtend);
        // }

        if (isTransitionTo(ClimbStates.ExtendNearBar2, ClimbStates.ExtendNearBar3)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtend);
            // climberPosition.set(Value.kReverse);
        }
        if (isTransitionTo(ClimbStates.ExtendToBar1fast)) {
            // climbMotor.setVelocity(rpmForBarExtend);
            climbMotor.setPower(pwrForBarExtendFast);
            Robot.shooterStateMachine.resetState();
        }
        if (isTransitionTo(ClimbStates.ExtendToBar1slow)) {
            climbMotor.setPower(pwrForBarExtendslow);
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
        if (isTransitionTo(ClimbStates.ExtendPastA, ClimbStates.ExtendPastA2)) {
            climbMotor.setPower(pwrForBarExtend);// pwrforbarextendslow
        }
        if (isTransitionTo(ClimbStates.PulltoHookB)) {
            climbMotor.setPower(pwrForBarPull);
        }
        if (isTransitionTo(ClimbStates.PullPastHookB)) {
            climbMotor.setPower(pwrForPullPast);
        }
        // ExtendNearBar2, ExtendToBar2CheckPressed, ExtendToBar2CheckReleased,
        // ExtendPastBar2,
        // PullToBar2CheckPresssed, PullToBar2CheckReleased, PullBar2ToStatHooksC,
        // PullBar2PastStatHooksC, Bar2SafetyExtend,

    }

    @Override
    public void doCurrentState() {
        switch (currentState) {
            case ExtendPastBar2:
                if (climbMotor.getRevolutions() > 52) {
                    climbMotor.setPower(pwrForBarExtendslow);
                }
                break;
            case ExtendPastBar3:
                if (climbMotor.getRevolutions() > 52) {
                    climbMotor.setPower(pwrForBarExtendslow);
                }
                break;
        }
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

    public boolean isClimbing() {
        // climbing = true;
        return climbing;
    }

    public void resetState() {
        currentState = ClimbStates.Start;
        climbing = false;
    }

    public void LogHeader() {
        Logger.Header("climbMotorRevs,climbMotorRPM,"
                + "climberHomed,startClimb,overrideLeadHooks,manualHome,lowLimitSwitch,RaiseLeadH, lowerLeadH, LeftLead, RightLead, DEFA, RedDEFB, WhiteDEFB, RedDEFC, WhiteDEFC,"
                + "Bar2Pressed, Bar2Released,Trident,"
                + "ClimbState,");
    }

    public void LogData() {
        Logger.doubles(climbMotorRevs, climbMotorRPM);
        Logger.booleans(climberHomed, startClimb, overrideLeadHooks, manualHome, lowLimitSwitch, raiseLeadHooks,
                lowerLeadHooks, leadHookContactLeft, leadHookContactRight,
                deflectionA, deflectionRedB, deflectionWhiteB, deflectionRedC, deflectionWhiteC, highBarPressed,
                highBarReleased, climbing);
        Logger.singleEnum(currentState);
    }

}
