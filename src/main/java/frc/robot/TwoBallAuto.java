// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Robot.autocorrectTargetAngle;
import static frc.robot.Robot.confidenceCounter;
import static frc.robot.Robot.currentAngle;
import static frc.robot.Robot.driveTrain;
import static frc.robot.Robot.lightRing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.MMAutonomous;

enum TBautoStates {
    Start, DriveBack, Shoot, Done
};

/**
 * Steps:
 * Drive back certain distance+Stop(3ft in tests)
 * Turn robot away from target(10Degrees), then Autoturn back to the goal
 */
public class TwoBallAuto extends MMAutonomous<TBautoStates> {
    double goalAngle;
    double autoTargetRunCounter;

    public TwoBallAuto() {
        super(TBautoStates.Start);
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        update();
        SmartDashboard.putString("Auto State", currentState.toString());
    }

    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = TBautoStates.DriveBack;
                break;
            case DriveBack:
                if (Robot.driveTrain.getDistanceFeet() <= -3.0) {
                    nextState = TBautoStates.Shoot;
                }
                break;
            case Shoot:
                nextState = TBautoStates.Done;
            case Done:
                break;
        }
    }

    @Override
    public void doTransition() {
        if (isTransitionTo(TBautoStates.DriveBack)){
            Robot.driveTrain.resetEncoders();
            Robot.intake.intake();
            Robot.driveTrain.Drive(-1, 0);
        }
        if (isTransitionTo(TBautoStates.Shoot)){
            Robot.driveTrain.Drive(0, 0);
            Robot.shooterStateMachine.shootAll();
            Robot.intake.idle();
        }
    }

    @Override
    public void doCurrentState() {     
    }
}
