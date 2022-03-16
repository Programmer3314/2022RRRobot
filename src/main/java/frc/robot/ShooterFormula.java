// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

//TODO add margin in parameters for rpm + angle in formula

/** Add your docs here. */
public class ShooterFormula {
    public ArrayList<TargetPoint> targetPointsLow;
    public ArrayList<TargetPoint> targetPointsHigh;

    // constructor that makes new list of target points
    //
    public ShooterFormula() {
        targetPointsLow = new ArrayList<>();
        // targetPointsLow.add(new TargetPoint(-5.2, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-4.8, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-2.2, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-1.8, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-1.2, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-0.8, 2000, 20, 2500, 5));
        // targetPointsLow.add(new TargetPoint(-0.2, 3100, 30, 2310, 5));
        targetPointsLow.add(new TargetPoint(0.2, 3100, 30, 2310, 5));
        targetPointsLow.add(new TargetPoint(3, 3640, 0, 2548, 5));
        targetPointsLow.add(new TargetPoint(4, 3750, 19, 2625, 5));
        targetPointsLow.add(new TargetPoint(7.5, 3000, 30, 3200, 5));
        // targetPointsLow.add(new TargetPoint(7.5, 2500, 20, 2730, 5));
        // targetPointsLow.add(new TargetPoint(11, 4250, 57, 2975, 5));
        // targetPointsLow.add(new TargetPoint(11, 4250, 57, 2975, 5));
        targetPointsLow.add(new TargetPoint(11, 4000, 57, 2975, 5));
        targetPointsLow.add(new TargetPoint(13.5, 4450, 57, 3325, 5));
        targetPointsLow.add(new TargetPoint(16, 4800, 65, 3360, 5));
        targetPointsLow.add(new TargetPoint(20, 5000, 90, 3500, 5));
        targetPointsLow.add(new TargetPoint(23, 6000, 70, 4200, 5));
        targetPointsLow.add(new TargetPoint(25, 6000, 70, 4200, 5));
//HIGH
        targetPointsHigh = new ArrayList<>();
        // Distance, ShooterRpm, CamAngle, FeedRpm, TargetError
        targetPointsHigh.add(new TargetPoint(-5.2, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-4.8, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-2.2, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-1.8, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-1.2, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-0.8, 2000, 20, 2500, 5));
        targetPointsHigh.add(new TargetPoint(-0.2, 3250, 28, 2310, 5));
        targetPointsHigh.add(new TargetPoint(0.2, 3250, 28, 2310, 5));//24pointblank
        targetPointsHigh.add(new TargetPoint(3, 3640, 0, 2548, 5));
        targetPointsHigh.add(new TargetPoint(4, 3600, 25, 2625, 5));
        targetPointsHigh.add(new TargetPoint(7.5, 4100, 57, 2730, 5));
        targetPointsHigh.add(new TargetPoint(11, 4750, 57, 4000, 5));
        targetPointsHigh.add(new TargetPoint(13.5, 5000, 53, 4000, 5));
        targetPointsHigh.add(new TargetPoint(16, 4800, 65, 3360, 5));
        targetPointsHigh.add(new TargetPoint(20, 5000, 90, 3500, 5));
        targetPointsHigh.add(new TargetPoint(23, 6000, 70, 4200, 5));
        targetPointsHigh.add(new TargetPoint(25, 6000, 70, 4200, 5));
    }

    public TargetPoint calculate(double targetDistance) {
        ArrayList<TargetPoint> targetPoints;
        //targetPoints = new ArrayList<>();
        if (Robot.povLeftShot) {
            targetPoints = targetPointsLow;
        } else {
            targetPoints = targetPointsHigh;
        }
        TargetPoint upper = null;
        TargetPoint lower = null;
        for (TargetPoint x : targetPoints) {
            if (x.distance >= targetDistance && lower != null) {
                upper = x;
                break;
            }
            if (x.distance <= targetDistance) {
                lower = x;
            }

        }
        if (upper == null || lower == null) {
            return null;
        } else {
            // creates ratio of difference between the 2 distance point
            double ratio = (targetDistance - lower.distance) / (upper.distance - lower.distance);
            double rpm = ratio * (upper.rpm - lower.rpm) + lower.rpm; // implents ratio to rpm
            double angle = ratio * (upper.angle - lower.angle) + lower.angle; // implents ratio to angle
            double feedrpm = ratio * (upper.feedrpm - lower.feedrpm) + lower.feedrpm;
            double targetError = ratio * (upper.turretMargin - lower.turretMargin) + lower.turretMargin;
            // we did this :)
            TargetPoint result = new TargetPoint(targetDistance, rpm, angle, feedrpm, targetError);
            return result;
        }
    }
}
