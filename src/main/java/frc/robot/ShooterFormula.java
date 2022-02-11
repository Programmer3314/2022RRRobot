// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

//TODO add margin in parameters for rpm + angle in formula

/** Add your docs here. */
public class ShooterFormula {
    public ArrayList<TargetPoint> targetPoints;

    // constructor that makes new list of target points
    //
    public ShooterFormula() {
        targetPoints = new ArrayList<>();
        targetPoints.add(new TargetPoint(2.5, 2800, 0, 1400));
        targetPoints.add(new TargetPoint(10, 3200, 0.3, 1600));
        targetPoints.add(new TargetPoint(16, 3500, 0.5, 1750));
    }

    public TargetPoint calculate(double targetDistance) {
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
            double feedrpm = ratio*(upper.feedrpm - lower.feedrpm) + lower.feedrpm;
            // we did this :)
            TargetPoint result = new TargetPoint(targetDistance, rpm, angle, feedrpm);
            return result;
        }
    }
}
