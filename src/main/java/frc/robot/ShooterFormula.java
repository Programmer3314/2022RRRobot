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
        //Distance, ShooterRpm, CamAngle, FeedRpm, TargetError
        targetPoints.add(new TargetPoint(0, 3400, 0, 3400, 3));
        targetPoints.add(new TargetPoint(3, 3640, 0, 3640, 3));

         targetPoints.add(new TargetPoint(4, 3750, 9 , 3750, 3));
        targetPoints.add(new TargetPoint(13.5, 4600, 45, 4600, 3));
        targetPoints.add(new TargetPoint(16, 4800, 58, 4800, 3));
        targetPoints.add(new TargetPoint(20, 5000, 70, 5000, 3));
        targetPoints.add(new TargetPoint(23, 6000, 70,6000, 3));
        targetPoints.add(new TargetPoint(25, 6000, 70, 6000, 3));
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
            double targetError = ratio * (upper.turretMargin - lower.turretMargin) + lower.turretMargin;
            // we did this :)
            TargetPoint result = new TargetPoint(targetDistance, rpm, angle, feedrpm, targetError);
            return result;
        }
    }
}
