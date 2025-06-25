package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Constants {
    
    public interface Intake {
        double PIVOT_GEAR_RATIO = 1/80; // made up number

        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0.0);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0.0);

    }

}
