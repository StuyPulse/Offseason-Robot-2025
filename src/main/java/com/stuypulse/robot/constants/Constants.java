package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
public interface Constants {
    public static final double SHOOTER_Y_OFFSET = 0;

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);


    public interface Swerve {
        double WIDTH = Units.inchesToMeters(18.75);
        double LENGTH = Units.inchesToMeters(18.75);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 5.36;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
            }
        }

        // Comment:
        // Back Right - Bottom Right
        // Back Left - Top Right
        // 

        // TODO: Rotation2D add 0.25 FIX.
        
        public interface FrontLeft {
            String ID = "Front Left";
            // Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.1062011);
            // Add 180 deg.
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.6062011 + 0.25);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.105222 + 0.25);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.225083 + 0.25);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }

        public interface FrontRight {
            String ID = "Front Right";
            // Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.186766);
            // add 180 deg
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.313234 + 0.25);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
    }
}