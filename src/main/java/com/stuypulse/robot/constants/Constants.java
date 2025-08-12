package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public interface Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double SHOOTER_Y_OFFSET = 0;

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
    public interface DoubleJointedArm {
        double BASE_HEIGHT = 0.2032; // 8 in -> 0.2032 m
        
        public interface Shoulder {
            double LENGTH = 0.9271; // 36.5 in -> 0.9779 m
            double MASS = 0.51845608; // 1.143 lbs -> 0.51845608 kg

            double MOTOR_GEAR_RATIO = 3515.0 / 27.0; // Check
            double ENCODER_GEAR_RATIO = 4.625 / 1;
            double GEAR_RATIO = MOTOR_GEAR_RATIO * ENCODER_GEAR_RATIO;

            double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0.0).getRotations();
            double ENCODER_UPPER_LIMIT_ROT = Rotation2d.fromDegrees(0.0).getRotations();

            Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-90);
            Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(90);

            
        }

        public interface Elbow {
            double LENGTH = 0.519884533; // 20.467895 in -> 0.519884533 m
            double MASS = 0.39780051; // 0.877 lbs -> 0.39780051 kg

            double MOTOR_GEAR_RATIO = 3515.0 / 27.0;
            double ENCODER_GEAR_RATIO = 3.0327 / 1; 
            double GEAR_RATIO = MOTOR_GEAR_RATIO * ENCODER_GEAR_RATIO;
    
            double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0.0).getRotations();
            double ENCODER_UPPER_LIMIT_ROT = Rotation2d.fromDegrees(0.0).getRotations();

            Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-180);
            Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
        }
    }

    public interface Wrist {
        double GEAR_RATIO = 1/1;
        double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0).getRotations();
    }

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

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.149902);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.270752);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.113037);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(-0.441162);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
    }
}
