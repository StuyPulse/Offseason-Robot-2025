package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public interface Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

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
}
