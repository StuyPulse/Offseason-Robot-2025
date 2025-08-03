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
        double BASE_HEIGHT = 0.2;
        public interface Shoulder {
            double LENGTH = 1.0;
            double MASS = 1.0;

            double MOTOR_GEAR_RATIO = 1.0 / 85.0;
            double ENCODER_GEAR_RATIO = 4.625 / 1;
            double GEAR_RATIO = MOTOR_GEAR_RATIO * ENCODER_GEAR_RATIO;

            double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0.0).getRotations();
            double ENCODER_UPPER_LIMIT_ROT = Rotation2d.fromDegrees(0.0).getRotations();

            Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-70);
            Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(-70);
            
        }

        public interface Elbow {
            double LENGTH = 1.0;
            double MASS = 1.0;

            double MOTOR_GEAR_RATIO = 1.0 / 85.0;
            double ENCODER_GEAR_RATIO = 4.625 / 1;
            double GEAR_RATIO = MOTOR_GEAR_RATIO * ENCODER_GEAR_RATIO;
    
            double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0.0).getRotations();
            double ENCODER_UPPER_LIMIT_ROT = Rotation2d.fromDegrees(0.0).getRotations();

            Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-70);
            Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(-70);
        }
    }

    public interface Wrist {
        double GEAR_RATIO = 1/1;
        double ENCODER_OFFSET_ROT = Rotation2d.fromDegrees(0).getRotations();
    }
}
