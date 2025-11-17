package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.02;
    String CANIVORE = "CANIVORE";
    double TARGET_DISTANCE_FROM_REEF = 0;
    
    public interface DoubleJointedArm {
        public static final double PATH_DT = 0.1; // Time between path points (sec)
        public static final double SPLINE_DURATION = 0.5; // Duration per spline segment

        public interface Shoulder {
            double DEFAULT = 90.0;
            double L2 = 0.0;
            double L3 = 0.0;
            double L4 = 0.0;

            double TOLERANCE = .05;
            
            Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
        
        public interface Elbow {
            double DEFAULT = -90.0;
            double L2 = 0.0;
            double L3 = 0.0;
            double L4 = 0.0;
            
            double TOLERANCE = 0.1;

            Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
    }
    
    public interface Wrist {
        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);
    }

    public interface Swerve {
        double MAX_MODULE_SPEED = 3.0;
        double MAX_MODULE_ACCEL = 4.0;
        double MODULE_VELOCITY_DEADBAND = 0.05;

        SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 2.5);
        SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 3.0);
        SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(360));
        SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));

    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad per s)", Swerve.MAX_ANGULAR_VELOCITY.get());
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad per s^2)", Swerve.MAX_ANGULAR_ACCELERATION.get());
        }
    }
}