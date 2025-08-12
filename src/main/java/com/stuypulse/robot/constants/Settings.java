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
        public interface Shoulder {

            double DEFAULT = 0.0;
            double L2 = 0.0;
            double L3 = 0.0;
            double L4 = 0.0;

            public interface PID{
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;
            }
            public interface FF{
                double kS = 0.0;
                double kG = 0.0;
                double kV = 0.0;
                double kA = 0.0;
            }

            double TOLERANCE = 0.1;
            
            Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
        
        public interface Elbow {

            double DEFAULT = 0.0;
            double L2 = 0.0;
            double L3 = 0.0;
            double L4 = 0.0;
            
            public interface PID{
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;
            }
            public interface FF{
                double kS = 0.0;
                double kG = 0.0;
                double kV = 0.0;
                double kA = 0.0;
            }

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
}