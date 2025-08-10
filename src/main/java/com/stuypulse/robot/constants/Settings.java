/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
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

}