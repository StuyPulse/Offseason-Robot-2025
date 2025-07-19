/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

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
            Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);
        }

        public interface Elbow {
            Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);
        }
    }
}
