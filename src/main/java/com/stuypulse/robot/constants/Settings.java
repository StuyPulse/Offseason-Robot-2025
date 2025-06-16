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

    public interface EnabledSubsystems {
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake Is Enabled", true);
    }

    public interface Intake {

        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(0.0);
        Rotation2d INTAKING_ANGLE = Rotation2d.fromDegrees(0.0);

        double INTAKING_SPEED = 1.0;
        double EJECTING_SPEED = -1.0;

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.0);

        public interface Constraints {

            Rotation2d MAX_VEL = Rotation2d.fromDegrees(0.0);
            Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(0.0);

        }


    }

}
