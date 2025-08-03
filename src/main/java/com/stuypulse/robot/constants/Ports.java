/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface DoubleJointedArm {
        int SHOULDER_MOTOR = 0;
        int SHOULDER_ENCODER = 0;
        int ELBOW_MOTOR = 0;
        int ELBOW_ENCODER = 0;
        int PIGEON = 0;
    }

    public interface Wrist {
        int WRIST_MOTOR = 0;
        int WRIST_ENCODER = 0;
    }
}
