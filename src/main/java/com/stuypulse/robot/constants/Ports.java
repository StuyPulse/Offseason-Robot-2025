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

    public interface Indexer {
        int LEFT_MOTOR = 0;
        int RIGHT_MOTOR = 1;

        int FRONT_BEAM_BREAK = 0;
        int BACK_BEAM_BREAK = 1;
    }
}
