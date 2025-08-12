package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface DoubleJointedArm {
        public interface Shoulder {
            int FRONT_MOTOR = 0;
            int BACK_MOTOR = 0;
            int ENCODER = 0;
        }

        public interface Elbow {
            int MOTOR = 0;
            int ENCODER = 0;
        }

    }

    public interface Wrist {
        int WRIST_MOTOR = 19;
        int WRIST_ENCODER = 20;
    }

    public interface Swerve {
        int PIGEON = 7;
        
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 1;
            int ENCODER = 11;
        }

        public interface FrontLeft {
            int DRIVE = 12;
            int TURN = 2;
            int ENCODER = 13;
        }

        public interface BackLeft {
            int DRIVE = 5;
            int TURN = 3;
            int ENCODER = 6;
        }

        public interface BackRight {
            int DRIVE = 8;
            int TURN = 4;
            int ENCODER = 9;
        }
    }
}
