package com.stuypulse.robot.constants;

public interface Gains {

    public interface Intake {
        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }

        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }
    }
} 
