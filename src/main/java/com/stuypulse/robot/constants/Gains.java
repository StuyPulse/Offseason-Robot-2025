package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.stuylib.network.SmartNumber;

public interface Gains {
    
    public interface Swerve {

        public interface Alignment {
            PIDConstants XY = new PIDConstants(2.5, 0, 0.1);
            PIDConstants THETA = new PIDConstants(3, 0, 0.1);
        }

        public interface Turn {
            double kP = 3.596;
            double kI = 0.0;
            double kD = 0.05;
        }

        public interface Drive {
            double kP = 6.7279E-06;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.4825;
            double kV = 1.7516;
            double kA = 0.075654;
        }
    }
}
