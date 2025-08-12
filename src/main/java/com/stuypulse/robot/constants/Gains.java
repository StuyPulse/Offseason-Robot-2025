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

    public interface Elevator {
        public interface PID {
            SmartNumber kP = new SmartNumber("Elevator/Controller/kP", 6.25);
            SmartNumber kI = new SmartNumber("Elevator/Controller/kI",0.0);
            SmartNumber kD = new SmartNumber("Elevator/Controller/kD",0.1);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Elevator/Controller/kS",0.0);
            SmartNumber kV = new SmartNumber("Elevator/Controller/kV",0.0);
            SmartNumber kA = new SmartNumber("Elevator/Controller/kA", 0.0);
            SmartNumber kG = new SmartNumber("Elevator/Controller/kG", 1.0);
        }
    }
}
