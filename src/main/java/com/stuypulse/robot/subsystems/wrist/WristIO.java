package com.stuypulse.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    @AutoLog
    public class WristIOInputs {
        public boolean wristMotorConnected = false;

        public double wristAngle = 0.0;
        public double wristAngularVel = 0.0;
        public double wristAngularAccel = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrentAmps = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {};
    public default void controlWrist(Rotation2d position) {};
}
