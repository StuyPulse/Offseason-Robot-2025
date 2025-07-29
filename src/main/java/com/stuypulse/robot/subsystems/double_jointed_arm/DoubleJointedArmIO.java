package com.stuypulse.robot.subsystems.double_jointed_arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DoubleJointedArmIO {
    @AutoLog
    public class DoubleJointedArmIOInputs {
        public boolean shoulderMotorConnected = false;
        public boolean elbowMotorConnected = false;

        public double shoulderAngle = 0.0;
        public double shoulderAngularVel = 0.0;
        public double shoulderAngularAccel = 0.0;
        public double shoulderAppliedVoltage = 0.0;
        public double shoulderCurrentAmps = 0.0;

        public double elbowAngle = 0.0;
        public double elbowAngularAccel = 0.0;
        public double elbowAngularVel = 0.0;
        public double elbowAppliedVoltage = 0.0;
        public double elbowCurrentAmps = 0.0;
    }

    /* Updates logged data */
    public abstract void updateInputs(DoubleJointedArmIOInputs inputs);

    /* Control functions */
    public abstract void controlShoulder(Rotation2d position, double feedforwardVoltage);
    public abstract void controlElbow(Rotation2d position, double feedforwardVoltage);

}
