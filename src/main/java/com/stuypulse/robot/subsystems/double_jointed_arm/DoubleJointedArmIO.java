package com.stuypulse.robot.subsystems.double_jointed_arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DoubleJointedArmIO {
    @AutoLog
    public class DoubleJointedArmIOInputs {
        public boolean shoulderMotorConnected = false;
        public boolean shoulderFollowerMotorConnected = false;
        public boolean elbowMotorConnected = false;

        public double shoulderAngle = 0.0;
        public double shoulderAngularVel = 0.0;
        public double shoulderAngularAccel = 0.0;
        public double shoulderAppliedVoltage = 0.0;
        public double shoulderCurrentAmps = 0.0;

        public double shoulderFollowerAngle = 0.0;
        public double shoulderFollowerAngularVel = 0.0;
        public double shoulderFollowerAngularAccel = 0.0;
        public double shoulderFollowerAppliedVoltage = 0.0;
        public double shoulderFollowerCurrentAmps = 0.0;

        public double elbowAngle = 0.0;
        public double elbowAngularAccel = 0.0;
        public double elbowAngularVel = 0.0;
        public double elbowAppliedVoltage = 0.0;
        public double elbowCurrentAmps = 0.0;
    }

    /* Updates logged data */
    public default void updateInputs(DoubleJointedArmIOInputs inputs) {};

    /* Control functions */
    public default void controlShoulder(Rotation2d position, double feedforwardVoltage) {};
    public default void controlElbow(Rotation2d position, double feedforwardVoltage) {};
    public default void runVoltageShoulder(double volts) {};
    public default void runVoltageElbow(double volts) {};

}