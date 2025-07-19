package com.stuypulse.robot.subsystems.double_jointed_arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public interface DoubleJointedArmIO {
    @AutoLog
    public class DoubleJointedArmIOInputs {
        public DoubleJointedArmIOData data = 
            new DoubleJointedArmIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record DoubleJointedArmIOData(
        boolean shoulderMotorConnected,
        boolean elbowMotorConnected,

        double shoulderAngle,
        double shoulderAngularVel,
        double shoulderAngularAccel,
        double shoulderAppliedVolts,
        double shoulderCurrentAmps,

        double elbowAngle,
        double elbowAngularAccel,
        double elbowAngularVel,
        double elbowAppliedVolts,
        double elbowCurrentAmps) {}

    // STATES
    public abstract void updateInputs(DoubleJointedArmIOInputs inputs);

    // CONTROL
    public abstract void runTorqueCurrentShoulder(double torqueCurrentAmps);
    public abstract void runTorqueCurrentElbow(double torqueCurrentAmps);
    public abstract void runVolts(double volts); // ONLY voltage override
    public abstract void setTargetAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle);
    public abstract void setPID(double kP, double kI, double kD);

    // BOOLEANS
    public abstract boolean isShoulderAtTarget();
    public abstract boolean isElbowAtTarget();
    public abstract boolean isArmAtTarget();

    // MATH
    public abstract Matrix<N2, N2> calculateMMatrix(); // Mass Inertia Matrix
    public abstract Matrix<N2, N2> calculateCMatrix(); // Centrifugal + Coriolis Matrix
    public abstract Matrix<N2, N1> calculateGMatrix(); // Torque due to Gravity Matrix
    public abstract Translation2d getEndPosition();

}
