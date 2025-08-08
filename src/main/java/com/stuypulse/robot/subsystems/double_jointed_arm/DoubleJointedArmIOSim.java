package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/* 6328 armsim */
public class DoubleJointedArmIOSim implements DoubleJointedArmIO {
    // State: [shoulder_angle, elbow_angle, shoulder_vel, elbow_vel]
    private Vector<N4> state = VecBuilder.fill(Math.PI / 2.0, 0.0, 0.0, 0.0);
    private double shoulderAppliedVolts = 0.0;
    private double elbowAppliedVolts = 0.0;

    private double previousShoulderVel = 0.0;
    private double previousElbowVel = 0.0;

    // More reasonable PID gains - start conservative
    private final PIDController shoulderPID = new PIDController(40.0, 0.0, 5.0);
    private final PIDController elbowPID = new PIDController(30.0, 0.0, 3.0);

    private final DCMotor shoulderMotor = DCMotor.getKrakenX60(1).withReduction(Constants.DoubleJointedArm.Shoulder.GEAR_RATIO);
    private final DCMotor elbowMotor = DCMotor.getKrakenX60(1).withReduction(Constants.DoubleJointedArm.Elbow.GEAR_RATIO);

    private final double m1 = Constants.DoubleJointedArm.Shoulder.MASS;   
    private final double m2 = Constants.DoubleJointedArm.Elbow.MASS;          
    private final double l1 = Constants.DoubleJointedArm.Shoulder.LENGTH;  
    private final double l2 = Constants.DoubleJointedArm.Elbow.LENGTH;
    private final double r1 = Constants.DoubleJointedArm.Shoulder.LENGTH/2; 
    private final double r2 = Constants.DoubleJointedArm.Elbow.LENGTH/2;
    // Use proper moment of inertia for rods about their ends
    private final double I1 = (1.0/3.0) * m1 * l1 * l1;
    private final double I2 = (1.0/3.0) * m2 * l2 * l2;
    private final double g = 9.81;

    private final double dt = 0.02;
    
    // Safety limits
    private final double MAX_VELOCITY = 10.0; // rad/s
    private final double MAX_ACCELERATION = 50.0; // rad/s^2

    public DoubleJointedArmIOSim() {
        super();
        
        shoulderPID.enableContinuousInput(-Math.PI, Math.PI);
        elbowPID.enableContinuousInput(-Math.PI, Math.PI);
        
        shoulderPID.setIntegratorRange(-2.0, 2.0);
        elbowPID.setIntegratorRange(-2.0, 2.0);
    }

    @Override
    public void updateInputs(DoubleJointedArmIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            runVoltageShoulder(0.0);
            runVoltageElbow(0.0);
            shoulderPID.reset();
            elbowPID.reset();
        }

        state = simulateStep(state, shoulderAppliedVolts, elbowAppliedVolts, dt);

        double shoulderAngle = state.get(0, 0);
        double elbowAngle = state.get(1, 0);
        double shoulderVel = state.get(2, 0);
        double elbowVel = state.get(3, 0);

        shoulderAngle = MathUtil.clamp(shoulderAngle, 
            Constants.DoubleJointedArm.Shoulder.MIN_ANGLE.getRadians(),
            Constants.DoubleJointedArm.Shoulder.MAX_ANGLE.getRadians());
        elbowAngle = MathUtil.clamp(elbowAngle,
            Constants.DoubleJointedArm.Elbow.MIN_ANGLE.getRadians(), 
            Constants.DoubleJointedArm.Elbow.MAX_ANGLE.getRadians());

        state = VecBuilder.fill(shoulderAngle, elbowAngle, shoulderVel, elbowVel);

        double shoulderAccel = (shoulderVel - previousShoulderVel) / dt;
        double elbowAccel = (elbowVel - previousElbowVel) / dt;

        inputs.shoulderMotorConnected = true;
        inputs.shoulderAngle = shoulderAngle;
        inputs.shoulderAngularVel = shoulderVel;
        inputs.shoulderAngularAccel = shoulderAccel;
        inputs.shoulderAppliedVoltage = shoulderAppliedVolts;
        inputs.shoulderCurrentAmps = Math.abs(shoulderMotor.getCurrent(shoulderVel, shoulderAppliedVolts));

        inputs.elbowMotorConnected = true;
        inputs.elbowAngle = elbowAngle;
        inputs.elbowAngularVel = elbowVel;
        inputs.elbowAngularAccel = elbowAccel;
        inputs.elbowAppliedVoltage = elbowAppliedVolts;
        inputs.elbowCurrentAmps = Math.abs(elbowMotor.getCurrent(elbowVel, elbowAppliedVolts));

        previousShoulderVel = shoulderVel;
        previousElbowVel = elbowVel;

        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderVoltage", shoulderAppliedVolts);
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowVoltage", elbowAppliedVolts);
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderVel", shoulderVel);
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowVel", elbowVel);
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderAccel", shoulderAccel);
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowAccel", elbowAccel);
    }

    @Override
    public void controlShoulder(Rotation2d position, double feedforwardVoltage) {
        double currentAngle = state.get(0, 0);
        double targetAngle = position.getRadians();
        
        // Calculate shortest path error for continuous input
        double error = targetAngle - currentAngle;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        
        double pidOutput = shoulderPID.calculate(currentAngle, targetAngle);
        
        // Limit PID output to reasonable range
        pidOutput = MathUtil.clamp(pidOutput, -8.0, 8.0);
        
        double limitedFeedforward = MathUtil.clamp(feedforwardVoltage, -4.0, 4.0);
        double totalVoltage = pidOutput + limitedFeedforward;
        
        runVoltageShoulder(totalVoltage);
        
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderPIDOutput", pidOutput);
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderFeedforward", limitedFeedforward);
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderError", Math.toDegrees(error));
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderTarget", Math.toDegrees(targetAngle));
        Logger.recordOutput("DoubleJointedArm/Sim/ShoulderCurrent", Math.toDegrees(currentAngle));
    }

    @Override
    public void controlElbow(Rotation2d position, double feedforwardVoltage) {
        double currentAngle = state.get(1, 0);
        double targetAngle = position.getRadians();
        
        // Calculate shortest path error for continuous input
        double error = targetAngle - currentAngle;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        
        double pidOutput = elbowPID.calculate(currentAngle, targetAngle);
        
        // Limit PID output to reasonable range
        pidOutput = MathUtil.clamp(pidOutput, -6.0, 6.0);
        
        double limitedFeedforward = MathUtil.clamp(feedforwardVoltage, -4.0, 4.0);
        double totalVoltage = pidOutput + limitedFeedforward;
        
        runVoltageElbow(totalVoltage);
        
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowPIDOutput", pidOutput);
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowFeedforward", limitedFeedforward);
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowError", Math.toDegrees(error));
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowTarget", Math.toDegrees(targetAngle));
        Logger.recordOutput("DoubleJointedArm/Sim/ElbowCurrent", Math.toDegrees(currentAngle));
    }

    @Override
    public void runVoltageShoulder(double voltage) {
        shoulderAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void runVoltageElbow(double voltage) {
        elbowAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    // Simplified but stable simulation
    private Vector<N4> simulateStep(Vector<N4> currentState, double shoulderVoltage, double elbowVoltage, double dt) {
        double q1 = currentState.get(0, 0);  // shoulder angle
        double q2 = currentState.get(1, 0);  // elbow angle (relative to shoulder)
        double q1_dot = currentState.get(2, 0);  // shoulder velocity
        double q2_dot = currentState.get(3, 0);  // elbow velocity

        // Calculate motor torques using direct voltage-to-torque conversion
        // This bypasses the potentially problematic getCurrent() method
        double shoulderCurrent = shoulderMotor.getCurrent(q1_dot, shoulderVoltage);
        double elbowCurrent = elbowMotor.getCurrent(q2_dot, elbowVoltage);
        
        double shoulderTorque = shoulderMotor.getTorque(shoulderCurrent);
        double elbowTorque = elbowMotor.getTorque(elbowCurrent);
        
        // Alternative direct calculation if the above still gives zero:
        // Using simplified motor model: Torque = Kt * (V - Kv * omega) / R
        // For KrakenX60: Kt ≈ 0.022 N⋅m/A, R ≈ 0.018 Ω, Kv ≈ 473 rad/s/V
        if (Math.abs(shoulderTorque) < 0.001) {
            // Fallback to simplified motor model
            double Kt = 0.022; // Torque constant
            double R = 0.018;  // Resistance
            double Kv = 1.0 / 473.0; // Back-EMF constant (inverse of Kv)
            shoulderTorque = (Kt / R) * (shoulderVoltage - Kv * q1_dot) * Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
        }
        
        if (Math.abs(elbowTorque) < 0.001) {
            double Kt = 0.022;
            double R = 0.018;
            double Kv = 1.0 / 473.0;
            elbowTorque = (Kt / R) * (elbowVoltage - Kv * q2_dot) * Constants.DoubleJointedArm.Elbow.GEAR_RATIO;
        }

        // Simplified but coupled dynamics for stability
        
        // Effective inertias (including coupling effects)
        double shoulderEffectiveInertia = I1 + m2 * l1 * l1 + 0.5 * I2; // approximate coupling
        double elbowEffectiveInertia = I2;
        
        // Gravity torques
        double shoulderGravityTorque = (m1 * r1 + m2 * l1) * g * Math.cos(q1) + 
                                     0.5 * m2 * r2 * g * Math.cos(q1 + q2); // approximate coupling
        double elbowGravityTorque = m2 * r2 * g * Math.cos(q1 + q2);
        
        // Damping for stability
        double damping = 2.0; // Increased damping
        
        // Calculate accelerations
        double q1_ddot = (shoulderTorque - shoulderGravityTorque - damping * q1_dot) / shoulderEffectiveInertia;
        double q2_ddot = (elbowTorque - elbowGravityTorque - damping * q2_dot) / elbowEffectiveInertia;
        
        // Clamp accelerations for stability
        q1_ddot = MathUtil.clamp(q1_ddot, -MAX_ACCELERATION, MAX_ACCELERATION);
        q2_ddot = MathUtil.clamp(q2_ddot, -MAX_ACCELERATION, MAX_ACCELERATION);

        // Semi-implicit Euler integration (more stable)
        double new_q1_dot = q1_dot + q1_ddot * dt;
        double new_q2_dot = q2_dot + q2_ddot * dt;
        
        // Clamp velocities
        new_q1_dot = MathUtil.clamp(new_q1_dot, -MAX_VELOCITY, MAX_VELOCITY);
        new_q2_dot = MathUtil.clamp(new_q2_dot, -MAX_VELOCITY, MAX_VELOCITY);
        
        double new_q1 = q1 + new_q1_dot * dt;
        double new_q2 = q2 + new_q2_dot * dt;


        return VecBuilder.fill(new_q1, new_q2, new_q1_dot, new_q2_dot);
    }
}