package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DoubleJointedArmIOReal implements DoubleJointedArmIO {
    // Hardware
    private final TalonFX shoulder;
    private final CANcoder shoulderEncoder;
    private final TalonFX shoulderFollower;

    private final TalonFX elbow;
    private final CANcoder elbowEncoder;

    // Status Signals (refreshed periodically upon initialization)
    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<AngularVelocity> shoulderAngularVel;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccel;
    private final StatusSignal<Voltage> shoulderAppliedVoltage;
    private final StatusSignal<Current> shoulderCurrentAmps;

    private final StatusSignal<Angle> shoulderFollowerAngle;
    private final StatusSignal<AngularVelocity> shoulderFollowerAngularVel;
    private final StatusSignal<AngularAcceleration> shoulderFollowerAngularAccel;
    private final StatusSignal<Voltage> shoulderFollowerAppliedVoltage;
    private final StatusSignal<Current> shoulderFollowerCurrentAmps;

    private final StatusSignal<Angle> elbowAngle;
    private final StatusSignal<AngularVelocity> elbowAngularVel;
    private final StatusSignal<AngularAcceleration> elbowAngularAccel;
    private final StatusSignal<Voltage> elbowAppliedVoltage;
    private final StatusSignal<Current> elbowCurrentAmps;


    public DoubleJointedArmIOReal() {
        super();
        shoulder = new TalonFX(Ports.DoubleJointedArm.SHOULDER_MOTOR);
        shoulderEncoder = new CANcoder(Ports.DoubleJointedArm.SHOULDER_ENCODER);
        shoulderFollower = new TalonFX(Ports.DoubleJointedArm.SHOULDER_FOLLOWER);

        elbow = new TalonFX(Ports.DoubleJointedArm.ELBOW_MOTOR);
        elbowEncoder = new CANcoder(Ports.DoubleJointedArm.ELBOW_ENCODER);

        // Configs
        Devices.DoubleJointedArm.Shoulder.motor_config.configure(shoulder);
        Devices.DoubleJointedArm.Shoulder.motor_followerConfig.configure(shoulderFollower);
        shoulderEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.cc_config);
        Devices.DoubleJointedArm.Elbow.motor_config.configure(elbow);
        elbowEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Elbow.cc_config);

        shoulderFollower.setControl(
                new Follower(Ports.DoubleJointedArm.SHOULDER_MOTOR, false));

        // Status Signal initialization
        shoulderAngle = shoulder.getPosition();
        shoulderAngularVel = shoulder.getVelocity();
        shoulderAngularAccel = shoulder.getAcceleration();
        shoulderAppliedVoltage = shoulder.getMotorVoltage();
        shoulderCurrentAmps = shoulder.getTorqueCurrent();

        shoulderFollowerAngle = shoulderFollower.getPosition();
        shoulderFollowerAngularVel = shoulderFollower.getVelocity();
        shoulderFollowerAngularAccel = shoulderFollower.getAcceleration();
        shoulderFollowerAppliedVoltage = shoulderFollower.getMotorVoltage();
        shoulderFollowerCurrentAmps = shoulderFollower.getTorqueCurrent();

        elbowAngle = elbow.getPosition();
        elbowAngularVel = elbow.getVelocity();
        elbowAngularAccel = elbow.getAcceleration();
        elbowAppliedVoltage = elbow.getMotorVoltage();
        elbowCurrentAmps = elbow.getTorqueCurrent();

        // Set update freqs
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            shoulderAngle,
            shoulderAngularVel,
            shoulderAngularAccel,
            shoulderAppliedVoltage,

            shoulderFollowerAngle,
            shoulderFollowerAngularVel,
            shoulderFollowerAngularAccel,
            shoulderFollowerAppliedVoltage,

            elbowAngle,
            elbowAngularVel,
            elbowAngularAccel,
            elbowAppliedVoltage);

        // Higher framerate recommended
        shoulderCurrentAmps.setUpdateFrequency(250);
        shoulderFollowerCurrentAmps.setUpdateFrequency(250);
        elbowCurrentAmps.setUpdateFrequency(250);
    }

    @Override
    public void updateInputs(DoubleJointedArmIOInputs inputs) { // called periodically in subsystem
        // Clear cache
        BaseStatusSignal.refreshAll(
            shoulderAngle, shoulderAngularVel, shoulderAngularAccel,
            shoulderAppliedVoltage, shoulderCurrentAmps,
            
            shoulderFollowerAngle, shoulderFollowerAngularVel,
            shoulderFollowerAngularAccel, shoulderFollowerAppliedVoltage, shoulderFollowerCurrentAmps,

            elbowAngle, elbowAngularVel, elbowAngularAccel,
            elbowAppliedVoltage, elbowCurrentAmps
        );

        inputs.shoulderMotorConnected = shoulder.isConnected();
        inputs.shoulderAngle = shoulderAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAngularVel = shoulderAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAngularAccel = shoulderAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAppliedVoltage = shoulderAppliedVoltage.getValueAsDouble();
        inputs.shoulderCurrentAmps = shoulderCurrentAmps.getValueAsDouble();

        inputs.shoulderFollowerMotorConnected = shoulderFollower.isConnected();
        inputs.shoulderFollowerAngle = shoulderFollowerAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAngularVel = shoulderFollowerAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAngularAccel = shoulderFollowerAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAppliedVoltage = shoulderFollowerAppliedVoltage.getValueAsDouble();
        inputs.shoulderFollowerCurrentAmps = shoulderFollowerCurrentAmps.getValueAsDouble();

        inputs.elbowMotorConnected = elbow.isConnected();
        inputs.elbowAngle = elbowAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAngularVel = elbowAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAngularAccel = elbowAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAppliedVoltage = elbowAppliedVoltage.getValueAsDouble();
        inputs.elbowCurrentAmps = elbowCurrentAmps.getValueAsDouble();
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderAngle.getValueAsDouble());
    }

    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowAngle.getValueAsDouble());
    }

    @Override
    public void controlShoulder(Rotation2d position, double feedforwardVoltage) {
        shoulder.setControl(new MotionMagicVoltage(position.getRotations()).withFeedForward(feedforwardVoltage));
    }
    
    @Override
    public void controlElbow(Rotation2d position, double feedforwardVoltage) {
        elbow.setControl(new MotionMagicVoltage(position.getRotations()).withFeedForward(feedforwardVoltage));
    }

    @Override
    public void runVoltageShoulder(double voltage) {
        shoulder.setControl(new VoltageOut(voltage));
    }

    @Override
    public void runVoltageElbow(double voltage) {
        elbow.setControl(new VoltageOut(voltage));
    }
}