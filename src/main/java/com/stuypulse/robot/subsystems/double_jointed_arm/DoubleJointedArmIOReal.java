package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class DoubleJointedArmIOReal implements DoubleJointedArmIO {
    // Hardware
    private final TalonFX frontShoulderMotor;
    private final TalonFX backShoulderMotor;
    private final CoreCANcoder shoulderEncoder;

    private final TalonFX elbowMotor;
    private final CoreCANcoder elbowEncoder;

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

        frontShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.FRONT_MOTOR);
        backShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.BACK_MOTOR);
        elbowMotor = new TalonFX(Ports.DoubleJointedArm.Elbow.MOTOR);

        shoulderEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Shoulder.ENCODER);
        elbowEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Elbow.ENCODER);
        
        // Configs
        Devices.DoubleJointedArm.Shoulder.motor_config.configure(frontShoulderMotor);
        Devices.DoubleJointedArm.Shoulder.motor_config.configure(backShoulderMotor); // Need different configs?
        shoulderEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.cc_config);

        backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));

        Devices.DoubleJointedArm.Elbow.motor_config.configure(elbowMotor);
        elbowEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Elbow.cc_config);

        // Status Signal initialization
        shoulderAngle = frontShoulderMotor.getPosition();
        shoulderAngularVel = frontShoulderMotor.getVelocity();
        shoulderAngularAccel = frontShoulderMotor.getAcceleration();
        shoulderAppliedVoltage = frontShoulderMotor.getMotorVoltage(); 
        shoulderCurrentAmps = frontShoulderMotor.getTorqueCurrent();   

        shoulderFollowerAngle = backShoulderMotor.getPosition();
        shoulderFollowerAngularVel = backShoulderMotor.getVelocity();
        shoulderFollowerAngularAccel = backShoulderMotor.getAcceleration();
        shoulderFollowerAppliedVoltage = backShoulderMotor.getMotorVoltage(); 
        shoulderFollowerCurrentAmps = backShoulderMotor.getTorqueCurrent();   

        elbowAngle = elbowMotor.getPosition();
        elbowAngularVel = elbowMotor.getVelocity();
        elbowAngularAccel = elbowMotor.getAcceleration();
        elbowAppliedVoltage = elbowMotor.getMotorVoltage(); 
        elbowCurrentAmps = elbowMotor.getTorqueCurrent();   

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

            
            shoulderFollowerAngle, shoulderFollowerAngularVel,
            shoulderFollowerAngularAccel, shoulderFollowerAppliedVoltage, shoulderFollowerCurrentAmps,

            elbowAngle, elbowAngularVel, elbowAngularAccel,
            elbowAppliedVoltage, elbowCurrentAmps
        );

        inputs.shoulderMotorConnected = frontShoulderMotor.isConnected();
        inputs.shoulderAngle = shoulderAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAngularVel = shoulderAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAngularAccel = shoulderAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderAppliedVoltage = shoulderAppliedVoltage.getValueAsDouble();
        inputs.shoulderCurrentAmps = shoulderCurrentAmps.getValueAsDouble();

        inputs.shoulderFollowerMotorConnected = backShoulderMotor.isConnected();
        inputs.shoulderFollowerAngle = shoulderFollowerAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAngularVel = shoulderFollowerAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAngularAccel = shoulderFollowerAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.shoulderFollowerAppliedVoltage = shoulderFollowerAppliedVoltage.getValueAsDouble();
        inputs.shoulderFollowerCurrentAmps = shoulderFollowerCurrentAmps.getValueAsDouble();

        inputs.elbowMotorConnected = elbowMotor.isConnected();
        inputs.elbowAngle = elbowAngle.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAngularVel = elbowAngularVel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAngularAccel = elbowAngularAccel.getValueAsDouble() * 2.0 * Math.PI;
        inputs.elbowAppliedVoltage = elbowAppliedVoltage.getValueAsDouble();
        inputs.elbowCurrentAmps = elbowCurrentAmps.getValueAsDouble();
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void controlShoulder(Rotation2d position, double feedforwardVoltage) {
        frontShoulderMotor.setControl(new MotionMagicVoltage(position.getRotations()).withFeedForward(feedforwardVoltage));
    }
    
    @Override
    public void controlElbow(Rotation2d position, double feedforwardVoltage) {
        elbowMotor.setControl(new MotionMagicVoltage(position.getRotations()).withFeedForward(feedforwardVoltage));
    }

    @Override
    public void runVoltageShoulder(double voltage) {
        frontShoulderMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void runVoltageElbow(double voltage) {
        elbowMotor.setControl(new VoltageOut(voltage));
    }
}