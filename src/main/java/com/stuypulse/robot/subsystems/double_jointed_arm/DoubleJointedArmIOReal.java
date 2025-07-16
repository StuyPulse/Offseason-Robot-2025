package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DoubleJointedArmIOReal implements DoubleJointedArmIO {
    
    // Hardware
    private TalonFX shoulder;
    private CANcoder shoulderEncoder;

    private TalonFX elbow;
    private CANcoder elbowEncoder;

    // Matricies
    private Matrix<N2, N2> mMatrix;
    private Matrix<N2, N2> cMatrix;
    private Matrix<N2, N1> gMatrix;

    // Status Signals
    private StatusSignal<Angle> shoulderAngle;
    private StatusSignal<AngularVelocity> shoulderAngularVel;
    private StatusSignal<AngularAcceleration> shoulderAngularAccel;
    private StatusSignal<Voltage> shoulderAppliedVoltage;
    private StatusSignal<Current> shoulderCurrentAmps;

    private StatusSignal<Angle> elbowAngle;
    private StatusSignal<AngularVelocity> elbowAngularVel;
    private StatusSignal<AngularAcceleration> elbowAngularAccel;
    private StatusSignal<Voltage> elbowAppliedVoltage;
    private StatusSignal<Current> elbowCurrentAmps;

    public DoubleJointedArmIOReal() {
        super();
        shoulder = new TalonFX(Ports.DoubleJointedArm.SHOULDER_MOTOR);
        shoulderEncoder = new CANcoder(Ports.DoubleJointedArm.SHOULDER_ENCODER);
        elbow = new TalonFX(Ports.DoubleJointedArm.ELBOW_MOTOR);
        elbowEncoder = new CANcoder(Ports.DoubleJointedArm.ELBOW_ENCODER);

        /* CONFIGS GO HERE */
        Devices.DoubleJointedArm.Shoulder.motor_config.configure(shoulder);
        shoulderEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.cc_config);

        Devices.DoubleJointedArm.Elbow.motor_config.configure(elbow);
        elbowEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Elbow.cc_config);

        shoulderAngle = shoulder.getPosition();
        shoulderAngularVel = shoulder.getVelocity();
        shoulderAngularAccel = shoulder.getAcceleration();
        shoulderAppliedVoltage = shoulder.getMotorVoltage();
        shoulderCurrentAmps = shoulder.getTorqueCurrent();

        elbowAngle = elbow.getPosition();
        elbowAngularVel = elbow.getVelocity();
        elbowAngularAccel = elbow.getAcceleration();
        elbowAppliedVoltage = elbow.getMotorVoltage();
        elbowCurrentAmps = elbow.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            shoulderAngle,
            shoulderAngularVel,
            shoulderAngularAccel,
            shoulderAppliedVoltage,
            shoulderCurrentAmps,

            elbowAngle,
            elbowAngularVel,
            elbowAngularAccel,
            elbowAppliedVoltage,
            elbowCurrentAmps);
        shoulderCurrentAmps.setUpdateFrequency(250);
        elbowCurrentAmps.setUpdateFrequency(250);
    }

}
