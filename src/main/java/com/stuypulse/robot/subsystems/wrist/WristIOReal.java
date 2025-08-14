package com.stuypulse.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class WristIOReal implements WristIO {
    private final TalonFX wrist;
    private final CANcoder encoder;

    private final StatusSignal<Angle> wristAngle;
    private final StatusSignal<AngularVelocity> wristAngularVel;
    private final StatusSignal<AngularAcceleration> wristAngularAccel;
    private final StatusSignal<Voltage> wristAppliedVoltage;
    private final StatusSignal<Current> wristCurrentAmps;

    public WristIOReal() {
        super();
        wrist = new TalonFX(Ports.Wrist.WRIST_MOTOR, "CANIVORE");
        Devices.Wrist.motor_config.configure(wrist);

        encoder = new CANcoder(Ports.Wrist.WRIST_ENCODER, "CANIVORE");
        encoder.getConfigurator().apply(Devices.Wrist.cc_config);

        wrist.setPosition(encoder.getAbsolutePosition().getValueAsDouble()); // zero position

        wristAngle = wrist.getPosition();
        wristAngularVel = wrist.getVelocity();
        wristAngularAccel = wrist.getAcceleration();
        wristAppliedVoltage = wrist.getMotorVoltage();
        wristCurrentAmps = wrist.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            wristAngle,
            wristAngularVel,
            wristAngularAccel,
            wristAppliedVoltage
        );

        wristCurrentAmps.setUpdateFrequency(250);
    }

    @Override 
    public void updateInputs(WristIOInputs inputs) {
        // Clear cache
        BaseStatusSignal.refreshAll(
            wristAngle, wristAngularVel, wristAngularAccel,
            wristAppliedVoltage, wristCurrentAmps
        );

        inputs.wristMotorConnected = wrist.isConnected();
        inputs.wristAngle = wrist.getPosition().getValueAsDouble() * 360;
        inputs.wristAngularVel = wrist.getVelocity().getValueAsDouble() * 360;
        inputs.wristAngularAccel = wrist.getAcceleration().getValueAsDouble() * 360;
        inputs.wristAppliedVoltage = wrist.getMotorVoltage().getValueAsDouble();
        inputs.wristCurrentAmps = wrist.getTorqueCurrent().getValueAsDouble();
    }

    public Rotation2d getWristAngle() { 
        return Rotation2d.fromRotations(wristAngle.getValueAsDouble()); 
    }

    @Override
    public void controlWrist(Rotation2d position) {
        wrist.setControl(new MotionMagicVoltage(position.getRotations())); // need to add feedforward
    }
}
