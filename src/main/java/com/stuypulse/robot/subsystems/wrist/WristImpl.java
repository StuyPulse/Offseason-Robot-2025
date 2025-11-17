package com.stuypulse.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristImpl extends Wrist {
    private final TalonFX rotationMotor;
    private final TalonFX rollerMotor;
    
    private final CoreCANcoder throughbore;

    private final Controller controller;

    public WristImpl() {
        super();
        rotationMotor = new TalonFX(Ports.Wrist.WRIST_MOTOR, Settings.CANIVORE);
        rollerMotor = new TalonFX(Ports.Wrist.ROLLER_MOTOR, Settings.CANIVORE);
        throughbore = new CoreCANcoder(Ports.Wrist.WRIST_ENCODER, Settings.CANIVORE);

        controller = new PIDController(Gains.Wrist.kP, Gains.Wrist.kI, Gains.Wrist.kD);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(throughbore.getAbsolutePosition().getValueAsDouble());
    }

    private boolean atTargetAngle() {
        return Math.abs(getAngle().getRadians() - state.getTargetAngle().getRadians()) < Settings.Wrist.ANGLE_TOLERANCE.getRadians();
    }

    public double getRPM() {
        return rollerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        rotationMotor.setVoltage(controller.update(state.getTargetAngle().getRadians(), getAngle().getRadians()));
        if (atTargetAngle()) {
            rollerMotor.set(state.getTargetRPM());
        }

        SmartDashboard.putNumber("Wrist/Current Angle (Deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist/Current RPM", getRPM());
    }
}
