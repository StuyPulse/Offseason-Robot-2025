package com.stuypulse.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristImpl extends Wrist {
    private TalonFX rotationMotor;
    private TalonFX rollerMotor;
    
    private CoreCANcoder throughbore;

    private Controller controller;

    public WristImpl() {
        super();
        rotationMotor = new TalonFX(Ports.Wrist.WRIST_MOTOR, Settings.CANIVORE);
        throughbore = new CoreCANcoder(Ports.Wrist.WRIST_ENCODER, Settings.CANIVORE);

        controller = new PIDController(Gains.Wrist.kP, Gains.Wrist.kI, Gains.Wrist.kD);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(throughbore.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        super.periodic();

        rotationMotor.setVoltage(controller.update(state.getTargetAngle().getRadians(), getAngle().getRadians()));
    }
}
