package com.stuypulse.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Devices;
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
    
    private final CANcoder throughbore;

    private Controller controller;

    private boolean lastIntegratorState = false;

    public WristImpl() {
        super();
        rotationMotor = new TalonFX(Ports.Wrist.WRIST_MOTOR, Settings.CANIVORE);
        rollerMotor = new TalonFX(Ports.Wrist.ROLLER_MOTOR, Settings.CANIVORE);
        throughbore = new CANcoder(Ports.Wrist.WRIST_ENCODER, Settings.CANIVORE);

        rotationMotor.getConfigurator().apply(Devices.Wrist.Rotation.getConfig());
        
        controller = new PIDController(Gains.Wrist.kP, 0f, Gains.Wrist.kD);
    
        rotationMotor.setPosition(throughbore.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees((360f*(rotationMotor.getPosition().getValueAsDouble() * Constants.Wrist.GEAR_RATIO + Constants.Wrist.ENCODER_OFFSET_ROT))%360f-180);
    }

    private boolean atTargetAngle() {
        return Math.abs(getAngle().getRadians() - state.getTargetAngle().getRadians()) < Settings.Wrist.ANGLE_TOLERANCE.getRadians();
    }

    private boolean withinIntegratorRange() {
        return Math.abs(getAngle().getRadians() - state.getTargetAngle().getRadians()) < Settings.Wrist.INTEGRATOR_TOLERANCE.getRadians();
    }
    
    public double getRPM() {
        return rollerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public void resetController(boolean forIntegration) {
        controller = new PIDController(Gains.Wrist.kP, forIntegration?Gains.Wrist.kI:0f , Gains.Wrist.kD);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(withinIntegratorRange() && lastIntegratorState == false) {
            lastIntegratorState = true;
            resetController(true);
        }
        else if(!withinIntegratorRange() && lastIntegratorState == true) {
            lastIntegratorState = false;
            resetController(false);
        }

        double volts = controller.update(state.getTargetAngle().getRadians(), getAngle().getRadians());        

        rotationMotor.setVoltage(volts);
        if (atTargetAngle()) {
            rollerMotor.set(state.getTargetRPM()/2000);
        }

        SmartDashboard.putNumber("Wrist/Current Angle (Deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist/Current RPM", getRPM());
        SmartDashboard.putBoolean("Wrist/Integrating?", lastIntegratorState);
        SmartDashboard.putNumber("Wrist/Calculated Voltage", volts);
    }
}
