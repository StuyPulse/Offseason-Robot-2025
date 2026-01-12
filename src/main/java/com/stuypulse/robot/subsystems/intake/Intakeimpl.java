package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intakeimpl extends Intake {
    private final TalonFX funnelMotor, rollerMotor;
    private SmartNumber setfunnelspeed, setRollerSpeed;

    public Intakeimpl() {
        funnelMotor = new TalonFX(Ports.Intake.FUNNEL, "CANIVORE");
        Devices.Funnel.motor_config.configure(funnelMotor);

        rollerMotor = new TalonFX(Ports.Intake.ROLLER, "CANIVORE");
        Devices.Roller.motor_config.configure(rollerMotor);
    }

    public double getFunnelRPM() {
        return funnelMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getRollerRPM() {
        return rollerMotor.getVelocity().getValueAsDouble() * 60;
    }
    
    @Override
    public void periodic() {
        funnelMotor.setControl(new DutyCycleOut(setfunnelspeed.getAsDouble()));
        rollerMotor.setControl(new DutyCycleOut(setRollerSpeed.getAsDouble()));

        SmartDashboard.putNumber("Intake/Roller RPM", getRollerRPM());
        SmartDashboard.putNumber("Intake/Funnel RPM", getFunnelRPM());
    }
    
}
