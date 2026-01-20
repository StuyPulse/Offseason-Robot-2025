package com.stuypulse.robot.subsystems.krakenintake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenIntakeImpl extends KrakenIntake {

    private final TalonFX rollerMotor;

    public KrakenIntakeImpl(int krakenID) {
        super();

        rollerMotor = new TalonFX(krakenID, Settings.CANIVORE);
        Devices.Intakes.KrakenIntake.motorConfig.configure(rollerMotor);
    }

    private void setMotorBasedOnState() {
        double speed = MathUtil.clamp(state.getKrakenIntakeRollerSpeed(), -1.0, 1.0);
        rollerMotor.set(speed);
    }

    private double getIntakeRPM() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    private double getSupplyVoltage() {
        return rollerMotor.getSupplyVoltage().getValueAsDouble();
    }

    private double getMotorVoltage() {
        return rollerMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
        SmartDashboard.putNumber("Intake/Kraken Intake/Rollers/Current RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Supply Voltage", getSupplyVoltage());
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Motor Voltage", getMotorVoltage());
        SmartDashboard.putString("Intake/Kraken Intake/Rollers/Current State", getState().toString());
    }

}
