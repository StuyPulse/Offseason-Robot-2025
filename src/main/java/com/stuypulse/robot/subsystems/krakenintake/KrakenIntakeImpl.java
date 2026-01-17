package com.stuypulse.robot.subsystems.krakenintake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenIntakeImpl extends KrakenIntake {

    private final TalonFX rollerMotor;
    // private final RelativeEncoder rollerMotorEncoder;

    public KrakenIntakeImpl() {
        super();

        rollerMotor = new TalonFX(Ports.Intakes.KrakenIntake.ROLLER);
        Devices.Intakes.KrakenIntake.motorConfig.configure(rollerMotor);

        //rollerMotorEncoder = rollerMotor.getEncoder();
    }

    private void setMotorBasedOnState() {
        double speed = MathUtil.clamp(state.getKrakenIntakeRollerSpeed(), -1.0, 1.0);
        rollerMotor.set(speed);
    }

    // private double getIntakeRPM() {
    //     return rollerMotorEncoder.getVelocity();
    // }

    @Override
    public void periodic() {
        setMotorBasedOnState();
        // SmartDashboard.putNumber("Intake/Kraken Intake/Rollers/Current RPM", getIntakeRPM());
        SmartDashboard.putString("Intake/Kraken Intake/Rollers/Current State", getState().toString());
    }

}
