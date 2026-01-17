package com.stuypulse.robot.subsystems.neointake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NeoIntakeImpl extends NeoIntake {

    private final SparkFlex rollerMotor;
    private final RelativeEncoder rollerMotorEncoder;

    public NeoIntakeImpl() {
        super();

        rollerMotor = new SparkFlex(Ports.Intakes.NeoIntake.ROLLER, MotorType.kBrushless);
        rollerMotor.configure(Devices.Intakes.NeoIntake.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotorEncoder = rollerMotor.getEncoder();
    }

    private void setMotorBasedOnState() {
        double speed = MathUtil.clamp(state.getIntakeRollerSpeed(), -1.0, 1.0);
        rollerMotor.set(speed);
    }

    private double getIntakeRPM() {
        return rollerMotorEncoder.getVelocity();
    }

    private double getBusVoltage() {
        return rollerMotor.getBusVoltage();
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Current RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Bug Voltage", getBusVoltage());
        SmartDashboard.putString("Intake/Neo Intake/Rollers/Current State", getState().toString());
    }

}
