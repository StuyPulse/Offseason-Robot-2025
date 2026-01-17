package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {

    private final SparkFlex rollerMotor;
    private final RelativeEncoder rollerMotorEncoder;

    public IntakeImpl() {
        super();

        rollerMotor = new SparkFlex(Ports.Intake.ROLLER, MotorType.kBrushless);
        rollerMotor.configure(Devices.Intake.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotorEncoder = rollerMotor.getEncoder();
    }

    private void setMotorBasedOnState() {
        double speed = MathUtil.clamp(state.getIntakeRollerSpeed(), 0.0, 1.0);
        rollerMotor.set(speed * (state.getIntakeReversed() ? -1 : 1));
    }

    private double getIntakeRPM() {
        return rollerMotorEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
        SmartDashboard.putNumber("Intake/Rollers/Current RPM", getIntakeRPM());
        SmartDashboard.putString("Intake/Rollers/Current State", getState().toString());
    }

}
