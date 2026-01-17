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

    private final SparkFlex rollerMotor1;
    private final SparkFlex rollerMotor2;
    private final RelativeEncoder rollerMotor1Encoder;
    private final RelativeEncoder rollerMotor2Encoder;

    public NeoIntakeImpl() {
        super();

        rollerMotor1 = new SparkFlex(Ports.Intakes.NeoIntake.ROLLER, MotorType.kBrushless);
        rollerMotor1.configure(Devices.Intakes.NeoIntake.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotor2 = new SparkFlex(Ports.Intakes.NeoIntake.ROLLER2, MotorType.kBrushless);
        rollerMotor2.configure(Devices.Intakes.NeoIntake.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotor1Encoder = rollerMotor1.getEncoder();
        rollerMotor2Encoder = rollerMotor2.getEncoder();
    }

    private void setMotorBasedOnState() {
        double speed = MathUtil.clamp(state.getIntakeRollerSpeed(), -1.0, 1.0);
        rollerMotor1.set(speed);
        rollerMotor2.set(-speed);
    }

    private double getAvgIntakeRPM() {
        return (rollerMotor1Encoder.getVelocity() + rollerMotor2Encoder.getVelocity()) / 2;
    }

    private double getBusVoltage1() {
        return rollerMotor1.getBusVoltage();
    }

    private double getBusVoltage2() {
        return rollerMotor2.getBusVoltage();
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Current RPM (Average of two encoders)", getAvgIntakeRPM());
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Bus Voltage (Motor 1)", getBusVoltage1());
        SmartDashboard.putNumber("Intake/Neo Intake/Rollers/Bus Voltage (Motor 2)", getBusVoltage2());
        SmartDashboard.putString("Intake/Neo Intake/Rollers/Current State", getState().toString());
    }

}
