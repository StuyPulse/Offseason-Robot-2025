package com.stuypulse.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends Elevator {
    private final TalonFX elevatorMotor;

    private double targetHeight;

    public ElevatorImpl() {
        elevatorMotor = new TalonFX(0, "*");
        Motors.Elevator.elevatorConfig.configure(elevatorMotor);

        targetHeight = Constants.Elevator.MIN_HEIGHT;
    }

    public double getCurrentHeight() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public void setTargetHeight(double newHeight) {
        targetHeight = SLMath.clamp(newHeight, Constants.Elevator.MIN_HEIGHT, Constants.Elevator.MAX_HEIGHT);
    }

    public boolean isAtTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE;
    }

    @Override
    public void periodic() {
        elevatorMotor.setControl(new MotionMagicVoltage(getTargetHeight()));

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator/Target Height", getTargetHeight());
    }
}
