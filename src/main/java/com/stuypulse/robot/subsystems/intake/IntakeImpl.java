package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {
    
    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private DigitalInput bumpSwitch;
    private Encoder relativeEncoder;

    // @Override
    // public SysIdRoutine getPivotSysIdRoutine() {
    //     return SysId.getRoutine(
    //         1, 
    //         5, 
    //         "Intake Pivot", 
    //         voltage -> setPivotVoltageOverride(Optional.of(voltage)), 
    //         () -> getCurrentAngle().getRotations(), 
    //         () -> pivotMotor.getVelocity().getValueAsDouble(), 
    //         () -> pivotMotor.getMotorVoltage().getValueAsDouble(), 
    //         getInstance()
    //     );
    // }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getPivotState().getTargetAngle().getDegrees(),
                Constants.Intake.MIN_ANGLE.getDegrees(),
                Constants.Intake.MAX_ANGLE.getDegrees()));
    }

    @Override
    public void zeroAngle() {

        if (bumpSwitch.get()) {
            relativeEncoder.reset();
        }

    }

    @Override
    public boolean isAtIntakeAngle() {

        if (relativeEncoder.getCurrentAngle() == Constants.Intake.INTAKING_ANGLE)

    }

    @Override
    public void periodic() {

        super.periodic();
        SmartDashboard.putNumber("Intake/Pivot/Target Angle (deg)", getTargetAngle().getDegrees());

    }

}
