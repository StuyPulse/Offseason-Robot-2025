package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.booleans.filters.BFilter;
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
    private BStream isAgainstHardstop;

    public IntakeImpl() {
        rollerMotor = new TalonFX(Ports.Intake.ROLLER_MOTOR);
        pivotMotor = new TalonFX(Ports.Intake.PIVOT_MOTOR);

        Motors.Intake.ROLLER_MOTOR_CONFIG.configure(rollerMotor);
        Motors.Intake.PIVOT_MOTOR_CONFIG.configure(pivotMotor);

        isAgainstHardstop = BStream.create(() -> pivotMotor.getStatorCurrent().getValueAsDouble() > pivotMotor.getMotorStallCurrent().getValueAsDouble())
            .filtered(new BDebounceRC.Both(Settings.Intake.CURRENT_HOMING_DEBOUNCE_SECS));
    }

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

    @Override
    public void zeroAngle() {
        if (isAgainstHardstop.get() && getPivotState() == PivotState.STOW) {
            pivotMotor.setPosition(Constants.Intake.MIN_ANGLE.getRotations());
        } else if (isAgainstHardstop.get() && getPivotState() == PivotState.INTAKING) {
            pivotMotor.setPosition(Constants.Intake.MAX_ANGLE.getRotations());
        }
    }

    @Override
    public boolean atTargetAngle() {
        Rotation2d currentAngle = new Rotation2d(pivotMotor.getPosition().getValue());
        if (currentAngle.minus(getPivotState().getTargetAngle()).getDegrees() < Settings.Intake.ANGLE_TOLERANCE_DEG) {
            return true;
        } else return false;
    }

    @Override
    public void periodic() {
        super.periodic();

        zeroAngle();

        pivotMotor.setControl(new MotionMagicVoltage(getPivotState().getTargetAngle().getRotations()));
        rollerMotor.setControl(new DutyCycleOut(getRollerState().getTargetSpeed()));

        SmartDashboard.putNumber("Intake/Pivot/Target Angle (deg)", getPivotState().getTargetAngle().getDegrees());

    }

}
