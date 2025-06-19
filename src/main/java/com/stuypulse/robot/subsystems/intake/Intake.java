package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Intake extends SubsystemBase {
    
    public static final Intake instance = new IntakeImpl();

    // static {
    //     if (Robot.isReal()) {
    //         instance = new IntakeImpl();
    //     } else {
    //         // instance = new IntakeSim();
    //     }
    // }

    public static Intake getInstance() {
        return instance;
    }

    public enum PivotState {
        STOW(Settings.Intake.STOW_ANGLE),
        INTAKING(Settings.Intake.INTAKING_ANGLE);

        private Rotation2d targetAngle;

        private PivotState(Rotation2d targetAngle) {
            this.targetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(targetAngle.getDegrees(), Constants.Intake.MIN_ANGLE.getDegrees(), Constants.Intake.MAX_ANGLE.getDegrees()));
        }

        public Rotation2d getTargetAngle() {
            return this.targetAngle;
        }
    }

    public enum RollerState {
        INTAKING(Settings.Intake.INTAKING_SPEED),
        EJECTING(Settings.Intake.EJECTING_SPEED),
        STOP(0);

        private double speed;

        private RollerState(double speed) {
            this.speed = speed;
        }

        public double getTargetSpeed() {
            return this.speed;
        }
    }

    private PivotState pivotState;
    private RollerState rollerState;

    protected Intake() {
        this.pivotState = PivotState.STOW;
        this.rollerState = RollerState.STOP;
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public void setPivotState(PivotState state) {
        this.pivotState = state;
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    // public abstract SysIdRoutine getPivotSysIdRoutine();

    public abstract void zeroAngle();
    public abstract boolean atTargetAngle();

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/Pivot State", getPivotState().toString());
        SmartDashboard.putString("Intake/Roller State", getRollerState().toString());
    }

    }
