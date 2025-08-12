package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    private final String name;
    private final Translation2d offset;

    private SwerveModuleState targetState;

    public SwerveModule(String name, Translation2d offset) {
        this.name = name;
        this.offset = offset;

        targetState = new SwerveModuleState();
    }

    @Override
    public final String getName() {
        return this.name;
    }

    public final Translation2d getModuleOffset() {
        return this.offset;
    }

    public abstract double getVelocity();

    public abstract Rotation2d getAngle();

    public abstract SwerveModulePosition getModulePosition();

    public final SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(getAngle());
        targetState = state;
    }

    public final SwerveModuleState getTargetState() {
        return targetState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Velocity", getVelocity());
    }
}