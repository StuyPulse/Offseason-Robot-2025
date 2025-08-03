package com.stuypulse.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    WristIO io;
    WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged(); 
    WristState state;

    public Wrist(WristIO io) {
        this.io = io;
        state = WristState.STOW;
    }

    public enum WristState {
        STOW(Rotation2d.fromDegrees(10)),
        INT(Rotation2d.fromDegrees(90)),
        L4_FRONT(Rotation2d.fromDegrees(10)),
        L4_BACK(Rotation2d.fromDegrees(10)),
        L3_FRONT(Rotation2d.fromDegrees(10)),
        L3_BACK(Rotation2d.fromDegrees(10)),
        L2_FRONT(Rotation2d.fromDegrees(10)),
        L2_BACK(Rotation2d.fromDegrees(10)),
        L1_FRONT(Rotation2d.fromDegrees(10)),
        L1_BACK(Rotation2d.fromDegrees(10));

        private WristState(Rotation2d wristAngle){
            this.wristAngle = wristAngle;
        }

        private Rotation2d wristAngle;

        public Rotation2d getWristTarget() { return wristAngle; }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        Logger.recordOutput("Wrist/CurrentState", state.name());
        Logger.recordOutput("Wrist/WristTarget", state.getWristTarget());
        Logger.recordOutput("Wrist/AtTarget", isWristAtTarget());

        io.controlWrist(state.getWristTarget());

    }

    @AutoLogOutput
    public WristState getState() {
        return state;
    }

    @AutoLogOutput
    public boolean isWristAtTarget() {
        double currentAngle = inputs.wristAngle;
        double targetAngle = state.getWristTarget().getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.Wrist.ANGLE_TOLERANCE.getDegrees();
    }

}
