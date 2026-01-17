package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase{

    public static final Intake instance;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeRollerState {
        INTAKING(Settings.Intake.INTAKE_SPEED.doubleValue()),
        OUTTAKING(-Settings.Intake.INTAKE_SPEED.doubleValue()),
        STOP(0.0);

        private Number intake_roller_speed;

        private IntakeRollerState(Number intake_roller_speed) {
            this.intake_roller_speed = intake_roller_speed;
        }

        public double getIntakeRollerSpeed() {
            return this.intake_roller_speed.doubleValue();
        }
    }

    protected IntakeRollerState state;

    protected Intake() {
        this.state = IntakeRollerState.STOP;
    }

    public IntakeRollerState getState() {
        return state;
    }

    public void setState(IntakeRollerState state) {
        this.state = state;
    }
}