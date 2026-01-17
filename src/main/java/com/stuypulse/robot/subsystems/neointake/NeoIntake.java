package com.stuypulse.robot.subsystems.neointake;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NeoIntake extends SubsystemBase{

    public static final NeoIntake instance;

    static {
        instance = new NeoIntakeImpl();
    }

    public static NeoIntake getInstance() {
        return instance;
    }

    public enum NeoIntakeRollerState {
        INTAKING(Settings.Intake.NeoIntake.INTAKE_SPEED.doubleValue()),
        OUTTAKING(-Settings.Intake.NeoIntake.INTAKE_SPEED.doubleValue()),
        STOP(0.0);

        private Number intake_roller_speed;

        private NeoIntakeRollerState(Number intake_roller_speed) {
            this.intake_roller_speed = intake_roller_speed;
        }

        public double getIntakeRollerSpeed() {
            return this.intake_roller_speed.doubleValue();
        }
    }

    protected NeoIntakeRollerState state;

    protected NeoIntake() {
        this.state = NeoIntakeRollerState.STOP;
    }

    public NeoIntakeRollerState getState() {
        return state;
    }

    public void setState(NeoIntakeRollerState state) {
        this.state = state;
    }
}