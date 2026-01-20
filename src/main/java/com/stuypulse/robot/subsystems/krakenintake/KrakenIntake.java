package com.stuypulse.robot.subsystems.krakenintake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports.Intakes;
import com.stuypulse.robot.constants.Settings.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class KrakenIntake extends SubsystemBase{

    public static final KrakenIntake instance;

    static {
        instance = new KrakenIntakeImpl(Intakes.KrakenIntake.ROLLER);
    }

    public static KrakenIntake getInstance() {
        return instance;
    }

    public enum KrakenIntakeRollerState {
        INTAKING(Settings.Intake.NeoIntake.INTAKE_SPEED.doubleValue()),
        OUTTAKING(-Settings.Intake.NeoIntake.INTAKE_SPEED.doubleValue()),
        STOP(0.0);

        private Number intake_roller_speed;

        private KrakenIntakeRollerState(Number intake_roller_speed) {
            this.intake_roller_speed = intake_roller_speed;
        }

        public double getKrakenIntakeRollerSpeed() {
            return this.intake_roller_speed.doubleValue();
        }
    }

    protected KrakenIntakeRollerState state;

    protected KrakenIntake() {
        this.state = KrakenIntakeRollerState.STOP;
    }

    public KrakenIntakeRollerState getState() {
        return state;
    }

    public void setState(KrakenIntakeRollerState state) {
        this.state = state;
    }
}