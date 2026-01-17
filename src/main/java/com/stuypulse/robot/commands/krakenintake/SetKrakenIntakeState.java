package com.stuypulse.robot.commands.krakenintake;

import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake;
import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake.KrakenIntakeRollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetKrakenIntakeState extends InstantCommand {
    
    private final KrakenIntake intake;
    private final KrakenIntakeRollerState state;

    public SetKrakenIntakeState(KrakenIntakeRollerState state) {
        this.intake = KrakenIntake.getInstance();
        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(state);
    }
}