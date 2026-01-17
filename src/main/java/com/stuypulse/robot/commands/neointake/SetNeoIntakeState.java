package com.stuypulse.robot.commands.neointake;

import com.stuypulse.robot.subsystems.neointake.NeoIntake;
import com.stuypulse.robot.subsystems.neointake.NeoIntake.NeoIntakeRollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetNeoIntakeState extends InstantCommand {
    
    private final NeoIntake intake;
    private final NeoIntakeRollerState state;

    public SetNeoIntakeState(NeoIntakeRollerState state) {
        this.intake = NeoIntake.getInstance();
        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(state);
    }

}
