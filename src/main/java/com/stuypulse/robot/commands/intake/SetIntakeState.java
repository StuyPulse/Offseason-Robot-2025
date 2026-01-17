package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeRollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeState extends InstantCommand {
    
    private final Intake intake;
    private final IntakeRollerState state;

    public SetIntakeState(IntakeRollerState state) {
        this.intake = Intake.getInstance();
        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(state);
    }

}
