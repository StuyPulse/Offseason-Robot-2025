package com.stuypulse.robot.commands.intake.roller;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeRollerSetState extends InstantCommand {

    private final Intake intake;
    private final RollerState rollerState;

    public IntakeRollerSetState(RollerState state) {
        this.intake = Intake.getInstance();
        this.rollerState = state;
    }

    @Override
    public void initialize() {
        intake.setRollerState(rollerState);
    }
}