package com.stuypulse.robot.commands.intake.pivot;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

abstract class IntakePivotSetState extends InstantCommand {

    private final Intake intake;
    private final PivotState pivotState;

    public IntakePivotSetState(PivotState state) {
        intake = Intake.getInstance();
        this.pivotState = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotState(pivotState);
    }
}