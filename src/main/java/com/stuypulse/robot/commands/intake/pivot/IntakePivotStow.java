package com.stuypulse.robot.commands.intake.pivot;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakePivotStow extends IntakePivotSetState {

    public IntakePivotStow() {
        super(PivotState.STOW);
    }
}