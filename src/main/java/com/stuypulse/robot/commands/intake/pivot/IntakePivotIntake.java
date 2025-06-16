package com.stuypulse.robot.commands.intake.pivot;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakePivotIntake extends IntakePivotSetState {

    public IntakePivotIntake() {
        super(PivotState.INTAKING);
    }
}