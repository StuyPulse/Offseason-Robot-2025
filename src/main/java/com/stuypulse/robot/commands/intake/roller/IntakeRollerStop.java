package com.stuypulse.robot.commands.intake.roller;

import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakeRollerStop extends IntakeRollerSetState {

    public IntakeRollerStop() {
        super(RollerState.STOP);
    }
}