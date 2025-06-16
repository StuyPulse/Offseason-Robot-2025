package com.stuypulse.robot.commands.intake.roller;

import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakeRollerIntake extends IntakeRollerSetState {

    public IntakeRollerIntake() {
        super(RollerState.INTAKING);
    }
}