package com.stuypulse.robot.commands.intake.roller;

import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakeRollerEject extends IntakeRollerSetState {

    public IntakeRollerEject() {
        super(RollerState.EJECTING);
    }
}