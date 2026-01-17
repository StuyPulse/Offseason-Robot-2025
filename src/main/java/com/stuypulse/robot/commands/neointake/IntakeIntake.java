package com.stuypulse.robot.commands.neointake;

import com.stuypulse.robot.commands.neointake.SetIntakeState;
import com.stuypulse.robot.subsystems.neointake.NeoIntake.NeoIntakeRollerState;

public class IntakeIntake extends SetIntakeState {

    public IntakeIntake() {
        super(NeoIntakeRollerState.INTAKING);
    }

}