package com.stuypulse.robot.commands.neointake;

import com.stuypulse.robot.subsystems.neointake.NeoIntake.NeoIntakeRollerState;

public class NeoIntakeIntake extends SetNeoIntakeState {

    public NeoIntakeIntake() {
        super(NeoIntakeRollerState.INTAKING);
    }

}