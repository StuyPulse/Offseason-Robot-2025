package com.stuypulse.robot.commands.neointake;

import com.stuypulse.robot.subsystems.neointake.NeoIntake.NeoIntakeRollerState;

public class NeoIntakeStop extends SetNeoIntakeState {
    
    public NeoIntakeStop() {
        super(NeoIntakeRollerState.STOP);
    }

}
