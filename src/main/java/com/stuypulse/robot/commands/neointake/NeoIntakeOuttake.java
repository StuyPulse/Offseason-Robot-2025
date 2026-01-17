package com.stuypulse.robot.commands.neointake;

import com.stuypulse.robot.subsystems.neointake.NeoIntake.NeoIntakeRollerState;

public class NeoIntakeOuttake extends SetNeoIntakeState {
    
    public NeoIntakeOuttake() {
        super(NeoIntakeRollerState.OUTTAKING);
    }

}
