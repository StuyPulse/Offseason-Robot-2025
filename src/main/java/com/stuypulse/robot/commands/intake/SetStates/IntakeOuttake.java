package com.stuypulse.robot.commands.intake.SetStates;

import com.stuypulse.robot.commands.intake.SetIntakeState;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeRollerState;

public class IntakeOuttake extends SetIntakeState{
    
    public IntakeOuttake() {
        super(IntakeRollerState.OUTTAKING);
    }

}
