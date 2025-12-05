package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

public class ArmStow extends ArmSetState {
    public ArmStow() {
            super(ArmState.STOW);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
