package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmL4Out extends ArmSetState {
    public ArmL4Out() {
            super(ArmState.L4_BACK_OUT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
