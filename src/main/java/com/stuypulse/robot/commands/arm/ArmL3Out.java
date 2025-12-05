package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmL3Out extends ArmSetState {
    public ArmL3Out() {
            super(ArmState.L3_BACK_OUT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
