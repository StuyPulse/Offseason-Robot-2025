package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmL1 extends ArmSetState {
    public ArmL1() {
            super(ArmState.L1_BACK);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
