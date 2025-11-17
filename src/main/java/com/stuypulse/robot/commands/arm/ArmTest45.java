package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTest45 extends ArmSetState {
    public ArmTest45() {
            super(ArmState.TEST_FRONT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
