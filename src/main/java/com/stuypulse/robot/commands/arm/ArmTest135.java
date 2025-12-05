package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTest135 extends ArmSetState {
    public ArmTest135() {
            super(ArmState.TEST_BACK);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
