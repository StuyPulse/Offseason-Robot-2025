package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTest4545 extends ArmSetState {
    public ArmTest4545() {
            super(ArmState.TEST_45);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
