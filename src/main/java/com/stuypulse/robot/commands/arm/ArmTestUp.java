package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTestUp extends ArmSetState {
    public ArmTestUp() {
            super(ArmState.INT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
