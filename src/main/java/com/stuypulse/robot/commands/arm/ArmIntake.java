package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmIntake extends ArmSetState {
    public ArmIntake() {
            super(ArmState.INTAKE_FRONT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
