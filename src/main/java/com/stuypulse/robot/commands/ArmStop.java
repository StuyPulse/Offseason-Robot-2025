package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.double_jointed_arm.DoubleJointedArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmStop extends InstantCommand {
    private DoubleJointedArm arm;

    public ArmStop() {
        arm = DoubleJointedArm.getInstance();
        addRequirements(arm);
    }

    public void initialize() {
        arm.runShoulder(0);
    }
}
