package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.double_jointed_arm.DoubleJointedArm;
import com.stuypulse.robot.subsystems.double_jointed_arm.DoubleJointedArm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetState extends Command {
    private DoubleJointedArm arm;
    private ArmState state;

    public ArmSetState(ArmState state) {
        this.state = state;
        this.arm = DoubleJointedArm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.getState().isFront() != state.isFront()) arm.switchSides();
        arm.setState(state);
    }

    @Override
    public boolean isFinished() {
        return arm.isArmAtTarget() && arm.getState() == state;
    }
}
