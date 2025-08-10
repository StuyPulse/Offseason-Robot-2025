package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetState extends Command {
    private Arm arm;
    private ArmState state;

    public ArmSetState(ArmState state) {
        this.state = state;
        this.arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.getState().isFront() != state.isFront()) {
            arm.setState(state);
            arm.switchSides();
        } else {
            arm.setState(state);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isArmAtTarget() && arm.getState() == state;
    }
}
