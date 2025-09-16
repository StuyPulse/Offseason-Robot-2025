package com.stuypulse.robot.commands.arm;

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
        arm.setState(state);
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetElbowAngle() && arm.atTargetElbowAngle() && arm.getState() == state;
    }
}
