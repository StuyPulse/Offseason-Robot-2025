package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetState extends Command {
    private Arm arm;
    private ArmState targetState;

    public ArmSetState(ArmState targetState) {
        this.targetState = targetState;
        this.arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.getState().isFront() != targetState.isFront()) {
            arm.switchSides(targetState);
        } else {
            arm.setState(targetState);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.getState() == targetState && arm.atTargetShoulderAngle() && arm.atTargetElbowAngle();
    }
}
