package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmTest45;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveArm extends SequentialCommandGroup{
    public MoveArm(PathPlannerPath... paths) {
        addCommands(
            new ArmTest45()
        );
    }
}
