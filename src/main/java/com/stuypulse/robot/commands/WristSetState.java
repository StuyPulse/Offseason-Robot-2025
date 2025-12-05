package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.wrist.*;
import com.stuypulse.robot.subsystems.wrist.Wrist.WristState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WristSetState extends InstantCommand {
    private final Wrist wrist;
    private final WristState state;

    public WristSetState(WristState state) {
        this.wrist = Wrist.getInstance();
        this.state = state;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.resetController(false);
        wrist.setState(state);
    }
}
