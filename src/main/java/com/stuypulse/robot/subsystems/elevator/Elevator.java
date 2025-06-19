package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {
    private static Elevator instance;

    static {
        if(Robot.isReal()) {
            instance = new ElevatorImpl();
        } else {
            instance = new ElevatorVisualizer();
        }
    }

    public static Elevator getInstance() {
        return instance;
    }

    public abstract double getCurrentHeight();
    public abstract double getTargetHeight();
    public abstract void setTargetHeight(double newHeight);
    public abstract boolean isAtTargetHeight();
}
