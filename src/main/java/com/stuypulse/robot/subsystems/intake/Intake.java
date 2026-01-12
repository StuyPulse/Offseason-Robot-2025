package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase{

    public static final IntakeImpl instance;

    static {
        instance = new IntakeImpl();
    }
    
    public static IntakeImpl getInstance() {
        return instance;
    }

    public Intake() {}
}
