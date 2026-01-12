package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase{

    public static final Intake instance;

    static {
        instance = new Intakeimpl();
    }
    
    public Intake getInstance() {
        return instance;
    }
     
}
