package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase{

    public static final IntakeImpl instance;

    private IntakeState intake;

    static {
        instance = new IntakeImpl();
    }
    
    public static IntakeImpl getInstance() {
        return instance;
    }

    public Intake() {
        intake = IntakeState.STOW;
    }

    public enum IntakeState {
        STOW(0.0),
        INTAKE(0.5);

        private double intakeCycle;

        private IntakeState(double dutyCycle) {
            this.intakeCycle = dutyCycle;
        } 

        public double getIntakeCycle() {
            return intakeCycle;
        }

        public void setIntakeCycle(double dutyCycle) {
            this.intakeCycle = dutyCycle;
        }
    }

    public void setIntakeState(IntakeState state) {
        this.intake = state;
    }
    public IntakeState getIntakeState() {
        return intake;
    }
    @Override
    public void periodic() {
        
    }
}