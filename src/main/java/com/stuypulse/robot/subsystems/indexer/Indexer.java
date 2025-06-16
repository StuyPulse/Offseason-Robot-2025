package com.stuypulse.robot.subsystems.indexer;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Indexer extends SubsystemBase {
    
    private static final Indexer instance;

    static {
        if (Robot.isReal()) {
            instance = new IndexerImpl();
        } else {
            instance = new IndexerImpl(); // update
        }
    }

    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKE(1),
        OUTTAKE(-1),
        STANDBY(0);

        private double speed;

        private IndexerState(double speed) {
            this.speed = speed;
        }
        public double getSpeed() {
            return speed;
        }
    }

    private IndexerState state;

    public Indexer() {
        this.state = IndexerState.STANDBY;
    }

    public IndexerState getState() {
        return state;
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public abstract boolean coralInPlace();
}
