package com.stuypulse.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public static final Wrist instance;
    public final WristState state;

    static {
        instance = new WristImpl();
    }

    public static Wrist getInstance() {
        return instance;
    }

    public enum WristState {
        FRONT(Rotation2d.fromDegrees(180)), 
        STOW(Rotation2d.fromDegrees(90)); 
        
        private Rotation2d targetAngle;
        
        private WristState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }

        public Rotation2d getTargetAngle() {
            return this.targetAngle;
        }
    }

    public Wrist() {
        this.state = WristState.STOW;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/Target Angle (Deg)", state.targetAngle.getDegrees());
    }
}
