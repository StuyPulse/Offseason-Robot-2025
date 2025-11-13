package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase{
    public static final Arm instance;

    static {
        if (Robot.isReal()){
            instance = new ArmImpl();
        } else {
            instance = new ArmImpl();
        }
    }

    protected TrapezoidProfile shoulderProfile;
    protected TrapezoidProfile.State currentShoulderState;
    protected TrapezoidProfile.State targetShoulderState;
    protected double targetShoulderAccel; // this should be next accel
    protected TrapezoidProfile elbowProfile;
    protected TrapezoidProfile.State currentElbowState;
    protected TrapezoidProfile.State targetElbowState;
    protected double targetElbowAccel;

    public static Arm getInstance(){
        return instance;
    }

    public enum ArmState {
        STOW(Rotation2d.fromDegrees(Settings.DoubleJointedArm.Shoulder.DEFAULT), Rotation2d.fromDegrees(Settings.DoubleJointedArm.Elbow.DEFAULT)),
        TEST_FRONT(Rotation2d.fromDegrees(45.0), Rotation2d.fromDegrees(-180.0)),
        TEST_45(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(45)),
        TEST_BACK(Rotation2d.fromDegrees(135.0), Rotation2d.fromDegrees(90)),
        INT(Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0));
        private Rotation2d shoulderTargetAngle;
        private Rotation2d elbowTargetAngle;

        private ArmState(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle){
            this.shoulderTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(shoulderTargetAngle.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MIN_ANGLE.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MAX_ANGLE.getDegrees()));
            this.elbowTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(elbowTargetAngle.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MIN_ANGLE.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MAX_ANGLE.getDegrees())); 
        } 

    public Rotation2d getShoulderTargetAngle(){
            return this.shoulderTargetAngle;
        }

        public Rotation2d getElbowTargetAngle(){
            return this.elbowTargetAngle;
        }

        public boolean isFront() {
        return this.name().endsWith("FRONT") || this.equals(ArmState.STOW);
        }

        // public ArmState getOpposite() {
        //     switch (this) {
        //         case TEST_FRONT: return TEST_BACK;
        //         case TEST_BACK: return TEST_FRONT;
        //         default: return STOW;
        //     }
        // }
    }

    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getElbowAngle();
    public abstract double getShoulderVelocity();
    public abstract double getElbowVelocity();

    // public abstract void setTargetPosition(Translation2d target);
    public abstract void switchSides(ArmState targetState);

    public abstract Translation2d getEndPosition();
    public abstract boolean atTargetElbowAngle();
    public abstract boolean atTargetShoulderAngle();
    public abstract Matrix<N2, N2> calculateMMatrix(); // Mass Intertia Matrix
    public abstract Matrix<N2, N2> calculateCMatrix(); // Centrifugal + Coriolis Matrix
    public abstract Matrix<N2, N1> calculateGMatrix(); // Torque due to Gravity Matrix
    public abstract Matrix<N2, N1> calculateTorque(Matrix<N2, N1> velocities, Matrix<N2, N1> accelerations); 
    public abstract Matrix<N2, N2> calculateBackEmf();
    public abstract Matrix<N2, N2> calculateMotorTorque();
    public abstract Matrix<N2, N1> calculateVoltage();

    private ArmState state; 

    protected Arm() {
        this.state = ArmState.INT;

        // We assume the arm starts at the STOW state
        shoulderProfile = new TrapezoidProfile(
            new Constraints(
                Constants.DoubleJointedArm.Shoulder.maxVelocity, 
                Constants.DoubleJointedArm.Shoulder.maxAcceleration
            )
        );
        
        currentShoulderState = new TrapezoidProfile.State(ArmState.STOW.shoulderTargetAngle.getRadians(), 0.0);
        targetShoulderState = new TrapezoidProfile.State(ArmState.INT.shoulderTargetAngle.getRadians(), 0.0); 
 
        elbowProfile = new TrapezoidProfile(
            new Constraints(
                Constants.DoubleJointedArm.Elbow.maxVelocity, 
                Constants.DoubleJointedArm.Elbow.maxAcceleration
            )
        );
        
        currentElbowState = new TrapezoidProfile.State(ArmState.STOW.elbowTargetAngle.getRadians(), 0.0);
        targetElbowState = new TrapezoidProfile.State(ArmState.INT.elbowTargetAngle.getRadians(), 0.0);
    }

    public ArmState getState(){
        return this.state;
    }

    public void setState(ArmState state) {
        this.state = state;

        targetShoulderState = new TrapezoidProfile.State(
            state.getShoulderTargetAngle().getRadians(),
            0.0
        );
    
        targetElbowState = new TrapezoidProfile.State(
            state.getElbowTargetAngle().getRadians(),
            0.0
        );
    }

    @Override
    public void periodic() {
      
    }
}