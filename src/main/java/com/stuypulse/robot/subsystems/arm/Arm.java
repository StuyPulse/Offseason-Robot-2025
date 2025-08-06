package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Constants.DoubleJointedArm;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase{
    public static final Arm instance;

    static {
        if (Robot.isReal()){
            instance = new ArmImpl();
        } else {
            instance = new ArmSim();
        }
    }

    public static Arm getInstance(){
        return instance;
    }

    public enum ArmState {
        RESTING(Rotation2d.fromRadians(Settings.DoubleJointedArm.Shoulder.DEFAULT), Rotation2d.fromDegrees(Settings.DoubleJointedArm.Elbow.DEFAULT));

        private Rotation2d shoulderTargetAngle;
        private Rotation2d elbowTargetAngle;

        private ArmState(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle){
            this.shoulderTargetAngle = Rotation2d.fromRadians(
                SLMath.clamp(shoulderTargetAngle.getRadians(), 
                Constants.DoubleJointedArm.Shoulder.MIN_ANGLE.getRadians(), 
                Constants.DoubleJointedArm.Shoulder.MAX_ANGLE.getRadians()));
            this.elbowTargetAngle = Rotation2d.fromRadians(
                SLMath.clamp(elbowTargetAngle.getRadians(), 
                Constants.DoubleJointedArm.Elbow.MIN_ANGLE.getRadians(), 
                Constants.DoubleJointedArm.Elbow.MAX_ANGLE.getRadians())); 
        } 

        public Rotation2d getShoulderTargetAngle(){
            return this.shoulderTargetAngle;
        }

        public Rotation2d getElbowTargetAngle(){
            return this.elbowTargetAngle;
        }
    }

    private ArmState state;

    public ArmState getState(){
        return this.state;
    }

    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getElbowAngle();
    public abstract void setTargetPosition(Translation2d target);
    public abstract Translation2d getEndPosition();
    public abstract void setTargetAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle);
    public abstract Matrix<N2, N2> calculateMMatrix(); // Mass Intertia Matrix
    public abstract Matrix<N2, N2> calculateCMatrix(); // Centrifugal + Coriolis Matrix
    public abstract Matrix<N2, N1> calculateGMatrix(); // Torque due to Gravity Matrix
    

    public void setState(ArmState state) {
        setGoal(state.getShoulderTargetAngle().getRadians(), state.getShoulderTargetAngle().getRadians());
    }

    protected void setGoal(double shoulderRads, double elbowRads) {
    }
}