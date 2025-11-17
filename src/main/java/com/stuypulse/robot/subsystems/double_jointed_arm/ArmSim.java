/*package com.stuypulse.robot.subsystems.double_jointed_arm;

import java.util.List;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {

    private Mechanism2d canvas;
        // Hardware
    // private final TalonFX frontShoulderMotor;
    // private final TalonFX backShoulderMotor;
    // private final TalonFX elbowMotor;
    
    private final Pigeon2 pigeon;
    private final CoreCANcoder elbowEncoder;

    private final Timer timer;

    // Matricies
    private Matrix<N2, N2> mMatrix;
    private Matrix<N2, N2> cMatrix;
    private Matrix<N2, N1> gMatrix;

    private Matrix<N2, N1> vMatrix;
    private Matrix<N2, N1> aMatrix;

    // Physical Constants
    private final double shoulderMass = Constants.DoubleJointedArm.Shoulder.MASS; // kg
    private final double elbowMass = Constants.DoubleJointedArm.Elbow.MASS;       // kg
    private final double shoulderLength = Constants.DoubleJointedArm.Shoulder.LENGTH; // m
    private final double elbowLength = Constants.DoubleJointedArm.Elbow.LENGTH;       // m
    private final double GRAVITY = 9.81;

    // Conversion Factors
    private final double shoulderGearRatio = Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
    private final double elbowGearRatio = Constants.DoubleJointedArm.Elbow.GEAR_RATIO;
    private final double TORQUE_TO_VOLTAGE = 1.0 / 12.0; // Converts Nm to volts

    private final PositionVoltage shoulderPositionReq = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage elbowPositionReq = new PositionVoltage(0).withSlot(1);

    private int trajectoryIndex = 1;

    public ArmSim() {
        // frontShoulderMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        // backShoulderMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        // elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);
        // pigeon = new Pigeon2(Ports.Arm.PIGEON);
        // elbowEncoder = new CoreCANcoder(Ports.Arm.ABSOLUTE_ENCODER);
        
        mMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        cMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        gMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        timer =  new Timer();
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(pigeon.getPitch().getValueAsDouble());
    }

    @Override
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition().getValueAsDouble());
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getVelocities() {
        double a0_0 = pigeon.getAngularVelocityYDevice().getValueAsDouble();
        double a1_0 = elbowEncoder.getVelocity().getValueAsDouble();
        vMatrix.set(0, 0, a0_0);
        vMatrix.set(1, 0, a1_0);
        return vMatrix;
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getAccelerations(){
        
        Pair<Double, Double> AStream = new Pair<>(pigeon.getAccelerationY().getValueAsDouble(), 
                                                  elbowEncoder.getVelocity().getValueAsDouble() / timer.get());

        timer.reset();

        aMatrix.set(0, 0, AStream.getSecond());
        aMatrix.set(1, 0, AStream.getFirst());

        return aMatrix;
    }

    @Override
    public Matrix<N2, N2> calculateMMatrix() {
        double c2 = Math.cos(getElbowAngle().getRadians());
        
        // Moment of inertia (can be pre determined)
        // 1/3 mass * length^2
        double I1 = shoulderMass * Math.pow(shoulderLength, 2) / 3; 
        double I2 = elbowMass * Math.pow(elbowLength, 2) / 3;
        
        double m0_0 = shoulderMass * Math.pow(shoulderLength/2, 2) 
                    + elbowMass * (Math.pow(shoulderLength, 2) + Math.pow(elbowLength/2, 2))
                    + I1 + I2 
                    + 2 * elbowMass * shoulderLength * (elbowLength/2) * c2;
        
        double m0_1 = elbowMass * Math.pow(elbowLength/2, 2) 
                    + I2 
                    + elbowMass * shoulderLength * (elbowLength/2) * c2;

        double m1_1 = elbowMass * Math.pow(elbowLength/2, 2) + I2;
        
        // M is symmetric
        mMatrix.set(0, 0, m0_0);
        mMatrix.set(0, 1, m0_1);
        mMatrix.set(1, 0, m0_1); // m1_0 = m0_1
        mMatrix.set(1, 1, m1_1);
        
        return mMatrix;
    }

    @Override
    public Matrix<N2, N2> calculateCMatrix() {
        double s2 = Math.sin(getElbowAngle().getRadians());
        double theta1_dot = getVelocities().get(0, 0);
        double theta2_dot = getVelocities().get(1, 0);
        
        double c0_0 = -elbowMass * shoulderLength * (elbowLength/2) * s2 * theta2_dot;
        double c0_1 = -elbowMass * shoulderLength * (elbowLength/2) * s2 * (theta1_dot + theta2_dot);
        double c1_0 = elbowMass * shoulderLength * (elbowLength/2) * s2 * theta1_dot;
        
        cMatrix.set(0, 0, c0_0);
        cMatrix.set(0, 1, c0_1);
        cMatrix.set(1, 0, c1_0);
        cMatrix.set(1, 1, 0.0);
        
        return cMatrix;
    }

    @Override
    public Matrix <N2, N1> calculateGMatrix(){
        double g0_0 = (shoulderMass * (shoulderLength / 2) + elbowLength * shoulderLength) * GRAVITY * Math.cos(getShoulderAngle().getRadians()
                        + elbowMass * (elbowLength / 2) * GRAVITY * Math.cos(getShoulderAngle().getRadians() + getElbowAngle().getRadians()));
        double g1_0 = (elbowMass * (elbowLength / 2) * GRAVITY * Math.cos(getShoulderAngle().getRadians() + getElbowAngle().getRadians()));
        
        gMatrix.set(0, 0, g0_0);
        gMatrix.set(1, 0, g1_0);
        
        return gMatrix;
    }

    public Matrix<N2, N1> calculateTorque() {
        Matrix<N2, N1> velocities = getVelocities();
        Matrix<N2, N1> accelerations = getAccelerations();
        
        // M*accel + C*vel + G
        return calculateMMatrix().times(accelerations)
               .plus(calculateCMatrix().times(velocities))
               .plus(calculateGMatrix());
    }

    @Override
    public Translation2d getEndPosition() {
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();

        Transform2d startPoint = new Transform2d(0.0, Constants.Arm.BASE_HEIGHT, new Rotation2d(0.0));
        Translation2d endPoint = startPoint.plus(new Transform2d(shoulderLength, 0.0, shoulder))
                                .plus(new Transform2d(elbowLength, 0.0, elbow.minus(new Rotation2d(Math.PI - shoulder.getRadians()))))
                                .getTranslation();
        return endPoint;
    }

    @Override
    public void periodic() { 

        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getRadians());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngle().getRadians());
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());

        SmartDashboard.putNumber("Arm/Shoulder Torque", calculateTorque().get(0, 0));
        SmartDashboard.putNumber("Arm/Elbow Torque", calculateTorque().get(1, 0));
    }
}*/