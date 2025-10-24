package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {
    // Hardware
    private final TalonFX frontShoulderMotor;
    private final TalonFX backShoulderMotor;
    private final TalonFX elbowMotor;
    
    private final CoreCANcoder shoulderEncoder;
    private final CoreCANcoder elbowEncoder;

    private final Timer timer;

    private boolean intermediate;
    private ArmState storedState;

    // Matricies
    private Matrix<N2, N2> mMatrix;
    private Matrix<N2, N2> cMatrix;
    private Matrix<N2, N1> gMatrix;

    private Matrix<N2, N1> vMatrix;
    private Matrix<N2, N1> aMatrix;

    private Matrix<N2, N2> kBMatrix;
    private Matrix<N2, N2> BMatrix;
    private Matrix<N2, N1> uMatrix;
    private Matrix<N2, N1> targetVelocityMatrix;
    private Matrix<N2, N1> targetAccelMatrix;

    // Physical Constants
    private final double shoulderMass = Constants.DoubleJointedArm.Shoulder.MASS;       // kg
    private final double elbowMass = Constants.DoubleJointedArm.Elbow.MASS + Constants.Wrist.MASS;             // kg
    private final double shoulderLength = Constants.DoubleJointedArm.Shoulder.LENGTH;   // m
    private final double elbowLength = Constants.DoubleJointedArm.Elbow.LENGTH;         // m
    private final double GRAVITY = 9.81;

    private final double kT = 7.09 / 366.0;  // Stall torque / Stall current
    private final double kU = 6000.0 / 12.0; // Free speed / Volts
    private final double r = 12.0 / 366.0;   // Volts / Stall current
    // Conversion Factors
    private final double shoulderGearRatio = Constants.DoubleJointedArm.Shoulder.MOTOR_GEAR_RATIO;
    private final double elbowGearRatio = Constants.DoubleJointedArm.Elbow.MOTOR_GEAR_RATIO;

    public ArmImpl() {
        frontShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.FRONT_MOTOR, "CANIVORE");
        backShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.BACK_MOTOR, "CANIVORE");
        elbowMotor = new TalonFX(Ports.DoubleJointedArm.Elbow.MOTOR, "CANIVORE");
        shoulderEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Shoulder.ENCODER, "CANIVORE");
        elbowEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Elbow.ENCODER, "CANIVORE");
        
        mMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        cMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        gMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        vMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        aMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        targetVelocityMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        targetAccelMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        kBMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        BMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        uMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        intermediate = false;
        storedState = ArmState.STOW;
        timer =  new Timer();
        
        configureMotors();
    }

    private void configureMotors() {
        frontShoulderMotor.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.getMotorConfig());
        backShoulderMotor.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.getMotorConfig());

        // Devices.DoubleJointedArm.Shoulder.motor_config.configure(frontShoulderMotor);
        
        // Devices.DoubleJointedArm.Shoulder.motor_followerConfig.configure(backShoulderMotor);
        // Devices.DoubleJointedArm.Elbow.motor_config.configure(elbowMotor);

        // backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));

        // Devices.DoubleJointedArm.Elbow.motor_config.configure(elbowMotor);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition().getValueAsDouble() * Constants.DoubleJointedArm.Shoulder.ENCODER_GEAR_RATIO + Constants.DoubleJointedArm.Shoulder.ENCODER_OFFSET_ROT);
    }
 
    // @Override
     //public Rotation2d getElbowAngle() {
       //  return Rotation2d.fromRotations(elbowEncoder.getPosition().getValueAsDouble() * Constants.DoubleJointedArm.Elbow.ENCODER_GEAR_RATIO + Constants.DoubleJointedArm.Elbow.ENCODER_OFFSET_ROT);
     // }

    public Rotation2d getPerfectElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getPosition().getValueAsDouble() * Constants.DoubleJointedArm.Elbow.ENCODER_GEAR_RATIO + Constants.DoubleJointedArm.Elbow.ENCODER_OFFSET_ROT);
    }

    public Rotation2d getRelativeElbowAngle() {
        double rawRelative = (0.5 - getShoulderAngle().getRotations() + getElbowAngle().getRotations());
        return Rotation2d.fromRotations((rawRelative > 0.5) ? 1-rawRelative : rawRelative);
    }

    @Override
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(getPerfectElbowAngle().getRotations() - (0.25 - getShoulderAngle().getRotations()));
    }

    @Override
    public double getShoulderVelocity() {
        return frontShoulderMotor.getVelocity().getValueAsDouble() * 2*Math.PI;
    }

    @Override
    public double getElbowVelocity() {
        return elbowEncoder.getVelocity().getValueAsDouble() * 2*Math.PI;
    }

    @Override
    public boolean atTargetElbowAngle(){
        double targetAngle = getState().getElbowTargetAngle().getRadians();
        return (Math.abs(targetAngle - getElbowAngle().getRadians()) <= Settings.DoubleJointedArm.Elbow.TOLERANCE);
    }

    @Override
    public boolean atTargetShoulderAngle(){
        double targetAngle = getState().getShoulderTargetAngle().getRadians();
        return (Math.abs(targetAngle - getShoulderAngle().getRadians()) <= Settings.DoubleJointedArm.Shoulder.TOLERANCE);
    }

    @Override
    public void switchSides(ArmState targetState) { 
        if (!intermediate) {
            intermediate = true;
            storedState = targetState;
            setState(ArmState.INT);
        }
    }

    @Override
    public Translation2d getEndPosition() {
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();

        Transform2d startPoint = new Transform2d(0.0, Constants.DoubleJointedArm.BASE_HEIGHT, new Rotation2d(0.0));
        Translation2d endPoint = startPoint.plus(new Transform2d(shoulderLength, 0.0, shoulder))
                                .plus(new Transform2d(elbowLength, 0.0, elbow.minus(new Rotation2d(Math.PI - shoulder.getRadians()))))
                                .getTranslation();
        return endPoint;
    }

    @Override
    public void periodic() {
        if (intermediate && atTargetElbowAngle() && atTargetShoulderAngle()) {
            intermediate = false;
            setState(storedState);
        }

        final double dt = 0.02;

        currentShoulderState.position = getShoulderAngle().getRadians();
        currentShoulderState.velocity = getShoulderVelocity();

        currentElbowState.position = getElbowAngle().getRadians();
        currentElbowState.velocity = getElbowVelocity();

        // this is the next step in the profile
        TrapezoidProfile.State nextShoulderState = shoulderProfile.calculate(
            dt,
            currentShoulderState, 
            targetShoulderState
        );
        
        TrapezoidProfile.State nextElbowState = elbowProfile.calculate(
            dt,
            currentElbowState,
            targetElbowState
        );

        double shoulderAccel = (nextShoulderState.velocity - currentShoulderState.velocity) / dt;
        double elbowAccel = (nextElbowState.velocity - currentElbowState.velocity) / dt;

        targetVelocityMatrix.set(0, 0, nextShoulderState.velocity);
        targetVelocityMatrix.set(1, 0, nextElbowState.velocity);

        targetAccelMatrix.set(0, 0, shoulderAccel);
        targetAccelMatrix.set(1, 0, elbowAccel);

        calculateTorque(targetVelocityMatrix, targetAccelMatrix);
        calculateBackEmf();
        calculateMotorTorque();

        double shoulderVolts = calculateVoltage().get(0, 0);
        double elbowVolts = calculateVoltage().get(1, 0);

        // frontShoulderMotor.setControl(new VoltageOut(shoulderVolts));
        
        // elbowMotor.setControl(new VoltageOut(elbowVolts));

        // Logging
        SmartDashboard.putNumber("DoubleJointedArm/currentShoulderState Position", currentShoulderState.position);
        SmartDashboard.putNumber("DoubleJointedArm/currentShoulderState Velocity From Motor Controller", getShoulderVelocity() * (180f / Math.PI ));
        SmartDashboard.putNumber("DoubleJointedArm/currentShoulderState Velocity", currentShoulderState.velocity);
        SmartDashboard.putNumber("DoubleJointedArm/targetShoulderState Velocity", targetShoulderState.velocity);
        SmartDashboard.putNumber("DoubleJointedArm/targetShoulderState Position", targetShoulderState.position);
        SmartDashboard.putNumber("DoubleJointedArm/nextShoulderState Velocity", nextShoulderState.velocity * (180f/Math.PI));
        SmartDashboard.putNumber("DoubleJointedArm/nextShoulderState Position", nextShoulderState.position);
        SmartDashboard.putString("DoubleJointedArm/State", getState().toString());
        // SmartDashboard.putNumber("DoubleJointedArm/Target Shoulder Angle", getState().getShoulderTargetAngle().getDegrees());
        // SmartDashboard.putNumber("DoubleJointedArm/Target Elbow Angle", getState().getElbowTargetAngle().getDegrees());

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("DoubleJointedArm/Relative elbow Angle", getRelativeElbowAngle().getDegrees());
        SmartDashboard.putNumber("DoubleJointedArm/Elbow Angle", getElbowAngle().getDegrees());
        // SmartDashboard.putNumber("DoubleJointedArm/End Height", getEndPosition().getY());

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder Velocity", getVelocities().get(0, 0));
        SmartDashboard.putNumber("DoubleJointedArm/Elbow Velocity", getVelocities().get(1, 0));

        // SmartDashboard.putNumber("DoubleJointedArm/Shoulder debug accel", frontShoulderMotor.getVelocity().getValueAsDouble()/0.02);
        // SmartDashboard.putNumber("DoubleJointedArm/Shoulder Acceleration", getAccelerations().get(0, 0));
        // SmartDashboard.putNumber("DoubleJointedArm/Elbow Acceleration", getAccelerations().get(1, 0));

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder Target Velocity", targetVelocityMatrix.get(0, 0) * (180 / Math.PI));
        SmartDashboard.putNumber("DoubleJointedArm/Elbow Target Velocity", targetVelocityMatrix.get(1, 0)* (180 / Math.PI));

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder Target Acceleration", targetAccelMatrix.get(0, 0));
        SmartDashboard.putNumber("DoubleJointedArm/Elbow Target Acceleration", targetAccelMatrix.get(1, 0));

        // SmartDashboard.putNumber("DoubleJointedArm/shoulder cancoder debugging", shoulderEncoder.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("DoubleJointedArm/shoulder motor debugging", frontShoulderMotor.getPosition().getValueAsDouble());

        // SmartDashboard.putNumber("DoubleJointedArm/Shoulder Torque", calculateTorque().get(0, 0));
        // SmartDashboard.putNumber("DoubleJointedArm/Elbow Torque", calculateTorque().get(1, 0));

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder Voltage", calculateVoltage().get(0,0));
        SmartDashboard.putNumber("DoubleJointedArm/Elbow Voltage", calculateVoltage().get(1,0));

        SmartDashboard.putNumber("DoubleJointedArm/Shoulder MT", BMatrix.inv().get(0, 0));
        SmartDashboard.putNumber("DoubleJointedArm/Elbow MT", BMatrix.inv().get(1,1));
        SmartDashboard.putNumber("DoubleJointedArm/Timer", timer.get());
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getVelocities() {
        double a0_0 = shoulderEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        double a1_0 = elbowEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        vMatrix.set(0, 0, a0_0);
        vMatrix.set(1, 0, a1_0);
        return vMatrix;
    }
    
    // Return 2x1 Matrix
    public Matrix<N2, N1> getAccelerations() {

        aMatrix.set(0, 0, new Rotation2d(frontShoulderMotor.getAcceleration().getValueAsDouble()).getRadians() * (1.0 / shoulderGearRatio));
        aMatrix.set(1, 0, new Rotation2d(elbowMotor.getAcceleration().getValueAsDouble()).getRadians() * (1.0 / elbowGearRatio));

        return aMatrix;
    }

    @Override
    public Matrix<N2, N2> calculateMMatrix() {
        double c2 = Math.cos(getRelativeElbowAngle().getRadians());
        
        // Moment of inertia (can be pre determined)
        //alex comments -> Moment of Inertia is essentially how resistant an object is to changes in its rotation.
        //the same way that mass is a measure of how much an object resists changes in its linear motion.
        //higher moment of inertia means it's harder to change the rotational speed of the object
        // for a rod rotating about one end: I = (1/3) * m * L^2 -> becomes lower if rotating about center of mass

        // 1/3 mass * length^2
        double I1 = shoulderMass * Math.pow(shoulderLength, 2) / 3.0; 
        double I2 = elbowMass * Math.pow(elbowLength, 2) / 3.0;
        
        double m0_0 = shoulderMass * Math.pow(shoulderLength/2.0, 2) 
                    + elbowMass * (Math.pow(shoulderLength, 2) + Math.pow(elbowLength/2.0, 2))
                    + I1 + I2 
                    + 2.0 * elbowMass * shoulderLength * (elbowLength/2.0) * c2;
        //calculate the inertia of the shoulder joint
        
        double m0_1 = elbowMass * Math.pow(elbowLength/2.0, 2) 
                    + I2 
                    + elbowMass * shoulderLength * (elbowLength/2.0) * c2;
        //calculate the inertia of the elbow joint affects the shoulder joint

        double m1_1 = elbowMass * Math.pow(elbowLength/2.0, 2) + I2;
        //calculate inertia of the elbow joint
        
        // M is symmetric
        mMatrix.set(0, 0, m0_0);
        mMatrix.set(0, 1, m0_1);
        mMatrix.set(1, 0, m0_1); // m1_0 = m0_1
        mMatrix.set(1, 1, m1_1);
        
        return mMatrix;
    }

    @Override
    public Matrix<N2, N2> calculateCMatrix() {
        double s2 = Math.sin(getRelativeElbowAngle().getRadians());
        double theta1_dot = getVelocities().get(0, 0);
        double theta2_dot = getVelocities().get(1, 0);
        
        double c0_0 = -elbowMass * shoulderLength * (elbowLength/2.0) * s2 * theta2_dot;
        double c0_1 = -elbowMass * shoulderLength * (elbowLength/2.0) * s2 * (theta1_dot + theta2_dot);
        double c1_0 = elbowMass * shoulderLength * (elbowLength/2.0) * s2 * theta1_dot;
        
        cMatrix.set(0, 0, c0_0);
        cMatrix.set(0, 1, c0_1);
        cMatrix.set(1, 0, c1_0);
        cMatrix.set(1, 1, 0.0);
        
        return cMatrix;
    }

    @Override
    public Matrix <N2, N1> calculateGMatrix(){
        double g0_0 = (shoulderMass * (shoulderLength / 2.0) + elbowLength * shoulderLength) * GRAVITY * Math.cos(getShoulderAngle().getRadians())
                        + elbowMass * (elbowLength / 2.0) * GRAVITY * Math.cos(getShoulderAngle().getRadians() + getRelativeElbowAngle().getRadians());
        double g1_0 = (elbowMass * (elbowLength / 2.0) * GRAVITY * Math.cos(getShoulderAngle().getRadians() + getRelativeElbowAngle().getRadians()));
        
        gMatrix.set(0, 0, g0_0);
        gMatrix.set(1, 0, g1_0);
        
        return gMatrix;
    }

    @Override
    public Matrix<N2, N1> calculateTorque(Matrix<N2, N1> velocities, Matrix<N2, N1> accelerations) {
        
        // M*accel + C*vel + G
        return calculateMMatrix().times(accelerations)
                .plus(calculateCMatrix().times(velocities))
                .plus(calculateGMatrix());
    }

    @Override
    public Matrix<N2, N2> calculateBackEmf() {
        double kB0_0 = (Math.pow(shoulderGearRatio, 2) * 2.0 * kT) / (kU * r);
        double kB1_1 = (Math.pow(elbowGearRatio, 2) * 1.0 * kT) / (kU * r);

        kBMatrix.set(0, 0, kB0_0);
        kBMatrix.set(0, 1, 0.0);
        kBMatrix.set(1, 0, 0.0);
        kBMatrix.set(1, 1, kB1_1);

        return kBMatrix;
    }

    @Override
    public Matrix<N2, N2> calculateMotorTorque() {
        double B0_0 = shoulderGearRatio * 2.0 * kT / r;
        double B1_1 = elbowGearRatio * 1.0 * kT / r;

        BMatrix.set(0, 0, B0_0);
        BMatrix.set(0, 1, 0.0);
        BMatrix.set(1, 0, 0.0);
        BMatrix.set(1, 1, B1_1);

        return BMatrix;
    }

    @Override 
    public Matrix<N2, N1> calculateVoltage() {
        return BMatrix.inv().times(calculateTorque(targetVelocityMatrix,targetAccelMatrix).plus(kBMatrix.times(targetVelocityMatrix)));
        // return calculateTorque(targetVelocityMatrix, targetAccelMatrix);
    }
}