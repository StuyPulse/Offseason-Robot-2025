package com.stuypulse.robot.subsystems.arm;

import java.util.List;

import org.opencv.core.Core;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;

public class ArmImpl extends Arm {
    // Hardware
    private final TalonFX frontShoulderMotor;
    private final TalonFX backShoulderMotor;
    private final TalonFX elbowMotor;
    
    private final CoreCANcoder shoulderEncoder;
    private final CoreCANcoder elbowEncoder;

    private final Timer timer;

    // Matricies
    private Matrix<N2, N2> mMatrix;
    private Matrix<N2, N2> cMatrix;
    private Matrix<N2, N1> gMatrix;

    private Matrix<N2, N1> vMatrix;
    private Matrix<N2, N1> aMatrix;

    // Physical Constants
    private final double shoulderMass = Constants.DoubleJointedArm.Shoulder.MASS;
    private final double elbowMass = Constants.DoubleJointedArm.Elbow.MASS;  
    private final double shoulderLength = Constants.DoubleJointedArm.Shoulder.LENGTH;
    private final double elbowLength = Constants.DoubleJointedArm.Elbow.LENGTH;  
    private final double GRAVITY = 9.81;

    // Conversion Factors
    private final double shoulderGearRatio = Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
    private final double elbowGearRatio = Constants.DoubleJointedArm.Elbow.GEAR_RATIO;
    private final double TORQUE_TO_VOLTAGE = 1.0 / 12.0; // Converts Nm to volts

    private final PositionVoltage shoulderPositionReq = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage elbowPositionReq = new PositionVoltage(0).withSlot(1);

    // Configuration Space + PathPlanner
    private final ArmConfigurationSpace configSpace; 
    private final ArmPathPlanner pathPlanner;
    private List<ArmTrajectoryPoint> currentTrajectory;
    private int trajectoryIndex = 0;

    public ArmImpl() {

        frontShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.FRONT_MOTOR);
        backShoulderMotor = new TalonFX(Ports.DoubleJointedArm.Shoulder.BACK_MOTOR);
        elbowMotor = new TalonFX(Ports.DoubleJointedArm.Elbow.MOTOR);
        shoulderEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Shoulder.ENCODER);
        elbowEncoder = new CoreCANcoder(Ports.DoubleJointedArm.Elbow.ENCODER);
        
        mMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        cMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        gMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        timer =  new Timer();
        
        configureMotors();

        configSpace = new ArmConfigurationSpace();
        pathPlanner = new ArmPathPlanner(configSpace);
    }

    private void configureMotors() {
        TalonFXConfiguration masterShoulderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerShoulderConfig = new TalonFXConfiguration();
        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        
        // Shoulder Configuration
        Slot0Configs shoulderSlot0 = masterShoulderConfig.Slot0;
        shoulderSlot0.withKP(Settings.DoubleJointedArm.Shoulder.PID.kP)
                    .withKI(Settings.DoubleJointedArm.Shoulder.PID.kI)
                    .withKD(Settings.DoubleJointedArm.Shoulder.PID.kD)
                    .withKS(Settings.DoubleJointedArm.Shoulder.FF.kS)  
                    .withKV(Settings.DoubleJointedArm.Shoulder.FF.kV)  
                    .withKA(Settings.DoubleJointedArm.Shoulder.FF.kA); 

        // Elbow Configuration
        Slot1Configs elbowSlot1 = elbowConfig.Slot1;
        elbowSlot1.withKP(Settings.DoubleJointedArm.Elbow.PID.kP)
                    .withKI(Settings.DoubleJointedArm.Elbow.PID.kI)
                    .withKD(Settings.DoubleJointedArm.Elbow.PID.kD)
                    .withKS(Settings.DoubleJointedArm.Elbow.FF.kS)
                    .withKV(Settings.DoubleJointedArm.Elbow.FF.kV)
                    .withKA(Settings.DoubleJointedArm.Elbow.FF.kA);
        
        // Maaster Motor Configuration
        masterShoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterShoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        frontShoulderMotor.getConfigurator().apply(masterShoulderConfig);

        // Follower Motor Configuration
        followerShoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerShoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        backShoulderMotor.getConfigurator().apply(followerShoulderConfig);

        backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));

        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotor.getConfigurator().apply(elbowConfig);
    }


    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition().getValueAsDouble());
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getVelocities() {
        double a0_0 = shoulderEncoder.getVelocity().getValueAsDouble();
        double a1_0 = elbowEncoder.getVelocity().getValueAsDouble();
        vMatrix.set(0, 0, a0_0);
        vMatrix.set(1, 0, a1_0);
        return vMatrix;
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getAccelerations(){
        
        Pair<Double, Double> AStream = new Pair<>(shoulderEncoder.getVelocity().getValueAsDouble() / timer.get(), 
                                                  elbowEncoder.getVelocity().getValueAsDouble() / timer.get());

        timer.reset();

        aMatrix.set(0, 0, AStream.getSecond()); // Switch? 
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
    public void setTargetAngles(Rotation2d shoulder, Rotation2d elbow) {
        Matrix<N2, N1> tau = calculateTorque();
        
        // Convert torque to volts (Need Gear Ratio)
        double shoulderVolts = tau.get(0, 0) / (shoulderGearRatio * TORQUE_TO_VOLTAGE);
        double elbowVolts = tau.get(1, 0) / (elbowGearRatio* TORQUE_TO_VOLTAGE);
        
        // FeedForward
        frontShoulderMotor.setControl(
            shoulderPositionReq
                .withPosition(shoulder.getRotations())
                .withFeedForward(shoulderVolts)
        );
        
        elbowMotor.setControl(
            elbowPositionReq
                .withPosition(elbow.getRotations())
                .withFeedForward(elbowVolts)
        );
    }

    // Convert target to joint angles
    private double[] toJointAngles(Translation2d point) {
        double d = Math.sqrt(Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2));
        if (d > shoulderLength + elbowLength || d < Math.abs(shoulderLength - elbowLength)) {
            return null; 
        }
        
        double theta2 = Math.acos((Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2) - shoulderLength*shoulderLength - elbowLength*elbowLength) 
                        / (2 * shoulderLength * elbowLength));
        double theta1 = Math.atan2(point.getY(), point.getX()) - Math.atan2(elbowLength * Math.sin(theta2), 
                                            shoulderLength + elbowLength * Math.cos(theta2));
        
        return new double[]{theta1, theta2};
    }

    @Override
    public void setTargetPosition(Translation2d target) {
        double[] targetAngles = toJointAngles(target);
        if (targetAngles == null) return; 

        // Plan path
        List<Translation2d> path = pathPlanner.findPath(
            getShoulderAngle(),
            getElbowAngle(),
            new Rotation2d(targetAngles[0]),
            new Rotation2d(targetAngles[1])
        );

        // Generate trajectory
        currentTrajectory = new ArmSpline(
            new double[]{
                getShoulderAngle().getRadians(),
                getElbowAngle().getRadians()
            },
            new double[]{0, 0}, // Start velocity
            new double[]{0, 0}, // Start acceleration
            targetAngles,
            new double[]{0, 0}, // End velocity
            new double[]{0, 0}, // End acceleration
            2.0     // Duration
        ).sampleTrajectory(50);
        
        trajectoryIndex = 0;
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
        if (currentTrajectory != null && trajectoryIndex < currentTrajectory.size()) {
            ArmTrajectoryPoint setpoint = currentTrajectory.get(trajectoryIndex++);
            setTargetAngles(
                Rotation2d.fromRadians(setpoint.theta1),
                Rotation2d.fromRadians(setpoint.theta2)
            );
        }

        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getRadians());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngle().getRadians());
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());

        SmartDashboard.putNumber("Arm/Shoulder Torque", calculateTorque().get(0, 0));
        SmartDashboard.putNumber("Arm/Elbow Torque", calculateTorque().get(1, 0));
    }
}