package com.stuypulse.robot.subsystems.double_jointed_arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Devices;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DoubleJointedArmIOReal implements DoubleJointedArmIO {
    // Physical Constants
    private final double SHOULDER_MASS = Constants.DoubleJointedArm.Shoulder.MASS;      // kg
    private final double ELBOW_MASS = Constants.DoubleJointedArm.Elbow.MASS;            // kg
    private final double SHOULDER_LENGTH = Constants.DoubleJointedArm.Shoulder.LENGTH;  // m
    private final double ELBOW_LENGTH = Constants.DoubleJointedArm.Elbow.LENGTH;        // m
    private final double GRAVITY = 9.81; // m/s²

    // Conversion Factors
    private final double SHOULDER_GEAR_RATIO = Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
    private final double ELBOW_GEAR_RATIO = Constants.DoubleJointedArm.Elbow.GEAR_RATIO;
    private final double TORQUE_TO_VOLTAGE = 1.0 / 12.0; // Convert Nm to volts

    // Hardware
    private final TalonFX shoulder;
    private final CANcoder shoulderEncoder;
    private final TalonFX elbow;
    private final CANcoder elbowEncoder;

    // Matricies
    private final Matrix<N2, N2> mMatrix = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N2> cMatrix = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N1> gMatrix = new Matrix<>(Nat.N2(), Nat.N1());

    // Status Signals (refreshed periodically upon initialization)
    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<AngularVelocity> shoulderAngularVel;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccel;
    private final StatusSignal<Voltage> shoulderAppliedVoltage;
    private final StatusSignal<Current> shoulderCurrentAmps;

    private final StatusSignal<Angle> elbowAngle;
    private final StatusSignal<AngularVelocity> elbowAngularVel;
    private final StatusSignal<AngularAcceleration> elbowAngularAccel;
    private final StatusSignal<Voltage> elbowAppliedVoltage;
    private final StatusSignal<Current> elbowCurrentAmps;


    public DoubleJointedArmIOReal() {
        super();
        shoulder = new TalonFX(Ports.DoubleJointedArm.SHOULDER_MOTOR);
        shoulderEncoder = new CANcoder(Ports.DoubleJointedArm.SHOULDER_ENCODER);
        elbow = new TalonFX(Ports.DoubleJointedArm.ELBOW_MOTOR);
        elbowEncoder = new CANcoder(Ports.DoubleJointedArm.ELBOW_ENCODER);

        // Configs
        Devices.DoubleJointedArm.Shoulder.motor_config.configure(shoulder);
        shoulderEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Shoulder.cc_config);
        Devices.DoubleJointedArm.Elbow.motor_config.configure(elbow);
        elbowEncoder.getConfigurator().apply(Devices.DoubleJointedArm.Elbow.cc_config);

        // Status Signal initialization
        shoulderAngle = shoulder.getPosition();
        shoulderAngularVel = shoulder.getVelocity();
        shoulderAngularAccel = shoulder.getAcceleration();
        shoulderAppliedVoltage = shoulder.getMotorVoltage();
        shoulderCurrentAmps = shoulder.getTorqueCurrent();

        elbowAngle = elbow.getPosition();
        elbowAngularVel = elbow.getVelocity();
        elbowAngularAccel = elbow.getAcceleration();
        elbowAppliedVoltage = elbow.getMotorVoltage();
        elbowCurrentAmps = elbow.getTorqueCurrent();

        // Set update freqs
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            shoulderAngle,
            shoulderAngularVel,
            shoulderAngularAccel,
            shoulderAppliedVoltage,

            elbowAngle,
            elbowAngularVel,
            elbowAngularAccel,
            elbowAppliedVoltage);

        // Higher framerate recommended
        shoulderCurrentAmps.setUpdateFrequency(250);
        elbowCurrentAmps.setUpdateFrequency(250);
    }

    // STATES
    @Override
    public void updateInputs(DoubleJointedArmIOInputs inputs) { // called periodically in subsystem
        // Clear cache
        BaseStatusSignal.refreshAll(
            shoulderAngle, shoulderAngularVel, shoulderAngularAccel,
            shoulderAppliedVoltage, shoulderCurrentAmps,
            elbowAngle, elbowAngularVel, elbowAngularAccel,
            elbowAppliedVoltage, elbowCurrentAmps
        );

        inputs.data =
        new DoubleJointedArmIOData(
            shoulder.isConnected(),
            elbow.isConnected(),
            Units.rotationsToDegrees(shoulderAngle.getValueAsDouble()),
            shoulderAngularVel.getValueAsDouble(),
            shoulderAngularAccel.getValueAsDouble(),
            shoulderAppliedVoltage.getValueAsDouble(),
            shoulderCurrentAmps.getValueAsDouble(),
            Units.rotationsToDegrees(elbowAngle.getValueAsDouble()),
            elbowAngularVel.getValueAsDouble(),
            elbowAngularAccel.getValueAsDouble(),
            elbowAppliedVoltage.getValueAsDouble(),
            elbowCurrentAmps.getValueAsDouble()
        );
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderAngle.getValueAsDouble());
    }

    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(shoulderAngle.getValueAsDouble());
    }

    /* CONTROL */
    @Override
    public void runTorqueCurrentShoulder(Rotation2d targetRotation, double torqueCurrentAmps) {
        shoulder.setControl(new MotionMagicTorqueCurrentFOC(targetRotation.getRotations()).withFeedForward(torqueCurrentAmps));
    }

    @Override
    public void runTorqueCurrentElbow(Rotation2d targetRotation, double torqueCurrentAmps) {
        elbow.setControl(new MotionMagicTorqueCurrentFOC(targetRotation.getRotations()).withFeedForward(torqueCurrentAmps));
    }

    @Override
    public void runVoltsShoulder(double volts) { // ONLY voltage override
        shoulder.setControl(new VoltageOut(volts));
    }

    @Override
    public void runVoltsElbow(double volts) { // ONLY voltage override
        elbow.setControl(new VoltageOut(volts));
    }

    /* STATES */
    @Override
    public boolean isShoulderAtTarget(Rotation2d shoulderTargetAngle) {
        double currentAngle = getShoulderAngle().getDegrees();
        double targetAngle = shoulderTargetAngle.getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Elbow.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isElbowAtTarget(Rotation2d elbowTargetAngle) {
        double currentAngle = getElbowAngle().getDegrees();
        double targetAngle = elbowTargetAngle.getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Elbow.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isArmAtTarget(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle) {
        return isShoulderAtTarget(shoulderTargetAngle) && isElbowAtTarget(elbowTargetAngle);
    }

    /* MATH */
    public Translation2d getEndPosition() { // Forward Kinematics
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();

        Transform2d startPoint = new Transform2d(0.0, Constants.DoubleJointedArm.BASE_HEIGHT, new Rotation2d());

        return startPoint
            .plus(new Transform2d(Constants.DoubleJointedArm.Shoulder.LENGTH, 0.0, shoulder))
            .plus(new Transform2d(Constants.DoubleJointedArm.Elbow.LENGTH, 0.0, elbow.minus(Rotation2d.fromDegrees(180.0 - shoulder.getDegrees()))))
            .getTranslation();
    }

    public Matrix<N2, N2> calculateMMatrix() { // Mass Inertia Matrix
        double m0_0 = SHOULDER_MASS * Math.pow(SHOULDER_LENGTH, 2) / 2
        + ELBOW_MASS * (Math.pow(SHOULDER_LENGTH, 2) + Math.pow(ELBOW_LENGTH / 2, 2))
        + SHOULDER_LENGTH / shoulderAngularVel.getValueAsDouble() // Moment of Inertia (Shoulder)
        + ELBOW_LENGTH / elbowEncoder.getVelocity().getValueAsDouble() // Moment of Inertia (Elbow)
        + 2 * ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.cos(getElbowAngle().getDegrees());

        double m0_1 = ELBOW_MASS * Math.pow((ELBOW_LENGTH / 2), 2) 
        + ELBOW_LENGTH / elbowEncoder.getVelocity().getValueAsDouble() // Moment of Inertia (Elbow)
        + ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.cos(getElbowAngle().getDegrees());

        double m1_0 = ELBOW_MASS * Math.pow((ELBOW_LENGTH / 2), 2) 
        + ELBOW_LENGTH / elbowEncoder.getVelocity().getValueAsDouble() // Moment of Inertia (Elbow)
        + ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.cos(getElbowAngle().getDegrees());

        double m1_1 = ELBOW_MASS * Math.pow((ELBOW_LENGTH / 2), 2) 
        + ELBOW_LENGTH / elbowEncoder.getVelocity().getValueAsDouble(); // Moment of Inertia (Elbow)

        mMatrix.set(0, 0, m0_0);
        mMatrix.set(0, 1, m0_1);
        mMatrix.set(1, 0, m1_0);
        mMatrix.set(1, 1, m1_1);

        return mMatrix;
    }

    public Matrix<N2, N2> calculateCMatrix() { // Centrifugal + Coriolis Matrix
        double c0_0 = - ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.sin(getShoulderAngle().getDegrees() + getElbowAngle().getDegrees()) * getElbowAngle().getDegrees();
        double c1_0 = - ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.sin(getShoulderAngle().getDegrees() + getElbowAngle().getDegrees()) * (getShoulderAngle().getDegrees() + getElbowAngle().getDegrees());
        double c0_1 = ELBOW_MASS * SHOULDER_LENGTH * (ELBOW_LENGTH / 2) * Math.sin(getShoulderAngle().getDegrees() + getElbowAngle().getDegrees()) * getShoulderAngle().getDegrees();
        double c1_1 = 0;

        cMatrix.set(0, 0, c0_0);
        cMatrix.set(1, 0, c1_0);
        cMatrix.set(0, 1, c0_1);
        cMatrix.set(1, 1, c1_1);
        return cMatrix;
    }

    public Matrix<N2, N1> calculateGMatrix() { // Torque due to Gravity Matrix 
        double g0_0 = (SHOULDER_MASS * (SHOULDER_LENGTH / 2) + ELBOW_LENGTH * SHOULDER_LENGTH) * GRAVITY * Math.cos(getShoulderAngle().getDegrees()
        + ELBOW_MASS * (ELBOW_LENGTH / 2) * GRAVITY * Math.cos(getShoulderAngle().getDegrees() + getElbowAngle().getDegrees()));
        double g0_1 = (ELBOW_MASS * (ELBOW_LENGTH / 2) * GRAVITY * Math.cos(getShoulderAngle().getDegrees() + getElbowAngle().getDegrees()));

        gMatrix.set(0, 0, g0_0);
        gMatrix.set(1, 0, g0_1);
        
        return gMatrix;    
    }

}
