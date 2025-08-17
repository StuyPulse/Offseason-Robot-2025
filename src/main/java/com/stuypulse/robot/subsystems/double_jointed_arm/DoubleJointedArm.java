package com.stuypulse.robot.subsystems.double_jointed_arm;

import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleJointedArm extends SubsystemBase {

    public static DoubleJointedArm instance;
    
    public static DoubleJointedArm getInstance() {
        if (instance == null) {
            instance = new DoubleJointedArm(Robot.isReal() ? new DoubleJointedArmIOReal() : new DoubleJointedArmIOSim());
            instance = new DoubleJointedArm(Robot.isReal() ? new DoubleJointedArmIOReal() : new DoubleJointedArmIOSim());
        }
        return instance;
    }

    private DoubleJointedArmIO io;
    private DoubleJointedArmIOInputsAutoLogged inputs = new DoubleJointedArmIOInputsAutoLogged();

    private ArmState state;
    private ArmState storedState;
    private boolean intermediate;

    private DoubleJointedArmVisualizer visualizerMeasured;
    private DoubleJointedArmVisualizer visualizerSetpoint;

    public DoubleJointedArm(DoubleJointedArmIO io) {
        this.io = io;
        state = ArmState.STOW;
        storedState = ArmState.STOW;
        intermediate = false;

        shoulderPID.setIntegratorRange(-2, 2);
        elbowPID.setIntegratorRange(-1, 1);

        // Idk abt this one
        shoulderPID.enableContinuousInput(-Math.PI, Math.PI);
        elbowPID.enableContinuousInput(-Math.PI, Math.PI);

        visualizerMeasured = new DoubleJointedArmVisualizer("ArmMeasured", null);
        visualizerSetpoint = new DoubleJointedArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));

        configSpace = new ArmConfigurationSpace();
        pathPlanner = new ArmPathPlanner(configSpace);
    }

    public enum ArmState {
        STOW(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0)),
        INT(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(45)),
        L4_FRONT(Rotation2d.fromDegrees(30), Rotation2d.fromDegrees(60)),
        L4_BACK(Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(-60)),
        L3_FRONT(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(45)),
        L3_BACK(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-45)),
        L2_FRONT(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(30)),
        L2_BACK(Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(-30)),
        L1_FRONT(Rotation2d.fromDegrees(75), Rotation2d.fromDegrees(15)),
        L1_BACK(Rotation2d.fromDegrees(105), Rotation2d.fromDegrees(-15)),

        CUSTOM(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) {
            @Override
            public void setAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
                this.shoulderAngle = shoulderAngle;
                this.elbowAngle = elbowAngle;
            }
        };

        private ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle){
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        public synchronized void setAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
            throw new UnsupportedOperationException("Only CUSTOM state supports dynamic angles");
        }

        protected Rotation2d shoulderAngle;
        protected Rotation2d elbowAngle;

        public Rotation2d getShoulderTarget() { return shoulderAngle; }
        public Rotation2d getElbowTarget() { return elbowAngle; }

        public ArmState getOpposite() {
            switch (this) {
                case L4_FRONT: return L4_BACK;
                case L4_BACK: return L4_FRONT;
                case L3_FRONT: return L3_BACK;
                case L3_BACK: return L3_FRONT;
                case L2_FRONT: return L2_BACK;
                case L2_BACK: return L2_FRONT;
                case L1_FRONT: return L1_BACK;
                case L1_BACK: return L1_FRONT;
                default: return STOW;
            }
        }

        public boolean isFront() {
            return this.name().endsWith("FRONT") || this.equals(ArmState.STOW);
        }
    }

    public void setState(ArmState state) { 
        this.state = state; 
    }

    public void switchSides() { 
        if (!intermediate) {
            intermediate = true;
            storedState = state.getOpposite();
            setState(ArmState.INT);
        }
    }

    /* GETTERS */

    @AutoLogOutput
    public ArmState getState() {
        return state;
    }

    @AutoLogOutput (key = "DoubleJointedArm/Joints/Shoulder/AngleRad")
    public Rotation2d getShoulderAngleRad() {
        return Rotation2d.fromRadians(inputs.shoulderAngle);
    }

    @AutoLogOutput (key = "DoubleJointedArm/Joints/Shoulder/AngleDeg")
    public double getShoulderAngleDeg() {
        return Rotation2d.fromRadians(inputs.shoulderAngle).getDegrees();
    }
    
    @AutoLogOutput (key = "DoubleJointedArm/Joints/Elbow/AngleRad")
    public Rotation2d getElbowAngleRad() {
        return Rotation2d.fromRadians(inputs.elbowAngle);
    }
    
    @AutoLogOutput (key = "DoubleJointedArm/Joints/Elbow/AngleDeg")
    public double getElbowAngleDeg() {
        return Rotation2d.fromRadians(inputs.elbowAngle).getDegrees();
    }
    
    @AutoLogOutput (key = "DoubleJointedArm/Arm/EndPosition")
    public Translation2d getEndPosition() { // Forward Kinematics
        Rotation2d shoulder = getShoulderAngleRad();
        Rotation2d elbow = getElbowAngleRad();

        Transform2d startPoint = new Transform2d(0.0, Constants.DoubleJointedArm.BASE_HEIGHT, new Rotation2d());

        return startPoint
            .plus(new Transform2d(Constants.DoubleJointedArm.Shoulder.LENGTH, 0.0, shoulder))
            .plus(new Transform2d(Constants.DoubleJointedArm.Elbow.LENGTH, 0.0, elbow.minus(Rotation2d.fromRadians(Math.PI - shoulder.getRadians()))))
            .getTranslation();
    }

    @AutoLogOutput (key = "DoubleJointedArm/Joints/Shoulder/AtTarget")
    public boolean isShoulderAtTarget() {
        double currentAngle = inputs.shoulderAngle;
        double targetAngle = state.getShoulderTarget().getRadians();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Shoulder.TOLERANCE;
    }

    @AutoLogOutput (key = "DoubleJointedArm/Joints/Elbow/AtTarget")
    public boolean isElbowAtTarget() {
        double currentAngle = inputs.elbowAngle;
        double targetAngle = state.getElbowTarget().getRadians();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Elbow.TOLERANCE;
    }
    
    @AutoLogOutput (key = "DoubleJointedArm/Arm/AtTarget")
    public boolean isArmAtTarget() {
        return isShoulderAtTarget() && isElbowAtTarget();
    }

    // Matricies XXX - I don't know if this is how you properly initialize them but they were giving runtime errors before i did ts
    private Matrix<N2, N2> mMatrix = new Matrix<N2, N2>(new SimpleMatrix(2, 2));
    private Matrix<N2, N2> cMatrix = new Matrix<N2, N2>(new SimpleMatrix(2, 2));
    private Matrix<N2, N1> gMatrix = new Matrix<N2, N1>(new SimpleMatrix(2, 2));
    private Matrix<N2, N1> vMatrix = new Matrix<N2, N1>(new SimpleMatrix(2, 2));
    private Matrix<N2, N1> aMatrix = new Matrix<N2, N1>(new SimpleMatrix(2, 2));

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

    private List<ArmSpline> trajectorySplines = new ArrayList<>();
    private int currentSplineIndex = 0;
    private double splineStartTime = 0;
    private boolean isFollowingTrajectory = false;

    // PID Controllers
    private final PIDController shoulderPID = new PIDController(
        Settings.DoubleJointedArm.Shoulder.PID.kP,
        Settings.DoubleJointedArm.Shoulder.PID.kI,
        Settings.DoubleJointedArm.Shoulder.PID.kD
    );

    private final PIDController elbowPID = new PIDController(
        Settings.DoubleJointedArm.Elbow.PID.kP,
        Settings.DoubleJointedArm.Elbow.PID.kI,
        Settings.DoubleJointedArm.Elbow.PID.kD
    );

    /*
     * @param position The angles of the joints, (θ1, θ2). Note that θ2 is relative to the first joint, not the horizontal
     * @param velocity The velocities of the joints in rad/s
     * @param acceleration The accelerations of the joints in rad/s^2
     * @param velocity The velocities of the joints in rad/s
     * @param acceleration The accelerations of the joints in rad/s^2
     */
    public Matrix<N2, N1> feedforwardVoltage(Matrix<N2, N1> position, Matrix<N2, N1> velocity, Matrix<N2, N1> acceleration) {
        Matrix<N2, N1> torque = calculateTorque();
        
        return VecBuilder.fill(
            DCMotor.getKrakenX60(1).withReduction(shoulderGearRatio).getVoltage(torque.get(0, 0), velocity.get(0, 0)),
            DCMotor.getKrakenX60(1).withReduction(elbowGearRatio).getVoltage(torque.get(1, 0), velocity.get(1, 0)));
    }

    public Rotation2d getShoulderAngle() {
        return new Rotation2d(inputs.shoulderAngle);
    }

    public Rotation2d getElbowAngle() {
        return new Rotation2d(inputs.elbowAngle);
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getVelocities() {
        vMatrix.set(0, 0, inputs.shoulderAngularVel);
        vMatrix.set(1, 0, inputs.elbowAngularVel);
        
        return vMatrix;
    }

    // Return 2x1 Matrix
    public Matrix<N2, N1> getAccelerations(){
        aMatrix.set(0, 0, inputs.shoulderAngularAccel); 
        aMatrix.set(1, 0, inputs.elbowAngularAccel);

        return aMatrix;
    }

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

    public void setTargetAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle) {

        List<Translation2d> path = pathPlanner.findPath(
            getShoulderAngle(),
            getElbowAngle(),
            shoulderAngle,
            elbowAngle
        );
    
        if (path.isEmpty()) {
            System.out.println("No valid path found!");
            return;
        }
    
        // Generate splines between waypoints
        trajectorySplines.clear();
        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d start = path.get(i);
            Translation2d end = path.get(i+1);
            
            // Calculate approximate velocities (tangent to path)
            Translation2d prevVel = (i == 0) ? new Translation2d() : 
                path.get(i).minus(path.get(i-1)).div(Settings.DoubleJointedArm.PATH_DT);
            Translation2d nextVel = i == path.size()-2 ? new Translation2d() : 
                path.get(i+2).minus(path.get(i+1)).div(Settings.DoubleJointedArm.PATH_DT);
            
            trajectorySplines.add(new ArmSpline(
                new double[]{start.getX(), start.getY()},  // Start angles
                new double[]{prevVel.getX(), prevVel.getY()},  // Start velocities
                new double[]{0, 0},  // Start accelerations
                new double[]{end.getX(), end.getY()},  // End angles
                new double[]{nextVel.getX(), nextVel.getY()},  // End velocities
                new double[]{0, 0},  // End accelerations
                Settings.DoubleJointedArm.SPLINE_DURATION  // Segment duration
            ));
        }
        
        currentSplineIndex = 0;
        splineStartTime = Timer.getFPGATimestamp();
        isFollowingTrajectory = true;
    }

    @Override
    public void periodic() {

        double shoulderAngleRad = inputs.shoulderAngle;
        double elbowRelativeRad = inputs.elbowAngle;

        double shoulderVelRad = inputs.shoulderAngularVel;
        double elbowRelativeVelRad = inputs.elbowAngularVel;

        double shoulderAccelRad = inputs.shoulderAngularAccel;
        double elbowRelativeAccelRad = inputs.elbowAngularAccel;
        
        if (intermediate && isArmAtTarget()) {
            setState(storedState);
            intermediate = false;
        }

        if (isFollowingTrajectory) {
            double currentTime = Timer.getFPGATimestamp() - splineStartTime;
            ArmSpline currentSpline = trajectorySplines.get(currentSplineIndex);
            double progress = Math.min(currentTime / currentSpline.getDuration(), 1.0);
            
            ArmTrajectoryPoint setpoint = currentSpline.getPoint(progress);
            
            // Calculate feedforward
            Matrix<N2, N1> ff = feedforwardVoltage(
                VecBuilder.fill(setpoint.theta1, setpoint.theta2),
                VecBuilder.fill(setpoint.omega1, setpoint.omega2),
                VecBuilder.fill(setpoint.alpha1, setpoint.alpha2)
            );
            
            // PID corrections
            double shoulderCorrection = shoulderPID.calculate(
                getShoulderAngle().getRadians(),
                setpoint.theta1
            );
            
            double elbowCorrection = elbowPID.calculate(
                getElbowAngle().getRadians(),
                setpoint.theta2
            );
            
            // Command motors
            io.controlShoulder(
                Rotation2d.fromRadians(setpoint.theta1),
                ff.get(0, 0) + shoulderCorrection
            );
            
            io.controlElbow(
                Rotation2d.fromRadians(setpoint.theta2),
                ff.get(1, 0) + elbowCorrection
            );
            
            // Transition logic
            if (currentTime >= currentSpline.getDuration()) {
                if (currentSplineIndex < trajectorySplines.size() - 1) {
                    currentSplineIndex++;
                    splineStartTime = Timer.getFPGATimestamp();
                    shoulderPID.reset();
                    elbowPID.reset();
                } else {
                    isFollowingTrajectory = false;
                    state = ArmState.CUSTOM;
                    state.setAngles(
                        Rotation2d.fromRadians(setpoint.theta1),
                        Rotation2d.fromRadians(setpoint.theta2)
                    );
                }
            }
        }

        visualizerMeasured.update(shoulderAngleRad, elbowRelativeRad);
        visualizerSetpoint.update(state.getShoulderTarget().getRadians(), state.getElbowTarget().getRadians());

        io.updateInputs(inputs);
        Logger.processInputs("DoubleJointedArm/", inputs);
        Logger.recordOutput("DoubleJointedArm/Arm/State", state.name());
        Logger.recordOutput("DoubleJointedArm/Joints/Shoulder/TargetDeg", state.getShoulderTarget().getDegrees());
        Logger.recordOutput("DoubleJointedArm/Joints/Elbow/TargetDeg", state.getElbowTarget().getDegrees());
        Logger.recordOutput("DoubleJointedArm/Arm/AtTarget", isArmAtTarget());
    }
}