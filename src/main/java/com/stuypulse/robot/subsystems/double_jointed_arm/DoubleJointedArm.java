package com.stuypulse.robot.subsystems.double_jointed_arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleJointedArm extends SubsystemBase {

    public static DoubleJointedArm instance;
    
    public static DoubleJointedArm getInstance() {
        if (instance == null) {
            instance = new DoubleJointedArm(Robot.isReal() ? new DoubleJointedArmIOReal() : new DoubleJointedArmIOReal()); // will fix once sim is finished
        }
        return instance;
    }

    private DoubleJointedArmIO io;
    private DoubleJointedArmIOInputsAutoLogged inputs = new DoubleJointedArmIOInputsAutoLogged();

    private ArmState state;
    private ArmState storedState;
    private boolean intermediate;

    public DoubleJointedArm(DoubleJointedArmIO io) {
        this.io = io;
        state = ArmState.STOW;
        storedState = ArmState.STOW;
        intermediate = false;
    }

    public enum ArmState {
        STOW(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        INT(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
        L4_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L4_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L3_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L3_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L2_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L2_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L1_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L1_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10));

        private ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle){
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        private Rotation2d shoulderAngle;
        private Rotation2d elbowAngle;

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DoubleJointedArm", inputs);
        Logger.recordOutput("DoubleJointedArm/CurrentState", state.name());
        Logger.recordOutput("DoubleJointedArm/ShoulderTarget", state.getShoulderTarget().getDegrees());
        Logger.recordOutput("DoubleJointedArm/ElbowTarget", state.getElbowTarget().getDegrees());
        Logger.recordOutput("DoubleJointedArm/AtTarget", isArmAtTarget());

        double shoulderAngleRad = Math.toRadians(inputs.shoulderAngle);
        double elbowRelativeRad = Math.toRadians(inputs.elbowAngle - inputs.shoulderAngle);

        double shoulderVelRad = Math.toRadians(inputs.shoulderAngularVel);
        double elbowRelativeVelRad = Math.toRadians(inputs.elbowAngularVel - inputs.shoulderAngularVel);

        double shoulderAccelRad = Math.toRadians(inputs.shoulderAngularAccel);
        double elbowRelativeAccelRad = Math.toRadians(inputs.elbowAngularAccel - inputs.shoulderAngularAccel);

        Matrix<N2, N1> positions = VecBuilder.fill(shoulderAngleRad, elbowRelativeRad);
        Matrix<N2, N1> velocities = VecBuilder.fill(shoulderVelRad, elbowRelativeVelRad);
        Matrix<N2, N1> accelerations = VecBuilder.fill(shoulderAccelRad, elbowRelativeAccelRad);
        
        Matrix<N2, N1> ff = feedforwardVoltage(positions, velocities, accelerations);
        
        if (intermediate && isArmAtTarget()) {
            setState(storedState);
            intermediate = false;
        }

        io.controlShoulder(state.getShoulderTarget(), ff.get(0, 0));
        io.controlElbow(state.getElbowTarget(), ff.get(1, 0));
    }

    /* GETTERS */

    @AutoLogOutput
    public ArmState getState() {
        return state;
    }
    
    @AutoLogOutput
    public Rotation2d getCurrentShoulderAngle() {
        return Rotation2d.fromDegrees(inputs.shoulderAngle);
    }
    
    @AutoLogOutput
    public Rotation2d getCurrentElbowAngle() {
        return Rotation2d.fromDegrees(inputs.elbowAngle);
    }
    
    @AutoLogOutput
    public Translation2d getEndPosition() { // Forward Kinematics
        Rotation2d shoulder = getCurrentShoulderAngle();
        Rotation2d elbow = getCurrentElbowAngle();

        Transform2d startPoint = new Transform2d(0.0, Constants.DoubleJointedArm.BASE_HEIGHT, new Rotation2d());

        return startPoint
            .plus(new Transform2d(Constants.DoubleJointedArm.Shoulder.LENGTH, 0.0, shoulder))
            .plus(new Transform2d(Constants.DoubleJointedArm.Elbow.LENGTH, 0.0, elbow.minus(Rotation2d.fromRadians(180.0 - shoulder.getRadians()))))
            .getTranslation();
    }

    @AutoLogOutput
    public boolean isShoulderAtTarget() {
        double currentAngle = inputs.shoulderAngle;
        double targetAngle = state.getShoulderTarget().getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Shoulder.ANGLE_TOLERANCE.getDegrees();
    }

    @AutoLogOutput
    public boolean isElbowAtTarget() {
        double currentAngle = inputs.elbowAngle;
        double targetAngle = state.getElbowTarget().getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Elbow.ANGLE_TOLERANCE.getDegrees();
    }

    @AutoLogOutput
    public boolean isArmAtTarget() {
        return isShoulderAtTarget() && isElbowAtTarget();
    }

    private final Matrix<N2, N2> M = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N2> C = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N1> Tg = new Matrix<>(Nat.N2(), Nat.N1());

    private final double m1 = Constants.DoubleJointedArm.Shoulder.MASS;   
    private final double m2 = Constants.DoubleJointedArm.Elbow.MASS;          
    private final double l1 = Constants.DoubleJointedArm.Shoulder.LENGTH;  
    private final double l2 = Constants.DoubleJointedArm.Elbow.LENGTH; // we never go this far
    private final double r1 = Constants.DoubleJointedArm.Shoulder.LENGTH/2; 
    private final double r2 = Constants.DoubleJointedArm.Elbow.LENGTH/2; // assuming relatively uniform mass distribution
    private final double I1 = Constants.DoubleJointedArm.Elbow.LENGTH/2;
    private final double I2 = Constants.DoubleJointedArm.Elbow.LENGTH/2; // idfk how to find this
    
    private final double GRAVITY = 9.81; // m/s²

    private final double SHOULDER_GEAR_RATIO = Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
    private final double ELBOW_GEAR_RATIO = Constants.DoubleJointedArm.Elbow.GEAR_RATIO;

    /*
     * @param position The angles of the joints, (θ1, θ2). Note that θ2 is relative to the first joint, not the horizontal
     * @param velocity The velocities of the joints in deg/s
     * @param acceleration The accelerations of the joints in deg/s^2
     */
    public Matrix<N2, N1> feedforwardVoltage(Matrix<N2, N1> position, Matrix<N2, N1> velocity, Matrix<N2, N1> acceleration) {
        Matrix<N2, N1> torque = 
            calcM(position)
                .times(acceleration)
                .plus(calcC(position, velocity).times(velocity)
                .plus(calcTg(position)));
        
        return VecBuilder.fill(
            DCMotor.getKrakenX60(1).withReduction(SHOULDER_GEAR_RATIO).getVoltage(torque.get(0, 0), velocity.get(0, 0)),
            DCMotor.getKrakenX60(1).withReduction(ELBOW_GEAR_RATIO).getVoltage(torque.get(1, 0), velocity.get(1, 0)));
    }

    public Matrix<N2, N2> calcM(Matrix<N2, N1> position) {
        M.set(
            0,
            0,
            m1 * Math.pow(r1, 2.0)
                + m2 * (Math.pow(l1, 2.0) + Math.pow(r2, 2.0))
                + I1 + I2 // here, I2 is not needed in virtual four bar double jointed arms because each joint is independently controlled
                + 2
                    * m1
                    * m2
                    * r2
                    * Math.cos(position.get(1, 0))
        );

        M.set(
            1,
            0,
            m2 * Math.pow(r2, 2.0) + I2 + m2*l1*r2*Math.cos(position.get(1,0))
            // I2 is not needed in above line if you have a virtual four bar, for same reasons as above
        );

        M.set(
            0,
            1,
            m2 * Math.pow(r2, 2.0) + I2 + m2*l1*r2*Math.cos(position.get(1,0))
            // however, I2 is always needed for the second joint COM calculations
        );

        M.set(
            1,
            1,
            m2 * Math.pow(r2, 2.0) + I2
        );

        return M;
    }

    public Matrix<N2, N2> calcC(Matrix<N2, N1> position, Matrix<N2, N1> velocity) {
        C.set(
            0,
            0,
            -m2
                * l1
                * r2
                * Math.sin(position.get(1, 0))
                * velocity.get(1, 0));
        C.set(
            1,
            0,
            m2
                * l1
                * r2
                * Math.sin(position.get(1, 0))
                * velocity.get(0, 0));
        C.set(
            0,
            1,
            -m1
                * l1
                * r2
                * Math.sin(position.get(1, 0))
                * (velocity.get(0, 0) + velocity.get(1, 0)));
        return C;
    }

    public Matrix<N2, N1> calcTg(Matrix<N2, N1> position) {
        Tg.set(
            0,
            0,
            (m1 * r2 + m2 * l1)
                    * GRAVITY
                    * Math.cos(position.get(0, 0))
                + m2
                    * r2
                    * GRAVITY
                    * Math.cos(position.get(0, 0) + position.get(1, 0)));
        Tg.set(
            1,
            0,
            m2 * r2 * GRAVITY * Math.cos(position.get(0, 0) + position.get(1, 0)));
        return Tg;
    }
}
