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

    public DoubleJointedArm(DoubleJointedArmIO io) {
        this.io = io;
        this.state = ArmState.STOW;
    }

    public enum ArmState {
        STOW(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L4_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L4_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L3_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L3_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L2_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L2_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L1_FRONT(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10)),
        L1_BACK(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10));

        private Rotation2d shoulderAngle;
        private Rotation2d elbowAngle;

        private ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle){
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        } 

        public Rotation2d getShoulderTargetAngle() { return shoulderAngle; }
        public Rotation2d getElbowTargetAngle() { return elbowAngle; }

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
            return this.name().endsWith("FRONT");
        }
    }

    public void setState(ArmState state) { this.state = state; }
    public void switchSides() { state = state.getOpposite(); }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DoubleJointedArm", inputs);

        Logger.recordOutput("DoubleJointedArm/CurrentState", state.name());
        Logger.recordOutput("DoubleJointedArm/ShoulderTarget", state.getShoulderTargetAngle().getDegrees());
        Logger.recordOutput("DoubleJointedArm/ElbowTarget", state.getElbowTargetAngle().getDegrees());
        Logger.recordOutput("DoubleJointedArm/AtTarget", isArmAtTarget(state.getElbowTargetAngle(), state.getShoulderTargetAngle()));

        // PUT ALL CALLS TO CONTROL METHODS HERE
        Matrix<N2, N1> positions = VecBuilder.fill(
            Math.toRadians(inputs.shoulderAngle), 
            Math.toRadians(inputs.elbowAngle)
        );
    
        Matrix<N2, N1> velocities = VecBuilder.fill(
            Math.toRadians(inputs.shoulderAngularVel),
            Math.toRadians(inputs.elbowAngularVel)
        );
    
        Matrix<N2, N1> accelerations = VecBuilder.fill(
            Math.toRadians(inputs.shoulderAngularAccel),
            Math.toRadians(inputs.elbowAngularAccel)
        );

        Matrix<N2, N1> ff = feedforwardVoltage(positions, velocities, accelerations);

        io.controlShoulder(state.getShoulderTargetAngle(), ff.get(0, 0));
        io.controlElbow(state.getElbowTargetAngle(), ff.get(1, 0));
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
    public boolean atTarget() {
        return isArmAtTarget(state.getShoulderTargetAngle(), state.getElbowTargetAngle());
    }

    @AutoLogOutput
    public boolean isShoulderAtTarget(Rotation2d shoulderTargetAngle) {
        double currentAngle = inputs.shoulderAngle;
        double targetAngle = shoulderTargetAngle.getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Shoulder.ANGLE_TOLERANCE.getDegrees();
    }

    @AutoLogOutput
    public boolean isElbowAtTarget(Rotation2d elbowTargetAngle) {
        double currentAngle = inputs.elbowAngle;
        double targetAngle = elbowTargetAngle.getDegrees();
        return Math.abs(currentAngle - targetAngle) < Settings.DoubleJointedArm.Elbow.ANGLE_TOLERANCE.getDegrees();
    }

    @AutoLogOutput
    public boolean isArmAtTarget(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle) {
        return isShoulderAtTarget(shoulderTargetAngle) && isElbowAtTarget(elbowTargetAngle);
    }

    /* LOGIC for controlling the double jointed arm: */

    // 1. Find M, C, G => calculate FF given gear ratio

    // 2. Given a current position (x, y) for the end effector on the double jointed arm, pathfind to a target position (x, y) 
    //    and calculate the required angular velocity and acceleration; clamp to motion profile limits

    // 3. Use motion profile setpoints calculated in the last step + calculate PID and add to FF => yipee!

    /* CONTROL LOGIC, STATES & MATH */

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
     * @param position The angles of the joints, (θ1, θ2). Note that θ2 is relative to the first joint, not the horizontal.
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

    public Matrix<N2, N2> calcM(Matrix<N2, N1> position) { // only second joint position
        M.set(
            0,
            0,
            m1 * Math.pow(r1, 2.0)
                + m2 * (Math.pow(l1, 2.0) + Math.pow(r2, 2.0))
                + I1 + I2
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
        );

        M.set(
            0,
            1,
            m2 * Math.pow(r2, 2.0) + I2 + m2*l1*r2*Math.cos(position.get(1,0))
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
