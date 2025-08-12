package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants.Swerve.BackLeft;
import com.stuypulse.robot.constants.Constants.Swerve.BackRight;
import com.stuypulse.robot.constants.Constants.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Constants.Swerve.FrontRight;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    
    private static final SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            new SwerveModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER),
            new SwerveModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER),
            new SwerveModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER),
            new SwerveModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER)
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics kinematics;
    private final FieldObject2d[] module2ds;

    protected SwerveDrive(SwerveModule... modules) {
        this.modules = modules;

        gyro = new Pigeon2(9, Settings.CANIVORE);

        kinematics = new SwerveDriveKinematics(getModuleOffsets());

        module2ds = new FieldObject2d[modules.length];
    }

    
    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getName()+"-2d");
            module2ds[i].setPose(Robot.isBlue() ? module2ds[i].getPose() : Field.transformToOppositeAlliance(module2ds[i].getPose()));
        }
    }

    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];

        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getModuleOffset();
        }

        return locations;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModule getModule(String id) {
        for (SwerveModule module : modules)
            if (module.getName().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    /** MODULE STATES API **/
    public void drive(Vector2D velocity, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.x,
                velocity.y,
                omega,
                gyro.getRotation2d());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        Vector2D xy = new Vector2D(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        xy = xy.clamp(Settings.Swerve.MAX_VELOCITY.get());
        robotSpeed.vxMetersPerSecond = xy.x;
        robotSpeed.vyMetersPerSecond = xy.y;

        robotSpeed.omegaRadiansPerSecond = SLMath.clamp(
            robotSpeed.omegaRadiansPerSecond, 
            -Settings.Swerve.MAX_ANGULAR_VELOCITY.get(),
            Settings.Swerve.MAX_ANGULAR_VELOCITY.get()
        );
        
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    private static SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND)
            return state;

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(filterModuleState(states[i]));
        }
    }

    /** PATH FOLLOWING **/
    public Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public double getGyroYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getGyroPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getGyroRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    /** KINEMATICS **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        SwerveModuleState[] state = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
        setModuleStates(state);
    }

    public void configureAutoBuilder() {        
        try{
            Odometry odometry = Odometry.getInstance();

            AutoBuilder.configure(
                odometry::getPose,
                odometry::reset,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> setChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                    Gains.Swerve.Alignment.XY,
                    Gains.Swerve.Alignment.THETA
                ),
                RobotConfig.fromGUISettings(),
                () -> false,
                instance
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for (int i = 0; i < modules.length; ++i) {
            Pose2d modulePose = new Pose2d(
                pose.getTranslation().plus(modules[i].getModuleOffset().rotateBy(angle)),
                modules[i].getState().angle.plus(angle)
            );
            module2ds[i].setPose(Robot.isBlue() ? modulePose : Field.transformToOppositeAlliance(modulePose));
        }

        SmartDashboard.putNumber("Swerve/Gyro Angle (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Pitch (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Roll (deg)", getGyroRoll());

        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getAccelerationX().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getAccelerationY().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getAccelerationZ().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());
        gyro.setYaw(gyro.getYaw().getValueAsDouble() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }

}