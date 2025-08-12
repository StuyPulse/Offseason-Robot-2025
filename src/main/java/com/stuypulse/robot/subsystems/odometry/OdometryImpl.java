package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OdometryImpl extends Odometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    private final FieldObject2d poseEstimatorPose2d;

    protected OdometryImpl() {
        var swerve = SwerveDrive.getInstance();
        var startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        poseEstimator =
            new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                startingPose,

                VecBuilder.fill(
                    0.1,
                    0.1,
                    0.1),
                    VecBuilder.fill(0.3, 0.3, 5)
                );

        field = new Field2d();

        poseEstimatorPose2d = field.getRobotObject();
        poseEstimatorPose2d.setPose(Robot.isBlue() ? new Pose2d() : Field.transformToOppositeAlliance(new Pose2d()));

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();

        poseEstimator.resetPosition(
            drive.getGyroAngle(),
            drive.getModulePositions(),
            pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    @Override
    public void addVisionData(Pose2d robotPose, double timestamp, Vector<N3> stdevs) {
        poseEstimator.setVisionMeasurementStdDevs(stdevs);
        poseEstimator.addVisionMeasurement(robotPose, timestamp);
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());

        poseEstimatorPose2d.setPose(Robot.isBlue() ? poseEstimator.getEstimatedPosition() : Field.transformToOppositeAlliance(poseEstimator.getEstimatedPosition()));

        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}