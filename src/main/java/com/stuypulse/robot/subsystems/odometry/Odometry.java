package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Odometry extends SubsystemBase {

    private static final Odometry instance;

    static {
        instance = new OdometryImpl();
    }

    public static Odometry getInstance(){
        return instance;
    }

    public abstract Field2d getField();

    public abstract void reset(Pose2d pose2d);

    public abstract Pose2d getPose();

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void seedFieldRelative() {
        Pose2d newPose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
        reset(newPose);
    }

    public abstract void addVisionData(Pose2d robotPose, double timestamp, Vector<N3> stdevs);
}