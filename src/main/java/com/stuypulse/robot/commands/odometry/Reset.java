package com.stuypulse.robot.commands.odometry;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Reset extends InstantCommand {
    public Reset(Rotation2d heading) {
        super(() -> Odometry.getInstance().reset(new Pose2d(Odometry.getInstance().getPose().getTranslation(), heading)));
    }

    public Reset(){
        this(new Rotation2d());
    }
}