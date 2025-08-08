package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmTrajectoryPoint extends Translation2d {
    public double theta1;    // Shoulder angle (rad)
    public double theta2;    // Elbow angle (rad)
    public double omega1;    // Shoulder velocity (rad/s)
    public double omega2;    // Elbow velocity (rad/s)
    public double alpha1;    // Shoulder acceleration (rad/s^2)
    public double alpha2;    // Elbow acceleration (rad/s^2)
    public double time;      // Time since start (s)

    public ArmTrajectoryPoint(double theta1, double theta2,
                            double omega1, double omega2,
                            double alpha1, double alpha2,
                            double time) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.omega1 = omega1;
        this.omega2 = omega2;
        this.alpha1 = alpha1;
        this.alpha2 = alpha2;
        this.time = time;
    }

    public ArmTrajectoryPoint() {}
}