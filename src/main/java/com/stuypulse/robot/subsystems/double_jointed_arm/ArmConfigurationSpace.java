package com.stuypulse.robot.subsystems.double_jointed_arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;

import java.util.*;

import com.stuypulse.robot.constants.Constants;

public class ArmConfigurationSpace {

    private final Rotation2d shoulderMinAngle = Constants.DoubleJointedArm.Shoulder.MIN_ANGLE;
    private final Rotation2d shoulderMaxAngle = Constants.DoubleJointedArm.Shoulder.MAX_ANGLE;
    private final Rotation2d elbowMinAngle = Constants.DoubleJointedArm.Elbow.MIN_ANGLE;
    private final Rotation2d elbowMaxAngle = Constants.DoubleJointedArm.Elbow.MAX_ANGLE;

    private final double shoulderLength = Constants.DoubleJointedArm.Shoulder.LENGTH;
    private final double elbowLength = Constants.DoubleJointedArm.Elbow.LENGTH;

    private final boolean[][] obstaclePoint; // Cartesian (Should get rid of?)
    private final List<Translation2d> obstaclePoints; // Radians
    private final double cartesianGrid = 100;
    
    // For visualization (Sim)
    private Translation2d currentEndEffectorPos;
    private Translation2d targetEndEffectorPos;
    private List<Translation2d> pathPoints = new ArrayList<>();

    public ArmConfigurationSpace() {
        this.obstaclePoint = new boolean[(int)cartesianGrid][(int)cartesianGrid];
        this.obstaclePoints = new ArrayList<>();
    }

    // Convert joint angles to Cartesian space
    public Translation2d toCartesian(Rotation2d theta1, Rotation2d theta2) {
        double x = shoulderLength * Math.cos(theta1.getRadians()) + elbowLength * Math.cos(theta1.getRadians() + theta2.getRadians());
        double y = shoulderLength * Math.sin(theta1.getRadians()) + elbowLength * Math.sin(theta1.getRadians() + theta2.getRadians());
        return new Translation2d(x, y);
    }

    // Convert Cartesian point to joint angles
    public Pair<Pair<Rotation2d, Rotation2d>, Pair<Rotation2d, Rotation2d>> toJointAngles(Translation2d point) {
        final double EPSILON = 1e-6; // Error
        double x = point.getX();
        double y = point.getY();
        double dSquared = x * x + y * y;
        double d = Math.sqrt(dSquared);
    
        // Check if point is reachable
        if (d > shoulderLength + elbowLength + EPSILON || 
            d + EPSILON < Math.abs(shoulderLength - elbowLength)) {
            return null; 
        }
    
        // Case 1: Fully stretched (only elbow-up solution exists)
        if (Math.abs(d - (shoulderLength + elbowLength)) < EPSILON) {
            double theta1 = Math.atan2(y, x);
            double theta2 = 0.0; // Elbow completely straight
            Pair<Rotation2d, Rotation2d> singleSolution = new Pair<>(
                new Rotation2d(theta1),
                new Rotation2d(theta2)
            );
            return new Pair<>(singleSolution, null);
        }
    
        // Case 2: Fully folded (only elbow-down solution exists)
        if (Math.abs(d - Math.abs(shoulderLength - elbowLength)) < EPSILON) {
            double theta1 = Math.atan2(y, x);
            double theta2 = Math.PI; // Elbow length less than shoulder length
            Pair<Rotation2d, Rotation2d> singleSolution = new Pair<>(
                new Rotation2d(theta1),
                new Rotation2d(theta2)
            );
            return new Pair<>(singleSolution, null);
        }
    
        // Normal case: Two solutions exist
        double cosTheta2 = (dSquared - shoulderLength*shoulderLength - elbowLength*elbowLength) 
                          / (2 * shoulderLength * elbowLength);
        double theta2ElbowUp = Math.acos(cosTheta2);
        double theta2ElbowDown = -theta2ElbowUp;
    
        double theta1ElbowUp = Math.atan2(y, x) - Math.atan2(
            elbowLength * Math.sin(theta2ElbowUp),
            shoulderLength + elbowLength * Math.cos(theta2ElbowUp)
        );
    
        double theta1ElbowDown = Math.atan2(y, x) - Math.atan2(
            elbowLength * Math.sin(theta2ElbowDown),
            shoulderLength + elbowLength * Math.cos(theta2ElbowDown)
        );
    
        Pair<Rotation2d, Rotation2d> elbowUpConfig = new Pair<>(
            new Rotation2d(theta1ElbowUp),
            new Rotation2d(theta2ElbowUp)
        );
    
        Pair<Rotation2d, Rotation2d> elbowDownConfig = new Pair<>(
            new Rotation2d(theta1ElbowDown),
            new Rotation2d(theta2ElbowDown)
        );
    
        return new Pair<>(elbowUpConfig, elbowDownConfig);
    }

    // Add obstacle in Cartesian space
    public void addObstacle(Translation2d point) {
        Pair<Translation2d, Translation2d> angles = new Pair<>(new Translation2d(toJointAngles(point).getFirst().getFirst().getRadians(), 
                                                                                toJointAngles(point).getFirst().getSecond().getRadians()), 
                                                               new Translation2d(toJointAngles(point).getSecond().getFirst().getRadians(), 
                                                                                toJointAngles(point).getSecond().getSecond().getRadians()));

        if (angles != null) {

            obstaclePoint[(int)point.getX()][(int)point.getY()] = true;
            obstaclePoints.add(angles.getFirst());
            if (angles.getSecond() != null) {
                obstaclePoints.add(angles.getSecond());
            }
        }
    }

    // Check if configuration is valid 
    public boolean isValidConfiguration(Rotation2d theta1, Rotation2d theta2) {
    
        if (theta1.getRadians() < shoulderMinAngle.getRadians() || theta1.getRadians() > shoulderMaxAngle.getRadians() || 
            theta2.getRadians() < elbowMinAngle.getRadians() || theta2.getRadians() > elbowMaxAngle.getRadians()) {
            return false;
        }
        
        // Check obstacle grid
        Translation2d point = new Translation2d(theta1.getRadians(), theta2.getRadians());

        for (int i = 0; i < obstaclePoints.size(); i++) {
            if (point.equals(obstaclePoints.get(i))) {
                return false;
            }
        }

        return false;

    }
}