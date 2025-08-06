package com.stuypulse.robot.subsystems.arm;

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

    private final boolean[][] obstaclePoint; 
    private final double gridResolution = 2.0 * Math.PI; // 2pi*2pi grid
    
    // For visualization (Sim)
    private Translation2d currentEndEffectorPos;
    private Translation2d targetEndEffectorPos;
    private List<Translation2d> pathPoints = new ArrayList<>();

    public ArmConfigurationSpace() {
        this.obstaclePoint = new boolean[(int)gridResolution][(int)gridResolution];
    }

    // Convert joint angles to Cartesian space
    public Translation2d toCartesian(Rotation2d theta1, Rotation2d theta2) {
        double x = shoulderLength * Math.cos(theta1.getRadians()) + elbowLength * Math.cos(theta1.getRadians() + theta2.getRadians());
        double y = shoulderLength * Math.sin(theta1.getRadians()) + elbowLength * Math.sin(theta1.getRadians() + theta2.getRadians());
        return new Translation2d(x, y);
    }

    // Convert Cartesian to joint angles
    // Generate two possible configurations?!?!? 
    public Rotation2d[] toJointAngles(Translation2d point) {
        double d = Math.sqrt(Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2));
        if (d > shoulderLength + elbowLength || d < Math.abs(shoulderLength - elbowLength)) {
            return null; 
        }
        
        Rotation2d theta2 = new Rotation2d(Math.acos((Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2) - shoulderLength*shoulderLength - elbowLength*elbowLength)
                                / (2 * shoulderLength * elbowLength)));
        Rotation2d theta1 = new Rotation2d(Math.atan2(point.getY(), point.getX()) - Math.atan2(elbowLength * Math.sin(theta2.getRadians()), 
                                                    shoulderLength + elbowLength * Math.cos(theta2.getRadians())));
        
        return new Rotation2d[]{theta1, theta2};
    }

    // Add obstacle in Cartesian space
    public void addObstacle(Translation2d point) {
        Rotation2d[] angles = toJointAngles(point);

        if (angles != null) {
            int theta1Idx = (int)((angles[0].getRadians() - shoulderMinAngle.getRadians()) / (shoulderMaxAngle.getRadians() - shoulderMinAngle.getRadians()) * gridResolution);
            int theta2Idx = (int)((angles[1].getRadians() - elbowMinAngle.getRadians()) / (elbowMaxAngle.getRadians() - elbowMinAngle.getRadians()) * gridResolution);
            
            if (theta1Idx >= 0 && theta1Idx < gridResolution && 
                theta2Idx >= 0 && theta2Idx < gridResolution) {
                obstaclePoint[theta1Idx][theta2Idx] = true;
            }
        }
    }

    // Check if configuration is valid (I think SLMath.clamp() got this? Double check)
    public boolean isValidConfiguration(Rotation2d theta1, Rotation2d theta2) {
    
        if (theta1.getRadians() < shoulderMinAngle.getRadians() || theta1.getRadians() > shoulderMaxAngle.getRadians() || 
            theta2.getRadians() < elbowMinAngle.getRadians() || theta2.getRadians() > elbowMaxAngle.getRadians()) {
            return false;
        }
        
        // Check obstacle grid
        int theta1Idx = (int)((theta1.getRadians() - shoulderMinAngle.getRadians()) / (shoulderMaxAngle.getRadians() - shoulderMinAngle.getRadians()) * gridResolution);
        int theta2Idx = (int)((theta2.getRadians() - elbowMinAngle.getRadians()) / (elbowMaxAngle.getRadians() - elbowMinAngle.getRadians()) * gridResolution);
        
        if (theta1Idx >= 0 && theta1Idx < gridResolution && 
            theta2Idx >= 0 && theta2Idx < gridResolution) {
            return !obstaclePoint[theta1Idx][theta2Idx];
        }
        return false;
    }
}