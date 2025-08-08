package com.stuypulse.robot.subsystems.double_jointed_arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.stuypulse.robot.constants.Constants;

/* 6328's Arm Visualizer code */
/** Helper class for creating a {@link Mechanism2d} and 3D component representation of an arm. */
public class DoubleJointedArmVisualizer {
  // Physical Constants
  private final double l1 = Constants.DoubleJointedArm.Shoulder.LENGTH;  
  private final double l2 = Constants.DoubleJointedArm.Elbow.LENGTH; 

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d mechanismRoot;
  private final LoggedMechanismLigament2d fixedLigament;
  private final LoggedMechanismLigament2d shoulderLigament;
  private final LoggedMechanismLigament2d elbowLigament;

  private String logKey;

  public DoubleJointedArmVisualizer(String logKey, Color8Bit colorOverride) {
    this.logKey = logKey;
    mechanism = new LoggedMechanism2d(8, 8, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 4, 3);
    
    // Fixed base ligament
    fixedLigament = mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Fixed", 
            0.2, 
            90, 
            6, 
            new Color8Bit(Color.kBlack)));
    
    // Shoulder ligament (first joint)
    shoulderLigament = mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Shoulder",
            l1,
            0, // Will be updated in update() method
            4,
            colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
    
    // Elbow ligament (second joint, attached to end of shoulder)
    elbowLigament = shoulderLigament.append(
        new LoggedMechanismLigament2d(
            "Elbow",
            l2,
            0, // Will be updated in update() method  
            4,
            colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
  }

  public void update(double shoulderAngle, double elbowAngle) {
    double shoulderAngleDegrees = Units.radiansToDegrees(shoulderAngle);
    shoulderLigament.setAngle(shoulderAngleDegrees);
    
    double elbowAngleDegrees = Units.radiansToDegrees(elbowAngle); // relative to shoulder
    elbowLigament.setAngle(elbowAngleDegrees);
    
    Logger.recordOutput("DoubleJointedArm/VisualizerMechanism2d/" + logKey, mechanism);

    // Base pose at origin
    var basePose = new Pose3d(0.0, 0.0, Constants.DoubleJointedArm.BASE_HEIGHT, new Rotation3d());
    
    var shoulderPose = basePose.transformBy(
        new Transform3d(
            new Translation3d(0.0, 0.0, 0.0),
            new Rotation3d(0.0, -shoulderAngle, 0.0))); // rotate about pitch
    
    var elbowPose = shoulderPose.transformBy(
        new Transform3d(
            new Translation3d(l1, 0.0, 0.0),
            new Rotation3d(0.0, -elbowAngle, 0.0)));
    
    var endEffectorPose = elbowPose.transformBy(
        new Transform3d(
            new Translation3d(l2, 0.0, 0.0),
            new Rotation3d()));
    
    Logger.recordOutput("DoubleJointedArm/Visualizer/Mechanism3d/" + logKey, shoulderPose, elbowPose, endEffectorPose);
    
    Logger.recordOutput("DoubleJointedArm/Visualizer/Mechanism/" + logKey + "/ShoulderAngleDeg", shoulderAngleDegrees);
    Logger.recordOutput("DoubleJointedArm/Visualizer/Mechanism/" + logKey + "/ElbowAngleDeg", elbowAngleDegrees);
  }
}