package com.stuypulse.robot.subsystems.double_jointed_arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonGetter;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleJointedArm extends SubsystemBase {

    private static DoubleJointedArm instance;

    private DoubleJointedArmIO io;
    private DoubleJointedArmIOInputsAutoLogged inputs = new DoubleJointedArmIOInputsAutoLogged();

    public enum DoubleJointedArmState {
        STOW(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10));

        private Rotation2d shoulderTargetAngle;
        private Rotation2d elbowTargetAngle;

        private DoubleJointedArmState(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle){
            this.shoulderTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(shoulderTargetAngle.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MIN_ANGLE.getDegrees(), 
                Constants.DoubleJointedArm.Shoulder.MAX_ANGLE.getDegrees()));
            this.elbowTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(elbowTargetAngle.getDegrees(), 
                Constants.DoubleJointedArm.Elbow.MIN_ANGLE.getDegrees(), 
                Constants.DoubleJointedArm.Elbow.MAX_ANGLE.getDegrees())); 
        } 

        public Rotation2d getShoulderTargetAngle(){
            return this.shoulderTargetAngle;
        }

        public Rotation2d getElbowTargetAngle(){
            return this.elbowTargetAngle;
        }

        private DoubleJointedArmState state;

        public DoubleJointedArmState getState(){
            return this.state;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DoubleJointedArm", inputs);
        // PUT ALL CALLS TO CONTROL METHODS HERE
    }

    /* LOGIC for controlling the double jointed arm: */

    // 0. Measure system: angle, angular velocity, angular acceleration

    // 1. Find M, C, G (mass inertia matrix, centrifugal + coriolis matrix, gravity torque matrix)

    // 2. Given a current position (x, y) for the end effector on the double jointed arm, pathfind to a target position (x, y) and calculate the required angular position, velocity, and acceleration

    // 3. Feed the current angular states and target angular states into the PID controllers for each joint

    // 4. Calculate the three matrices using the angular states, which outputs a toruqe input

    // 5. Using torque current control on the motor, feed the torque input into the motor
}
