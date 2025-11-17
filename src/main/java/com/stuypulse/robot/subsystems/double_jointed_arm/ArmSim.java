package com.stuypulse.robot.subsystems.double_jointed_arm;

import java.util.List;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {

    private Mechanism2d canvas;
    // Hardware
    private final DCMotor shoulderMotors;
    private final DCMotor elbowMotor;

    private final DCMotorSim shoulderSim;
    private final DCMotorSim elbowSim;

    

    // States
    private boolean intermediate;
    private ArmState storedState;

    // Matricies
    private Matrix<N2, N2> mMatrix;
    private Matrix<N2, N2> cMatrix;
    private Matrix<N2, N1> gMatrix;

    private Matrix<N2, N1> vMatrix;
    private Matrix<N2, N1> aMatrix;

    private Matrix<N2, N2> kBMatrix;
    private Matrix<N2, N2> BMatrix;
    private Matrix<N2, N1> uMatrix;
    private Matrix<N2, N1> targetVelocityMatrix;
    private Matrix<N2, N1> targetAccelMatrix;

    // Physical Constants
    private final double shoulderMass = Constants.DoubleJointedArm.Shoulder.MASS;       // kg
    private final double elbowMass = Constants.DoubleJointedArm.Elbow.MASS;             // kg
    private final double shoulderLength = Constants.DoubleJointedArm.Shoulder.LENGTH;   // m
    private final double elbowLength = Constants.DoubleJointedArm.Elbow.LENGTH;         // m
    private final double GRAVITY = 9.81;

    private final double kT = 7.09 / 366.0;  // Stall torque / Stall current
    private final double kU = 6000.0 / 12.0; // Free speed / Volts
    private final double r = 12.0 / 366.0;   // Volts / Stall current

    private final double shoulderWidth = Constants.DoubleJointedArm.Shoulder.WIDTH;
    private final double elbowWidth = Constants.DoubleJointedArm.Elbow.WIDTH;

    private final double I1 = shoulderMass*(Math.pow(shoulderWidth,2)+Math.pow(shoulderLength, 2))/12f;
    private final double I2 = elbowMass*(Math.pow(elbowWidth,2)+Math.pow(elbowLength, 2))/12f + Constants.Wrist.MASS * Math.pow(elbowLength/2, 2);

    // Conversion Factors
    private final double shoulderGearRatio = Constants.DoubleJointedArm.Shoulder.GEAR_RATIO;
    private final double elbowGearRatio = Constants.DoubleJointedArm.Elbow.GEAR_RATIO;

    //PID
    private final Controller elbowController;
    private final Controller shoulderController;

    public ArmSim() {
        
        shoulderMotors = DCMotor.getKrakenX60(2);
        elbowMotor = DCMotor.getKrakenX60(1);

        shoulderSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(shoulderMotors, I1, shoulderGearRatio), shoulderMotors);
        elbowSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(elbowMotor, I2, elbowGearRatio), elbowMotor);





        mMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        cMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        gMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        vMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        aMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        targetVelocityMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        targetAccelMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        kBMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        BMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        uMatrix = new Matrix<>(Nat.N2(), Nat.N1());

        intermediate = false;
        storedState = ArmState.STOW;
    }

    @Override
    public void periodic() {


    }
}