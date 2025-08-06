/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.arm.DoubleJointedArm;
import com.stuypulse.robot.subsystems.arm.DoubleJointedArmIO;
import com.stuypulse.robot.subsystems.arm.DoubleJointedArmIOReal;
import com.stuypulse.robot.subsystems.arm.DoubleJointedArmIOSim;
import com.stuypulse.robot.subsystems.arm.DoubleJointedArmVisualizer;
import com.stuypulse.robot.subsystems.wrist.Wrist;
import com.stuypulse.robot.subsystems.wrist.WristIOReal;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    private DoubleJointedArm dja;
    private Wrist wrist;

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
              // Real robot, instantiate hardware IO implementations
              dja = new DoubleJointedArm(new DoubleJointedArmIOReal());
              wrist = new Wrist(new WristIOReal());
              break;
      
            case SIM:
              // Sim robot, instantiate physics sim IO implementations
              dja = new DoubleJointedArm(new DoubleJointedArmIOSim());
              break;
      
            default:
              // Replayed robot, disable IO implementations
              dja = new DoubleJointedArm(new DoubleJointedArmIO() {});
            //   drive = new Drive(new DriveIO() {}, new GyroIO() {});
            //   roller = new Roller(new RollerIO() {});
              break;
        }
        
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {}

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
