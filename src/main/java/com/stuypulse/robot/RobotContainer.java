/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.ArmL4;
import com.stuypulse.robot.commands.arm.ArmL4Out;
import com.stuypulse.robot.commands.arm.ArmTest45;
import com.stuypulse.robot.commands.arm.ArmTest4545;
import com.stuypulse.robot.commands.arm.ArmTestUp;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MoveArm;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    private final Arm dja = Arm.getInstance();

	// Autons
	private static SendableChooser<Command> autonChooser = new SendableChooser<>();

	public RobotContainer() {
		configureButtonBindings();
	}

	/****************/
	/*** DEFAULTS ***/
	/****************/

	private void configureDefaultCommands() {}

	/***************/
	/*** BUTTONS ***/
	/***************/

	private void configureButtonBindings() {
		driver.getTopButton().onTrue(new ArmL4());
		driver.getBottomButton().onTrue(new ArmL4Out());
        driver.getDPadRight().onTrue(new ArmTest45());
		driver.getDPadUp().onTrue(new ArmTestUp());
		driver.getDPadDown().onTrue(new ArmTest4545());
    }

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
