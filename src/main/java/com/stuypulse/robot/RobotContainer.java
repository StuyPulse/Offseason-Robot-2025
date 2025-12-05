/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.WristSetState;
import com.stuypulse.robot.commands.arm.ArmL4;
import com.stuypulse.robot.commands.arm.ArmL4Out;
import com.stuypulse.robot.commands.arm.ArmStow;
import com.stuypulse.robot.commands.arm.ArmTest45;
import com.stuypulse.robot.commands.arm.ArmTest135;
import com.stuypulse.robot.commands.arm.ArmTestUp;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.wrist.Wrist;
import com.stuypulse.robot.subsystems.wrist.Wrist.WristState;
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
	// public final Arm arm;
	// public final Wrist wrist;
	public final SwerveDrive swerve;

	// Autons
	private static SendableChooser<Command> autonChooser = new SendableChooser<>();

	public RobotContainer() {
		// arm = Arm.getInstance();
		// wrist = Wrist.getInstance();
		swerve = SwerveDrive.getInstance();
		// configureButtonBindings();
		configureDefaultCommands();
	}

	/****************/
	/*** DEFAULTS ***/
	/****************/

	private void configureDefaultCommands() {
		swerve.setDefaultCommand(new SwerveDriveDrive(driver));
	}

	/***************/
	/*** BUTTONS ***/
	/***************/

	private void configureButtonBindings() {
		driver.getDPadLeft().onTrue(new ArmTest135());
		driver.getDPadRight().onTrue(new ArmTest45());
		driver.getDPadUp().onTrue(new ArmTestUp());
		driver.getDPadDown().onTrue(new ArmStow());
		
		driver.getTopButton().onTrue(new WristSetState(WristState.STOW));
		driver.getLeftButton().onTrue(new WristSetState(WristState.BACK));
		driver.getRightButton().onTrue(new WristSetState(WristState.FRONT));

		driver.getRightBumper().onTrue(new ArmL4());
		driver.getRightTriggerButton().onTrue(new ArmL4Out());
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
