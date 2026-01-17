/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.krakenintake.KrakenIntakeIntake;
import com.stuypulse.robot.commands.krakenintake.KrakenIntakeOuttake;
import com.stuypulse.robot.commands.krakenintake.KrakenIntakeStop;
import com.stuypulse.robot.commands.neointake.NeoIntakeIntake;
import com.stuypulse.robot.commands.neointake.NeoIntakeOuttake;
import com.stuypulse.robot.commands.neointake.NeoIntakeStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake;
import com.stuypulse.robot.subsystems.neointake.NeoIntake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
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
	private final NeoIntake neoIntake = NeoIntake.getInstance();
	private final KrakenIntake krakenIntake = KrakenIntake.getInstance();
	private final SwerveDrive swerve = SwerveDrive.getInstance();

	// Autons
	private static SendableChooser<Command> autonChooser = new SendableChooser<>();

	public RobotContainer() {
		configureButtonBindings();
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

		// Static Intake
		driver.getLeftTriggerButton()
			.onTrue(new NeoIntakeIntake())
			.onFalse(new NeoIntakeStop());

		driver.getRightTriggerButton()
			.onTrue(new NeoIntakeOuttake())
			.onFalse(new NeoIntakeStop());

		// Spring Loaded Intake
		// driver.getLeftBumper()
		// 	.onTrue(new KrakenIntakeIntake())
		// 	.onFalse(new KrakenIntakeStop());

		// driver.getRightBumper()
		// 	.onTrue(new KrakenIntakeOuttake())
		// 	.onFalse(new KrakenIntakeStop());
			
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
