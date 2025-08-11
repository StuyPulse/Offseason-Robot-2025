/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

    private static Alliance alliance;

    private RobotContainer robot;
    private Command auto;

    public Robot() {
        Logger.recordMetadata("OFFSEASON_ROBOT_25", "ROBOT_PROJECT"); // Set a metadata value

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            break;

            case SIM:
            // Running a physics simulator, log to NT
            Logger.addDataReceiver(new NT4Publisher());
            break;

            case REPLAY:
            // // Replaying a log, set up replay source
            // setUseTiming(false); // Run as fast as possible
            // String logPath = LogFileUtil.findReplayLog();
            // Logger.setReplaySource(new WPILOGReader(logPath));
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        robot = new RobotContainer();
        alliance = Alliance.Blue;
    }

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }
    
    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
