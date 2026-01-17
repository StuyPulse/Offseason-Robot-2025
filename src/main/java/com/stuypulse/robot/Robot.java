/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.stuypulse.robot.commands.neointake.NeoIntakeStop;

public class Robot extends TimedRobot {

    private static Alliance alliance;

    private RobotContainer robot;
    private Command auto;

    public Robot() {
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
        new NeoIntakeStop();
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

        Shuffleboard.selectTab("Autonomous");
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
        new NeoIntakeStop();
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
