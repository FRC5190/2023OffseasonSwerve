// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.Drive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Subsystems
    private final Drive drive_ = new Drive();

    // Robot State
    private final RobotState robot_state_ = new RobotState(drive_);

    // Xbox Controller
    private final CommandXboxController controller_ = new CommandXboxController(0);

    @Override
    public void robotInit() {
        drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, controller_));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robot_state_.update();
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    // Might need this in the future
    // robot_state_.reset(robot_state_.getPosition());
    // robot_state_.update();
    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
