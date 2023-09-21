// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.Arm;
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
    private final Arm arm_ = new Arm();

    // Robot State
    private final RobotState robot_state_ = new RobotState(drive_);

    // Superstructure
    private final Superstructure superstructure_ = new Superstructure(arm_);

    // Xbox Controller
    private final CommandXboxController driver_controller_ = new CommandXboxController(0);
    private final CommandXboxController operator_controller_ = new CommandXboxController(1);
    

    @Override
    public void robotInit() {
        drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

        setupTeleopControls();
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

    // Teleop Controls
    public void setupTeleopControls() {
        // Add Intake wheel code

        //  * X:       Stow
        operator_controller_.x().onTrue(superstructure_.setPosition(Superstructure.Position.STOW));
        //  * A:       Intake
        operator_controller_.a().onTrue(superstructure_.setPosition(Superstructure.Position.INTAKE));
        //  * B:       L1 Cube
        operator_controller_.b().onTrue(superstructure_.setPosition(Superstructure.Position.CUBE_L1));
        //  * Y:       L2 Cube
        operator_controller_.y().onTrue(superstructure_.setPosition(Superstructure.Position.CUBE_L2));
    }
}
