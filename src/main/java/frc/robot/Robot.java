// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.Position;
import frc.robot.auto.AutoSelector;
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

    // Telemetry
    private final Telemetry telemetry_ = new Telemetry(robot_state_, arm_);

    // Autonomous
    AutoSelector auto_selector_ = new AutoSelector();


    @Override
    public void robotInit() {
        drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

        setupTeleopControls();
        // superstructure_.setPosition(Position.STOW);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robot_state_.update();
        telemetry_.periodic();
    }

    public Command test() {
        return new SequentialCommandGroup(
            new InstantCommand(()-> superstructure_.setPosition(Position.STOW)),
            new WaitCommand(5.0),
            superstructure_.setPosition(Position.CUBE_L1)
        );
    }

    @Override
    public void autonomousInit() {
        // superstructure_.setPosition(Position.CUBE_L1);
        System.out.println("AUTO ENABLED");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        test().schedule();
    }

    @Override
    public void teleopPeriodic() {

        System.out.println(superstructure_.getState());
        // System.out.println(arm_.getAngle());
        SmartDashboard.putNumber("Arm Angle", arm_.getAngle());
        
    }

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
