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
import frc.robot.subsystems.Intake;

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
    private final Intake intake_ = new Intake();

    // Robot State
    private final RobotState robot_state_ = new RobotState(drive_);

    // Superstructure
    private final Superstructure superstructure_ = new Superstructure(arm_, intake_);

    // Autonomous
    private final AutoSelector auto_selector_ = new AutoSelector();

    // Xbox Controller
    private final CommandXboxController driver_controller_ = new CommandXboxController(0);
    private final CommandXboxController operator_controller_ = new CommandXboxController(1);

    // Telemetry
    private final Telemetry telemetry_ = new Telemetry(robot_state_, arm_, auto_selector_);

 


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
        SmartDashboard.putString("Intake State", intake_.getState());
    }

    public Command test() {
        return new SequentialCommandGroup(
            superstructure_.setPosition(Position.STOW),
            new WaitCommand(3.0),
            superstructure_.setPosition(Position.CUBE_L1),
            new WaitCommand(3.0),
            superstructure_.setPosition(Position.CUBE_L2),
            new WaitCommand(3.0),
            superstructure_.setPosition(Position.INTAKE),
            new WaitCommand(3.0),
            superstructure_.setPosition(Position.STOW)
        );
    }

    @Override
    public void autonomousInit() {
        // superstructure_.setPosition(Position.CUBE_L1);
        System.out.println("AUTO ENABLED");

        
        // auto_selector_.run(robot_state_, superstructure_, arm_, drive_).schedule();
        
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        test().schedule();
    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putString("State", superstructure_.getState());
        SmartDashboard.putNumber("Arm Setpoint", arm_.getPositionSetpoint());
        // System.out.println(arm_.getAngle());
        SmartDashboard.putNumber("Arm Angle", Math.toDegrees(arm_.getAngle()));
        
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
        //  * Left Trigger:  Intake
        driver_controller_.rightTrigger(0.4).onTrue(
            new InstantCommand(() -> intake_.setPercent(0.4)));
        driver_controller_.rightTrigger(0.4).onFalse(
            new InstantCommand(() -> intake_.setPercent(0.0)));
        //  * Right Trigger: Outtake
        driver_controller_.leftTrigger(0.4).onTrue(
            new InstantCommand(() -> intake_.setPercent(-0.4)));
        driver_controller_.leftTrigger(0.4).onFalse(
            new InstantCommand(() -> intake_.setPercent(0.0)));

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
