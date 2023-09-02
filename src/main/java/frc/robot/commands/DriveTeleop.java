package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTeleop extends CommandBase {
    // Subsystems
    private final Drive drive_;
    private final RobotState robot_state_;

    // Xbox Controller
    private final CommandXboxController controller_;

    // Constructor
    public DriveTeleop(Drive drive, RobotState robot_state, CommandXboxController controller) {
        // Assign member variables
        drive_ = drive;
        robot_state_ = robot_state;
        controller_ = controller;

        // Add subsystem requirements
        addRequirements(drive_);
    }

    @Override
    public void execute() {
        // Inputs input
        double xSpeed = -controller_.getLeftY() * Constants.kTranslationMultiplier;
        double ySpeed = -controller_.getLeftX() * Constants.kTranslationMultiplier;
        double rSpeed = -controller_.getRightX() * Constants.kRotationMultiplier;
        boolean robot_oriented = controller_.rightTrigger(0.2).getAsBoolean();
        boolean hold_position_mode = controller_.x().getAsBoolean();

        // Create chassis speeds
        ChassisSpeeds speeds = robot_oriented ?
            new ChassisSpeeds(xSpeed, ySpeed, rSpeed) :
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed,
                robot_state_.getPosition().getRotation());

        // Set speeds
        if (hold_position_mode) {
            drive_.HoldPosition();
        }
        else{
            drive_.setSpeeds(speeds, Drive.OutputType.OPEN_LOOP);
        }
    }

    // Constants
    private static class Constants {
        // Joystick Multiplier (percent -> speed)
        public static final double kTranslationMultiplier = 2.5;
        public static final double kRotationMultiplier = Math.PI;
    }
}
