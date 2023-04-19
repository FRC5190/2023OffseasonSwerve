package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    // Slew rate limiter
    private SlewRateLimiter limiter_;

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
        boolean precise_mode = controller_.leftTrigger(0.2).getAsBoolean();
        double sensitivity = (precise_mode) ? Constants.kTranslationMultiplierPrecise : 
            Constants.kTranslationMultiplierRegular;
        double xSpeed = (Math.abs(controller_.getLeftY()) > Constants.kDeadzone) ? -controller_.getLeftY() * sensitivity : 0;
        double ySpeed = (Math.abs(controller_.getLeftX()) > Constants.kDeadzone) ? -controller_.getLeftX() * sensitivity : 0;
        double rSpeed = (Math.abs(controller_.getRightX()) > Constants.kDeadzone) ? 
            -controller_.getRightX() * Constants.kRotationMultiplier : 0;
        boolean robot_oriented = controller_.rightTrigger(0.2).getAsBoolean();
        boolean hold_position_mode = controller_.x().getAsBoolean();

        // Slew rate limiter
        limiter_.calculate(Constants.kSlewRateLimiter);
        xSpeed = limiter_.calculate(xSpeed);
        ySpeed = limiter_.calculate(ySpeed);

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
        public static final double kTranslationMultiplierRegular = 2.5;
        public static final double kTranslationMultiplierPrecise = 1.3;
        public static final double kRotationMultiplier = Math.PI;

        // Slew rate limiter
        public static final double kSlewRateLimiter = 0.5;
        // Deadzone
        public static final double kDeadzone = 0.05;
    }
}
