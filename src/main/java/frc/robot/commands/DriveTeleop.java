package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class DriveTeleop extends CommandBase{
    private final Drive drive_;
    private final CommandXboxController controller_;
    private final boolean fieldOriented_;
    // Might add slew rate limiter later

    // Constructor
    public DriveTeleop(Drive drive, CommandXboxController controller, Boolean fieldOriented){
        drive_ = drive;
        controller_ = controller;
        fieldOriented_ = true;

        addRequirements(drive_);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Joystick input
        double xSpeed = controller_.getLeftX() * 0.3;
        double ySpeed = controller_.getLeftY() * 0.3;
        double turnSpeed = controller_.getRightX() * 0.5;

        // Might add deadzone/deadband later

        // Converts joystick input to desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = (fieldOriented_) ? ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, -turnSpeed, drive_.getRotation2d()) :
                new ChassisSpeeds(-ySpeed, -xSpeed, -turnSpeed);

        // Converts the desired chassis speeds to module states
        SwerveModuleState[] moduleStates = drive_.getKinematics().toSwerveModuleStates(chassisSpeeds);

        // Outputs module states to the swerve modules
        drive_.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        drive_.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
