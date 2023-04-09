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
        fieldOriented_ = fieldOriented;
        
        addRequirements(drive_);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Joystick input
        double xSpeed = controller_.getLeftX();
        double ySpeed = controller_.getLeftY();
        double turnSpeed = controller_.getRightX();

        // Might add deadzone/deadband later

        // Converts joystick input to desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = (fieldOriented_) ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, drive_.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        
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
