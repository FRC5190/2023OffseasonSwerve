package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class TurnToAngle extends CommandBase{
    // Subsystems
    private final Drive drive_;
    private final RobotState robot_state_;

    // Target Angle
    private final double target_angle_;

    // Chassis Speeds
    public ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0.5);

    // Constructor
    public TurnToAngle(double target_angle, Drive drive, RobotState robot_state) {
        // Assign member variables
        drive_ = drive;
        robot_state_ = robot_state;
        target_angle_ = target_angle;
    }

    @Override
    public void execute() {
        drive_.setSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(robot_state_.getDegree() - target_angle_) < Constants.tolerance);
    }

    @Override
    public void end(boolean interrupted) {
        drive_.setSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    public static class Constants {
        public static final double tolerance = 5.0;
    }
}
