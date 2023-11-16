package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drive;

public class RobotState {
    // Swerve Drive
    private final Drive drive_;

    // Position Estimator
    private final SwerveDrivePoseEstimator pose_estimator_;

    // Constructor
    public RobotState(Drive drive) {
        // Assign member variables
        drive_ = drive;

        // Initialize pose estimator
        pose_estimator_ = new SwerveDrivePoseEstimator(
            drive_.getKinematics(), drive.getAngle(), drive.getSwerveModulePositions(),
            new Pose2d());
    }

    // Update
    public void update() {
        // Update pose estimator with new drive measurements
        pose_estimator_.update(drive_.getAngle(), drive_.getSwerveModulePositions());
    }

    // Get Position
    public Pose2d getPosition() {
        return pose_estimator_.getEstimatedPosition();
    }

    // Get Degree
    public double getDegree() {
        return pose_estimator_.getEstimatedPosition().getRotation().getDegrees();
    }

    // Get Rotation2d
    public Rotation2d getRotation2d() {
        return pose_estimator_.getEstimatedPosition().getRotation();
    }

    // Reset Position
    // Add this after testing
    public void reset(Pose2d pose) {
        pose_estimator_.resetPosition(drive_.getAngle(), drive_.getSwerveModulePositions(), pose);
    }
}
