package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drive;

public class DriveForwardRotate extends SequentialCommandGroup{
    // Subsystems
    Drive drive_ = new Drive();

    // Poses
    private static final Pose2d kStartPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private static final Pose2d kEndPose = new Pose2d(2, 0, Rotation2d.fromDegrees(180));

    // Probably need to create a new separate class like DriveTrajectory in 2023 code to just create the trajectories
    // and then just call the command in the auto selector

    // Constructor
    public DriveForwardRotate(Drive drive) {
        drive_ = drive;
        addCommands(Move());
    }

    // Im sorry i ran out of names for this command
    public Command Move() {
        // Get Poses
        Pose2d start_pos_ = kStartPose;
        Pose2d end_pos_ = kEndPose;

        // Generate Trajectory
        Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos_, new ArrayList<>(), end_pos_, AutoConfig.kConfig);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            t1, 
            drive_ :: getPose, 
            drive_.getKinematics(), 
            AutoConfig.getController(), 
            drive_::setModuleStates, 
            drive_);

        // Add Commands
        return new SequentialCommandGroup(
            new InstantCommand(() -> drive_.resetOdometry(t1.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> drive_.stopModules())
        );
    }
}
