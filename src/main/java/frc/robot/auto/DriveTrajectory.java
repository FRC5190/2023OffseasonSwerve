package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTrajectory {
    // Instance variables
    private final Drive drive_;
    private final RobotState robot_state_;

    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
        new PathConstraints(3.0, 2.0),
        new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)),
        new PathPoint(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0))
    );

    PathPlannerTrajectory traj2 = PathPlanner.loadPath("path_new", new PathConstraints(3.0, 2.0));

    // Constructor
    public DriveTrajectory(Drive drive) {
        drive_ = drive;
        robot_state_ = new RobotState(drive_);
    }

    public Command followTrajectoryCommand(boolean isFirstTrajectory) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstTrajectory) {
                    robot_state_.reset(traj2.getInitialHolonomicPose());
                    SmartDashboard.putNumber("Trajectory Time", traj2.getTotalTimeSeconds());
                }
            }),
            new PPSwerveControllerCommand(
                traj2,
                robot_state_::getPosition,
                drive_.getKinematics(),
                Constants.xController,
                Constants.yController,
                Constants.thetaController,
                drive_::setModuleStates,
                false,
                drive_
            ),
            new InstantCommand(() -> {
                drive_.HoldPosition();
            })
        );
    }
}
