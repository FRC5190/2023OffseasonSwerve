package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTrajectory {
    // Instance variables
    private final Drive drive_;
    private final RobotState robot_state_;

    // Constructor
    public DriveTrajectory(Drive drive) {
        drive_ = drive;
        robot_state_ = new RobotState(drive_);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstTrajectory) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstTrajectory)
                    robot_state_.reset(traj.getInitialPose());
            }),
            new PPSwerveControllerCommand(
                traj,
                robot_state_::getPosition,
                drive_.getKinematics(),
                Constants.xController,
                Constants.yController,
                Constants.thetaController,
                drive_::setModuleStates,
                false,
                drive_
            )
            
            // new SwerveControllerCommand(
            //     traj,
            //     robot_state_::getPosition,
            //     drive_.getKinematics(),
            //     Local.holonomicDriveController,
            //     drive_::setModuleStates,
            //     drive_
            // )
        );
    }

    // public static class Local {
    //     public static HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    //         Constants.xController,
    //         Constants.yController,
    //         Constants.thetaController
    //     );
    // }
    
}
