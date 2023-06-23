package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTrajectory {
    // Instance variables
    // private final Autonomous auto_path_ = new Autonomous();
    private final Drive drive_;
    private final RobotState robot_state_;

    public DriveTrajectory(Drive drive){
        drive_ = drive;
        robot_state_ = new RobotState(drive_);
    }


    // Method to run Holonomic Trajectory path following
    // Need to make it look cleaner but its proof of concept
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstPath)
                    robot_state_.resetPose(traj.getInitialHolonomicPose());
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
        );
    }


    // Constants Class
    private static class Constants{
        // NOTE BEFORE RUNNING: CHANGE GAINS!
        public static final PIDController xController = new PIDController(1, 0, 0);
        public static final PIDController yController = new PIDController(1, 0, 0);
        public static final PIDController thetaController = new PIDController(0.5, 0, 0);
    }


}
