package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTrajectory extends CommandBase{
    private final PathPlannerTrajectory traj_;
    private final Timer timer_;
    private double t;
    
    public DriveTrajectory(PathPlannerTrajectory traj) {
        traj_ = traj;
        timer_ = new Timer();
    }


    @Override
    public void initialize() {
        timer_.start();
    }

    @Override
    public void execute() {
        t = timer_.get();
    }

    public Pose2d getTrajectoryPose(double time) {
        final PathPlannerState state = (PathPlannerState) traj_.sample(t);
        return new Pose2d(new Translation2d(state.poseMeters.getX(), state.poseMeters.getY()), state.holonomicRotation);
    }

    public Command followTrajectoryCommand(boolean isFirstTrajectory, RobotState robot_state_, Drive drive_) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstTrajectory) {
                    robot_state_.reset(traj_.getInitialHolonomicPose());
                    SmartDashboard.putNumber("Trajectory Total Time", traj_.getTotalTimeSeconds());
                }
            }),
            new PPSwerveControllerCommand(
                traj_,
                robot_state_::getPosition,
                drive_.getKinematics(),
                Constants.kXController,
                Constants.kYController,
                Constants.kThetaController,
                drive_::setModuleStates,
                true,
                drive_
            )
        );
    }

    public double getTrajectoryTime() {
        return traj_.getTotalTimeSeconds();
    }

    public static class Constants {
        public static double kP = 0.5;

        public static PIDController kXController = new PIDController(kP, 0, 0);
        public static PIDController kYController = new PIDController(kP, 0, 0);
        public static PIDController kThetaController = new PIDController(kP, 0, 0);
    }
    
}
