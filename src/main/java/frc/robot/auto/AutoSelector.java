package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class AutoSelector {
    private final SendableChooser<Routine> routine_chooser_;
    private final Drive drive_;
    private final DriveTrajectory drive_trajectory_;

    public AutoSelector(Drive drive) {
        routine_chooser_ = new SendableChooser<>();
        routine_chooser_.setDefaultOption("Go Forward While Turning", Routine.GO_FORWARD_WHILE_TURNING);
        drive_ = drive;
        drive_trajectory_ = new DriveTrajectory(drive_);
    }

    public Command run() {
        if(routine_chooser_.getSelected() == Routine.GO_FORWARD_WHILE_TURNING) {
            return drive_trajectory_.followTrajectoryCommand(Trajectories.forward_trajectory, false);
        }
        else {
            return drive_trajectory_.followTrajectoryCommand(Trajectories.forward_trajectory, false);
        }
    }

    public SendableChooser<Routine> getRoutineChooser() {
        return routine_chooser_;
    }

    public Routine getRoutine() {
        return routine_chooser_.getSelected();
    }

    public enum Routine {
        GO_FORWARD_WHILE_TURNING,
    }

    public static class Trajectories {
        public static final PathPlannerTrajectory forward_trajectory = PathPlanner.loadPath(
            "GO_FORWARD_WHILE_TURNING", new PathConstraints(1.0, 0.5));
    }
}
