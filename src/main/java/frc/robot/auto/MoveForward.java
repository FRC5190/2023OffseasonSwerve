package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class MoveForward extends SequentialCommandGroup {
    public MoveForward(Drive drive_) {
        addRequirements(drive_);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("path_new", 1.0, 0.5);

        
    }
}
