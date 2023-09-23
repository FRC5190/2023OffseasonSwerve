package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Superstructure;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

public class ArmSimTest extends SequentialCommandGroup{
    // Trajectories
    private final PathPlannerTrajectory traj1 = PathPlanner.loadPath("LowScoreTaxiBump", new PathConstraints(4, 3));

    // private final Drive drive_;
    // private final frc.robot.RobotState robot_state_;

    // private final DriveTrajectory follow = new DriveTrajectory()


    // Constructor
    public ArmSimTest(Superstructure superstructure, Arm arm, Drive drive, frc.robot.RobotState robot_state) {
        // drive_ = drive;
        // robot_state_ = robot_state;
        addCommands(
            superstructure.setPosition(Superstructure.Position.STOW),
            new WaitCommand(1.0),
            superstructure.setPosition(Superstructure.Position.CUBE_L1),
            new WaitCommand(2.5),
            superstructure.setIntakePercent(0.5),
            new WaitCommand(1.0),
            new ParallelCommandGroup(
                superstructure.setPosition(Superstructure.Position.STOW),
                superstructure.setIntakePercent(0.0)
            ),
            new InstantCommand(() -> new DriveTrajectory(traj1).followTrajectoryCommand(true, robot_state, drive))
        );
    }
} 
