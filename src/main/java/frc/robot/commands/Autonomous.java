package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class Autonomous {
    // Instance variables
    private final Drive drive_;
    private final DriveTrajectory drive_traj_;
    private final Field2d field_ = new Field2d();
    private final PathPlannerTrajectory trajectory = PathPlanner.loadPath(
        "path", new PathConstraints(4, 3));

    // Constructor
    public Autonomous(Drive drive) {
        // Initialize the field in the sim
        // TO DO probably need to change this to be instantiated in the robot class
        SmartDashboard.putData("Field", field_);

        drive_ = drive;
        drive_traj_ = new DriveTrajectory(drive_);
    }

    // Methods
    public PathPlannerTrajectory getTrajectory(){
        return trajectory;
    }

    public Command run() {
        return drive_traj_.followTrajectoryCommand(getTrajectory(), true);
    }

    public void simulationPeriodic() {
        // Timer 
        Timer timer = new Timer();

        timer.start();
        // Sample the trajectory
        while (timer.get() <= trajectory.getTotalTimeSeconds()){
            // Get the state of the robot at the current time
            // Ensure it is a PathPlannerState because only then will it have the holonomicRotation
            PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
            
            // Set Robot position in simulation
            field_.setRobotPose(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation);
        }
    }
}