package frc.robot.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drive;

public class AutoConfig {
    // Constraints
    public static final double kMaxVelocity = 2.5;
    public static final double kMaxAcceleration = 1.9;
    // public static final double kMaxAngularVelocity = 2.5; kMaxCentripetalAcceleration

    // Trajectory Configs
    public static final TrajectoryConfig kConfig = 
        new TrajectoryConfig(kMaxVelocity, kMaxAcceleration).setKinematics(Drive.getKinematics());

    // PID Controllers
    private static PIDController xController = new PIDController(Constants.kPXController, 0.0, 0.0);
    private static PIDController yController = new PIDController(Constants.kPYController, 0.0, 0.0);
    private static ProfiledPIDController thetaController = 
        new ProfiledPIDController(Constants.kPThetaController, 0.0, 0.0, Constants.kThetaControllerConstraints);

    // Controller
    public static HolonomicDriveController controller_ = new HolonomicDriveController(
        xController, yController, thetaController
    );

    // Methods
    public static HolonomicDriveController getController() {
        return controller_;
    }

    // Constants Class
    private static class Constants {
        // Control *Tune after testing*
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final double kPThetaController = 1.0;

        // Constraints
        public static final double kMaxAngularVelocity = Math.PI;
        public static final double kMaxCentripetalAcceleration = Math.PI;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxCentripetalAcceleration);
    }
}
