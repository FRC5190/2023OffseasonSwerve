package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    // REMEMBER TO CHANGE THE GAINS
    public static final double kp = 1.0;
    public static final PIDController xController = new PIDController(kp, 0, 0);
    public static final PIDController yController = new PIDController(kp, 0, 0);

    
    // public static final ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
    public static final PIDController thetaController = new PIDController(1.0, 0, 0);
    
}
