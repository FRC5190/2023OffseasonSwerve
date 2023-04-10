package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.SPI;
// import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
// import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SubsystemBase{
    
    // Pigeon 2.0 Gyro
        // private WPI_Pigeon2 gyro_;

    // navX Gyro
    private AHRS gyro_; 
    
    // Create instances for each Swerve Module
    private final SwerveModule front_left_ = new SwerveModule(
        Module.FRONT_LEFT.drive_motor_id, 
        Module.FRONT_LEFT.steer_motor_id, 
        Module.FRONT_LEFT.cancoder_id);
    private final SwerveModule front_right_ = new SwerveModule(
        Module.FRONT_RIGHT.drive_motor_id, 
        Module.FRONT_RIGHT.steer_motor_id, 
        Module.FRONT_RIGHT.cancoder_id);
    private final SwerveModule back_left_ = new SwerveModule(
        Module.BACK_LEFT.drive_motor_id, 
        Module.BACK_LEFT.steer_motor_id, 
        Module.BACK_LEFT.cancoder_id);
    private final SwerveModule back_right_ = new SwerveModule(
        Module.BACK_RIGHT.drive_motor_id, 
        Module.BACK_RIGHT.steer_motor_id, 
        Module.BACK_RIGHT.cancoder_id);


    // Creating Kinematics object
    SwerveDriveKinematics kinematics_ = new SwerveDriveKinematics(
        Constants.kFrontLeftLocation,
        Constants.kFrontRightLocation,
        Constants.kBackLeftLocation,
        Constants.kBackRightLocation
    );

    SwerveDriveOdometry odometer_ = new SwerveDriveOdometry(kinematics_, getRotation2d(), getSwerveModulePositions());

    // Constructor
    public Drive() {
        // Initialize Gyro
        gyro_ = new AHRS(SPI.Port.kMXP);

        // Pigeon 2.0 Gyro
            // gyro_ = new WPI_Pigeon2(Constants.kGyroId);
            // gyro_.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

        
        // Thread delays the zeroing of the gyro but allows other stuff to continue happening
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // Methods
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[] {
            front_left_.getModulePosition(),
            front_right_.getModulePosition(),
            back_left_.getModulePosition(),
            back_right_.getModulePosition(),
        };
    }

    public void zeroHeading() {
        gyro_.reset();
    }

    public double getYaw() {
        SmartDashboard.putNumber("Gyro Heading", gyro_.getYaw());
        return Math.toRadians(gyro_.getYaw()); // returns degrees
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Pose2d getPose() {
        return odometer_.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer_.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics_;
    }

    @Override
    public void periodic(){
        odometer_.update(getRotation2d(), getSwerveModulePositions());
        SmartDashboard.putNumber("Robot Heading", getYaw());
        SmartDashboard.putNumber("Robot Theta", getPose().getRotation().getDegrees());
    }
    
    public void stopModules() {
        front_left_.stop();
        front_right_.stop();
        back_left_.stop();
        back_right_.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Incase any module is given a speed that is higher than the max, 
        // desaturateWheelSpeeds() will reduce all of the module speeds until all module speeds are under the limit 
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond); 

        front_left_.setDesiredState(desiredStates[0]);
        front_right_.setDesiredState(desiredStates[1]);
        back_left_.setDesiredState(desiredStates[2]);
        back_right_.setDesiredState(desiredStates[3]);
    }
    
    // Module IDs and CANCoder Offsets
    public enum Module {
        // * ADD VALUES ONCE DRIVETRAIN IS WIRED *
        FRONT_LEFT(0, 0, 0, 0.0),
        FRONT_RIGHT(0, 0, 0, 0.0),
        BACK_LEFT(0, 0, 0, 0.0),
        BACK_RIGHT(0, 0, 0, 0.0);

        final int drive_motor_id;
        final int steer_motor_id;
        final int cancoder_id;
        final double cancoder_offset;

        Module(int drive_motor, int steer_motor, int cancoder, double offset) {
            this.drive_motor_id = drive_motor;
            this.steer_motor_id = steer_motor;
            this.cancoder_id = cancoder;
            this.cancoder_offset = offset;
        }
    }

    // Constants Class
    public static class Constants {
        // Gyro ID
        public static final int kGyroId = 0;

        // Max Velocity?
        public static final int kPhysicalMaxSpeedMetersPerSecond = 1;

        // * UPDATE BASED ON DRIVETRAIN CONSTRUCTION *
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381); 
        public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

    }
}
