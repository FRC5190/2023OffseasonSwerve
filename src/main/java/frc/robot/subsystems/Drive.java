package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    // Pigeon 2.0 Gyro
    // private WPI_Pigeon2 gyro_;

    // navX Gyro
    private AHRS gyro_;

    // Create instances for each Swerve Module
    private final SwerveModule front_left_ = new SwerveModule(Constants.kFrontLeftConfig);
    private final SwerveModule front_right_ = new SwerveModule(Constants.kFrontRightConfig);
    private final SwerveModule back_left_ = new SwerveModule(Constants.kBackLeftConfig);
    private final SwerveModule back_right_ = new SwerveModule(Constants.kBackRightConfig);

    // Create kinematics object
    SwerveDriveKinematics kinematics_ = new SwerveDriveKinematics(
            Constants.kFrontLeftLocation,
            Constants.kFrontRightLocation,
            Constants.kBackLeftLocation,
            Constants.kBackRightLocation
    );

    SwerveDriveOdometry odometer_;

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
            } catch (Exception ignored) {
            }
        }).start();

        odometer_ = new SwerveDriveOdometry(kinematics_, getRotation2d(),
                getSwerveModulePositions());
    }

    // Methods
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
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
        return Rotation2d.fromRadians(-getYaw());
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
    public void periodic() {
        odometer_.update(getRotation2d(), getSwerveModulePositions());
        SmartDashboard.putNumber("Robot Heading", getYaw());
        SmartDashboard.putNumber("Robot Theta", getPose().getRotation().getDegrees());
        front_left_.periodic();
        front_right_.periodic();
        back_left_.periodic();
        back_right_.periodic();
    }

    public void stopModules() {
        front_left_.stop();
        front_right_.stop();
        back_left_.stop();
        back_right_.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Incase any module is given a speed that is higher than the max,
        // desaturateWheelSpeeds() will reduce all of the module speeds until all module speeds
        // are under the limit
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.kMaxSpeed);

        front_left_.setDesiredState(desiredStates[0]);
        front_right_.setDesiredState(desiredStates[1]);
        back_left_.setDesiredState(desiredStates[2]);
        back_right_.setDesiredState(desiredStates[3]);
    }

    // Constants Class
    private static class Constants {
        // Swerve Modules
        public static final SwerveModule.Configuration kFrontLeftConfig =
                new SwerveModule.Configuration();
        public static final SwerveModule.Configuration kFrontRightConfig =
                new SwerveModule.Configuration();
        public static final SwerveModule.Configuration kBackLeftConfig =
                new SwerveModule.Configuration();
        public static final SwerveModule.Configuration kBackRightConfig =
                new SwerveModule.Configuration();

        // Initialize Swerve Module Constants
        static {
            kFrontLeftConfig.drive_id = 1;
            kFrontLeftConfig.steer_id = 2;
            kFrontLeftConfig.cancoder_id = 11;
            kFrontLeftConfig.module_offset_deg = 9.49;

            kFrontRightConfig.drive_id = 3;
            kFrontRightConfig.steer_id = 4;
            kFrontRightConfig.cancoder_id = 12;
            kFrontRightConfig.module_offset_deg = 267.13;

            kBackLeftConfig.drive_id = 5;
            kBackLeftConfig.steer_id = 6;
            kBackLeftConfig.cancoder_id = 13;
            kBackLeftConfig.module_offset_deg = 295.57;

            kBackRightConfig.drive_id = 7;
            kBackRightConfig.steer_id = 8;
            kBackRightConfig.cancoder_id = 14;
            kBackRightConfig.module_offset_deg = 279.58;
        }

        // Gyro ID
        public static final int kGyroId = 17;

        // Max Velocity
        public static final int kMaxSpeed = 1;

        // * UPDATE BASED ON DRIVETRAIN CONSTRUCTION *
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.27, 0.27);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.27, -0.27);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.27, 0.27);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.27, -0.27);
    }
}
