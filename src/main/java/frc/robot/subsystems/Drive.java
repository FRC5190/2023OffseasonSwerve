package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // Swerve Module Instances
    private final SwerveModule front_left_ = new SwerveModule(Constants.kFrontLeftConfig);
    private final SwerveModule front_right_ = new SwerveModule(Constants.kFrontRightConfig);
    private final SwerveModule back_left_ = new SwerveModule(Constants.kBackLeftConfig);
    private final SwerveModule back_right_ = new SwerveModule(Constants.kBackRightConfig);

    // Swerve Modules Array
    private final SwerveModule[] modules_ = {front_left_, front_right_, back_left_, back_right_};

    // navX Gyro
    private final AHRS gyro_;

    // Kinematics
    SwerveDriveKinematics kinematics_ = new SwerveDriveKinematics(
        Constants.kFrontLeftLocation, Constants.kFrontRightLocation,
        Constants.kBackLeftLocation, Constants.kBackRightLocation);

    // IO
    private final IO io_ = new IO();
    private OutputType output_type_ = OutputType.OPEN_LOOP;

    // PID Controllers
    PIDController turn_controller_ = new PIDController(Constants.kP, 0, 0);

    // Constructor
    public Drive() {
        // Initialize Gyro
        gyro_ = new AHRS(SPI.Port.kMXP);
        new Thread(() -> {
            try {
                // Wait for 1 second (hardware initialization) before zeroing heading
                Thread.sleep(1000);
                gyro_.zeroYaw();
            } catch (Exception ignored) {
            }
        }).start();
    }

    @Override
    public void periodic() {
        // Read inputs
        io_.angle = -Math.toRadians(gyro_.getYaw());

        // Calculate outputs
        SwerveModuleState[] module_states = kinematics_.toSwerveModuleStates(io_.speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(module_states, Constants.kMaxSpeed);

        // Set outputs and telemetry
        for (int i = 0; i < modules_.length; i++) {
            modules_[i].setDesiredState(module_states[i], output_type_);

            SmartDashboard.putNumber(String.format("Module [%d] Speed", i),
                modules_[i].getDriveVelocity());
            SmartDashboard.putNumber(String.format("Module [%d] Angle", i),
                modules_[i].getSteerPosition().getDegrees());
        }
    }

    // Get Module Positions
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            front_left_.getModulePosition(),
            front_right_.getModulePosition(),
            back_left_.getModulePosition(),
            back_right_.getModulePosition(),
        };
    }

    // Get Gyro Angle
    public Rotation2d getAngle() {
        return new Rotation2d(io_.angle);
    }

    // Get Kinematics
    public SwerveDriveKinematics getKinematics() {
        return kinematics_;
    }

    // Set Speeds
    public void setSpeeds(ChassisSpeeds speeds, OutputType output_type) {
        io_.speeds = speeds;
        output_type_ = output_type;
    }

    // Set Speeds (Closed Loop)
    public void setSpeeds(ChassisSpeeds speeds) {
        setSpeeds(speeds, OutputType.VELOCITY);
    }

    // Puts wheels in X shape for brake
    public void HoldPosition() {
        front_left_.setAngle(45);
        front_right_.setAngle(-45);
        back_left_.setAngle(-45);
        back_right_.setAngle(45);
    }

    /**
     * Sets Robot angle - relative to field
     * @param target_angle_degs In Degrees
     */
    public void setRobotAngle(double target_angle_degs) {
        // Convert degrees to radians
        double target_angle_rad = Math.toRadians(target_angle_degs);

        // Calculate angle correction
        double angle_correction = turn_controller_.calculate(
            getAngle().getRadians(), target_angle_rad);
        
        // Set speeds using angle correction
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, angle_correction);
        setSpeeds(speeds);
    }

    // Output Type
    public enum OutputType {
        VELOCITY, OPEN_LOOP
    }

    // IO
    private static class IO {
        // Inputs
        double angle;

        // Outputs
        ChassisSpeeds speeds = new ChassisSpeeds();
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

        // Max Velocity
        public static final double kMaxSpeed = 3.66;

        // Module Locations
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.27, 0.27);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.27, -0.27);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.27, 0.27);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.27, -0.27);

        // PID Constants
        public static final double kP = 0.35;
    }
}
