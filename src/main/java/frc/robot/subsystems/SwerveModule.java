package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // Motor Controllers
    private final CANSparkMax drive_motor_;
    private final CANSparkMax steer_motor_;

    // Sensors
    private final RelativeEncoder drive_encoder_;
    private final RelativeEncoder steer_encoder_;
    private final CANCoder cancoder_;

    // Control
    private final PIDController drive_pid_controller_; // in m/s
    private final PIDController steer_pid_controller_; // in radians

    // State
    private final int enc_id_;
    private final double enc_offset_;

    // Constructor
    public SwerveModule(Configuration configuration) {
        // Initialize motor controllers
        drive_motor_ = new CANSparkMax(configuration.drive_id, CANSparkMax.MotorType.kBrushless);
        steer_motor_ = new CANSparkMax(configuration.steer_id, CANSparkMax.MotorType.kBrushless);

        drive_motor_.restoreFactoryDefaults();
        steer_motor_.restoreFactoryDefaults();

        drive_motor_.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steer_motor_.setIdleMode(CANSparkMax.IdleMode.kBrake);

        steer_motor_.setInverted(true);

        // Initialize encoders
        drive_encoder_ = drive_motor_.getEncoder();
        drive_encoder_.setPositionConversionFactor(
                2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio);
        drive_encoder_.setVelocityConversionFactor(
                2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio / 60);

        steer_encoder_ = steer_motor_.getEncoder();
        steer_encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kSteerGearRatio);
        steer_encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kSteerGearRatio / 60);

        cancoder_ = new CANCoder(configuration.cancoder_id);
        cancoder_.configFactoryDefault();
        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
//        config.sensorCoefficient = 2 * Math.PI / 4096.0;
//        config.unitString = "rad";
//        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = 0;
        cancoder_.configAllSettings(config);

        // CANCoder id used for Smart Dashboard
        enc_id_ = configuration.cancoder_id;
        enc_offset_ = configuration.module_offset_deg;

        // Initialize PID controllers
        drive_pid_controller_ = new PIDController(Constants.kDriveKp, 0.0, 0.0);

        steer_pid_controller_ = new PIDController(Constants.kSteerKp, 0.0, 0.0);
        steer_pid_controller_.enableContinuousInput(-Math.PI, Math.PI);

        // Reset encoders
        resetEncoders();

        // Reset CANCoder
        cancoder_.configFactoryDefault();
    }

    // Get Drive Position
    public double getDrivePosition() {
        return drive_encoder_.getPosition();
    }

    // Get Steer Position
    public Rotation2d getSteerPosition() {
        return new Rotation2d(steer_encoder_.getPosition() % (2 * Math.PI));
    }

    // Get Module Position
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
    }

    // Get Drive Velocity
    public double getDriveVelocity() {
        // ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants
        // .kDriveEncoderRPM2MeterPerSec)
        return drive_encoder_.getVelocity();
    }

    // Get Steer Velocity
    public double getSteerVelocity() {
        return steer_encoder_.getVelocity();
    }

    // Get CANCoder Absolute Position
    public double getCANCoderDeg() {
        return cancoder_.getAbsolutePosition() - enc_offset_;
    }

    // Reset Encoders
    public void resetEncoders() {
        drive_encoder_.setPosition(0);
        steer_encoder_.setPosition(Math.toRadians(getCANCoderDeg()));
    }

    // Get Module State
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
    }

    public void periodic() {
//        SmartDashboard.putNumber("Swerve[" + enc_id_ + "]", getSteerPosition().getDegrees());
//        SmartDashboard.putNumber("Swerve[" + enc_id_ + "] V", getDriveVelocity());

        SmartDashboard.putNumber("Swerve[" + enc_id_ + "]", steer_pid_controller_.getPositionError());
//        drive_motor_.set(drive_pid_controller_.calculate(getDriveVelocity()));
        drive_motor_.set(drive_pid_controller_.getSetpoint());
        steer_motor_.set(steer_pid_controller_.calculate(getSteerPosition().getRadians()));
    }

    // Set Desired State
    public void setDesiredState(SwerveModuleState state) {
        // Add telemetry to SmartDashboard
        state = SwerveModuleState.optimize(state, getState().angle);

        drive_pid_controller_.setSetpoint(state.speedMetersPerSecond);
        steer_pid_controller_.setSetpoint(state.angle.getRadians());

    }

    // Stop Motors
    public void stop() {
        drive_motor_.set(0);
        steer_motor_.set(0);
    }

    // Module Configuration
    public static class Configuration {
        public int drive_id;
        public int steer_id;
        public int cancoder_id;

        public double module_offset_deg;
    }

    // Constants Class
    private static class Constants {
        // Control (TODO: tune)
        public static final double kDriveKp = 0.5;
        public static final double kSteerKp = 0.5;

        // Hardware
        public static double kDriveGearRatio = 8.14;
        public static double kSteerGearRatio = 150.0 / 7;
        public static double kWheelRadius = 0.0508;
    }
}
