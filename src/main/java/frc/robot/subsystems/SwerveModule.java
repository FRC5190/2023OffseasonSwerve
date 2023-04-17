package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    // Swerve Module Configuration
    private final Configuration configuration_;

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

    // Constructor
    public SwerveModule(Configuration configuration) {
        // Store configuration.
        configuration_ = configuration;

        // Initialize motor controllers
        drive_motor_ = new CANSparkMax(configuration_.drive_id, CANSparkMax.MotorType.kBrushless);
        drive_motor_.restoreFactoryDefaults();
        drive_motor_.setIdleMode(CANSparkMax.IdleMode.kBrake);

        steer_motor_ = new CANSparkMax(configuration_.steer_id, CANSparkMax.MotorType.kBrushless);
        steer_motor_.restoreFactoryDefaults();
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
        cancoder_.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition);

        // Initialize PID controllers
        drive_pid_controller_ = new PIDController(Constants.kDriveKp, 0.0, 0.0);

        steer_pid_controller_ = new PIDController(Constants.kSteerKp, 0.0, 0.0);
        steer_pid_controller_.enableContinuousInput(-Math.PI, Math.PI);

        // Reset encoders
        resetEncoders();
    }

    // Get Drive Position
    public double getDrivePosition() {
        return drive_encoder_.getPosition();
    }

    // Get Steer Position
    public Rotation2d getSteerPosition() {
        return new Rotation2d(
            Math.IEEEremainder(steer_encoder_.getPosition(), 2 * Math.PI));
    }

    // Get Module Position
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
    }

    // Get Drive Velocity
    public double getDriveVelocity() {
        return drive_encoder_.getVelocity();
    }

    // Get CANCoder Absolute Position
    public double getCANCoderDeg() {
        return cancoder_.getAbsolutePosition() - configuration_.module_offset_deg;
    }

    // Get Module State
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
    }

    // Reset Encoders
    public void resetEncoders() {
        drive_encoder_.setPosition(0);
        steer_encoder_.setPosition(Math.toRadians(getCANCoderDeg()));
    }

    /**
     * Sets Steer angle for individual module
     * @param desired_angle In Degrees
     */
    public void setAngle(double desired_angle_degs) {
        // Convert deg to rad
        double desired_angle_rad = Math.toRadians(desired_angle_degs);

        // Set steer angle
        double steering_correction = steer_pid_controller_.calculate(
            getSteerPosition().getRadians(), desired_angle_rad);
        steer_motor_.set(steering_correction);
    }

    // Set Desired State
    public void setDesiredState(SwerveModuleState state, Drive.OutputType output_type) {
        // Optimize state to minimize change in angle
        state = SwerveModuleState.optimize(state, getState().angle);

        // Set steer angle
        double steering_correction = steer_pid_controller_.calculate(
            getSteerPosition().getRadians(), state.angle.getRadians());
        steer_motor_.set(steering_correction);

        // Set drive output
        switch (output_type) {
            case OPEN_LOOP:
                drive_motor_.set(state.speedMetersPerSecond / Constants.kMaxModuleSpeed);
                break;
            case VELOCITY:
                double drive_correction = drive_pid_controller_.calculate(
                    getDriveVelocity(), state.speedMetersPerSecond);
                drive_motor_.set(drive_correction);
                break;
        }
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
        // Control
        public static final double kDriveKp = 0.5;
        public static final double kSteerKp = 0.5;

        // Hardware
        public static final double kDriveGearRatio = 8.14;
        public static final double kSteerGearRatio = 150.0 / 7;
        public static final double kWheelRadius = 0.0508;
        public static final double kMaxModuleSpeed = 3.66;
    }
}
