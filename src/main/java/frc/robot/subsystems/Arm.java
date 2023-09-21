package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    // Motor Controller
    private final CANSparkMax arm_motor_;

    // Encoder
    private final RelativeEncoder encoder_;

    // Control
    private final ArmFeedforward ff_;
    private final ProfiledPIDController fb_;
    private boolean reset_pid_ = false;

    // IO
    private final PeriodicIO io_ = new PeriodicIO();
    private OutputType output_type_ = OutputType.PERCENT;

    // Simulation
    private final SingleJointedArmSim physics_sim_;
    private final SimDeviceSim arm_motor_sim_;


    public Arm() {
        // Initialize Motor Controller
        arm_motor_ = new CANSparkMax(Constants.kMotorId, MotorType.kBrushless);
        arm_motor_.restoreFactoryDefaults();
        arm_motor_.setInverted(true);
        arm_motor_.setIdleMode(IdleMode.kCoast);

        // Initialize Encoder
        encoder_ = arm_motor_.getEncoder();
        encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
        encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

        // Initialize Control
        ff_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
        fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
            Constants.kMaxVelocity, Constants.kMaxAcceleration));
        
        // Initialize Simulation
        physics_sim_ = new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(Constants.kV, Constants.kA),
            DCMotor.getNEO(1), Constants.kGearRatio, Constants.kArmLength, Constants.kMinAngle,
            Constants.kMaxAngle, false
        );
        arm_motor_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kMotorId + "]");
        
        // Safety
        arm_motor_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
        arm_motor_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
        arm_motor_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);

        // Zero Encoder
        zero();
        physics_sim_.setState(VecBuilder.fill(Constants.kMaxAngle, 0));
    }

    @Override
    public void periodic() {
        // Read inputs
        io_.angle = encoder_.getPosition();
        io_.angular_velocity = encoder_.getVelocity();
        io_.current = arm_motor_.getOutputCurrent();

        if(io_.wants_zero) {
            io_.wants_zero = false;
            encoder_.setPosition(Constants.kMaxAngle);
        }

        // Reset Controller
        if(reset_pid_) {
            reset_pid_ = false;
            fb_.reset(io_.angle, io_.angular_velocity);
        }


        // Write Outputs
        switch(output_type_) {
            case PERCENT:
                arm_motor_.set(io_.demand);
                
                // Set simulated inputs
                if (RobotBase.isSimulation())
                    arm_motor_sim_.getDouble("Applied Output").set(io_.demand * 12);
                break;
            case ANGLE:
                double feedback = fb_.calculate(io_.angle);

                double velocity_setpoint = fb_.getSetpoint().velocity;
                double acceleration_setpoint = (velocity_setpoint - io_.angular_velocity) / 0.02;
                double feedforward = ff_.calculate(io_.angle, velocity_setpoint, acceleration_setpoint);

                arm_motor_.setVoltage(feedback + feedforward);
                break;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update physics sim with inputs
        double voltage = arm_motor_.getAppliedOutput();
        if (output_type_ == OutputType.ANGLE)
        voltage -= Constants.kG * Math.cos(io_.angle);
        physics_sim_.setInputVoltage(voltage);

        // Update physics sim forward in time
        physics_sim_.update(0.02);

        // Update encoder values
        arm_motor_sim_.getDouble("Position").set(physics_sim_.getAngleRads());
        arm_motor_sim_.getDouble("Velocity").set(physics_sim_.getVelocityRadPerSec());
    }

    public void setPercent(double percent) {
        output_type_ = OutputType.PERCENT;
        io_.demand = percent;
        reset_pid_ = true;
    }

    public void setAngle(double angle) {
        output_type_ = OutputType.ANGLE;
        fb_.setGoal(angle);
        reset_pid_ = true;
    }

    public void enableSoftLimits(boolean value) {
        arm_motor_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
        arm_motor_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);
    }

    // Getters
    public double getAngle() {
        return io_.angle;
    }

    public double getAngularVelocity() {
        return io_.angular_velocity;
    }

    public double getAngularVelocitySetpoint() {
        return fb_.getSetpoint().velocity;
    }

    public double getCurrent() {
        return io_.current;
    }

    public void zero() {
        io_.wants_zero = true;
    }

    // Output Type
    private enum OutputType {
        PERCENT, ANGLE
    }

    //IO
    private static class PeriodicIO {
        // Inputs
        double angle;
        double angular_velocity;
        double current;

        // Output
        boolean wants_zero;
        double demand;
    }

    // Constants
    private static class Constants {
        // Motor Controller 
        public static final int kMotorId = 9;

        // Physical Constants
        public static final double kGearRatio = 22.0 / 34.0;
        public static final double kMinAngle = Math.toRadians(-40);
        public static final double kMaxAngle = Math.toRadians(170);
        public static final double kArmLength = 0.30;

        // Feedforward (Run Sysid)
        public static final double kA = 0.041608; //volts * seconds^2 / radians
        public static final double kG = 0.38624; //volts
        public static final double kS = 0.06490; //volts
        public static final double kV = 2.20370; //volts * seconds/radians

        // Feedback 
        public static double kMaxVelocity = 3.14;
        public static double kMaxAcceleration = 3.14;
        public static double kP = 0.01;

        // Current Limit
        public static final double kCurrentLimit = 50;
    }
}
