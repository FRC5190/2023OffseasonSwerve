package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final CANSparkMax intake_motor_;

    private final PeriodicIO io_ = new PeriodicIO();

    public Intake() {
        intake_motor_ = new CANSparkMax(Constants.kIntakeMotorId, CANSparkMax.MotorType.kBrushless);
        intake_motor_.restoreFactoryDefaults();
        intake_motor_.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake_motor_.enableVoltageCompensation(12);
        intake_motor_.setInverted(true);
    }

    @Override
    public void periodic() {
        // Read inputs
        io_.supply_current = intake_motor_.getOutputCurrent();

        // Write outputs
        intake_motor_.set(io_.demand);

    }

    public void setPercent(double value) {
        io_.demand = value;
    }

    public String getState() {
        if (io_.supply_current > 0) {
            return "INTAKING";
        } else if(io_.supply_current < 0) {
            return "OUTTAKING";
        }
        else {
            return "IDLE";
        }
    }

    public static class PeriodicIO {
        // Inputs
        double supply_current;

        // Outputs
        double demand;
    }

    public static class Constants {
        // Motor ID
        public static final int kIntakeMotorId = 10;
    }
}
