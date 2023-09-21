package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends CommandBase{
    // Subsystem
    private final Arm arm_;

    // Position 
    private final double position_;

    public ArmToPosition(Arm arm, double position) {
        // Assign member variables
        arm_ = arm;
        position_ = position;

        addRequirements(arm_);
    }

    @Override
    public void initialize() {
        arm_.setAngle(position_);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm_.getAngle() - position_) < Constants.kTolerance;
    }

    // Constants
    private static final class Constants {
        public static final double kTolerance = 0.087;
    }
}
