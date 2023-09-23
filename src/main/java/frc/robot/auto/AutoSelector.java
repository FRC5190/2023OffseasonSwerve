package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

public class AutoSelector {
    private Command autonomous_command_ = null;

    // Sendable Choosers
    private final SendableChooser<Routine> routine_chooser_;

    public AutoSelector() {
        // Initialize Sendable Choosers
        routine_chooser_ = new SendableChooser<>();
        routine_chooser_.setDefaultOption("Score Low Taxi Bump", Routine.LOWTAXIBUMP);
    }

    //Routine Chooser Getter
    public SendableChooser<Routine> getRoutineChooser() {
        return routine_chooser_;
    }

    public Command run(RobotState robot_state, Superstructure superstructure, Arm arm, Drive drive) {
        if(routine_chooser_.getSelected() == Routine.LOWTAXIBUMP) {
            autonomous_command_ = new ArmSimTest(superstructure, arm, drive, robot_state);
            return autonomous_command_;
        }
        else {
            return new ArmSimTest(superstructure, arm, drive, robot_state);
        }
    }

    public enum Routine {
        LOWTAXIBUMP
    }
}
