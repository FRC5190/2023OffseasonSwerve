package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class AutoSelector {
    private Command autonomous_command_ = null;

    // Sendable Choosers
    private final SendableChooser<Routine> routine_chooser_;

    // Constructor
    public AutoSelector() {
        routine_chooser_ = new SendableChooser<>();
        routine_chooser_.setDefaultOption("Drive Forward and Rotate", Routine.DRIVEFORWARDROTATE);
    }

    // Getters
    public SendableChooser<Routine> getRoutineChooser() {
        return routine_chooser_;
    }

    // Run Autos
    public Command run(Drive drive_){
        if (routine_chooser_.getSelected() == Routine.DRIVEFORWARDROTATE) {
            autonomous_command_ = new DriveForwardRotate(drive_);
            return autonomous_command_;
        }
        else {
            autonomous_command_ = new DriveForwardRotate(drive_);
            return autonomous_command_;
        }
    }


    // Enums
    public enum Routine {
        DRIVEFORWARDROTATE,
    }
}
