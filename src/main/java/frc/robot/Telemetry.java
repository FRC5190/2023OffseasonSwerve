package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.AutoSelector;

public class Telemetry {

    public Telemetry(AutoSelector auto_selector) {
        // Create the shuffleboard tab for the 2023 Swerve
        ShuffleboardTab tab_ = Shuffleboard.getTab("2023 Swerve");

        ShuffleboardLayout auto_layout = tab_.getLayout("Autonomous", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
        auto_layout.add("Routine", auto_selector.getRoutineChooser())
            .withSize(2,1);
    }
    
}
