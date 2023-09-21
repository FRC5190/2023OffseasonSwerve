package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.subsystems.Arm;

public class AutoSelector {
    // private Command auto_command_ = null;

    public AutoSelector() {
        ;
    }

    public Command run(RobotState robot_state, Superstructure superstructure, Arm arm) {
        return new ArmSimTest(superstructure, arm);
    }
}
