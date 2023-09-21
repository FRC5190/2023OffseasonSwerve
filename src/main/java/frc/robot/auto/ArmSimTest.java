package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Superstructure;
import frc.robot.subsystems.Arm;

public class ArmSimTest extends SequentialCommandGroup{
    // Constructor
    public ArmSimTest(Superstructure superstructure, Arm arm) {
        addCommands(
            superstructure.setPosition(Superstructure.Position.STOW),
            new WaitCommand(1.0),
            superstructure.setPosition(Superstructure.Position.CUBE_L2)
        );
    }
}
