package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToPosition;
import frc.robot.subsystems.Arm;

public class Superstructure {
    // Subsystems
    private final Arm arm_;

    // Position State
    public String state = "STOW";

    public Superstructure(Arm arm) {
        arm_ = arm;
    }


    public Command setPosition(Position pos) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> this.state = pos.posname),
            new ArmToPosition(arm_, pos.angle).withTimeout(3)
        );
    }

    public String getState() {
        return state;
    }




    // Positions
    public enum Position {
        // Stow everything inside
        STOW(170, "STOW"),

        // Ground Intake
        INTAKE(-35, "INTAKE"),

        // Cube Scoring
        CUBE_L1(15, "CUBE_L1"),
        CUBE_L2(55, "CUBE_L2");

        final double angle;
        final String posname;

        Position(double angle_deg, String name) {
            this.angle = Math.toRadians(angle_deg);
            this.posname = name;
        }
    }
}
