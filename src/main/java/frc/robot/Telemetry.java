package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Arm;

public class Telemetry {
    // Periodic
    private final List<Runnable> periodic_registry_ = new ArrayList<>();

    // Visualizations
    private final Field2d field_;
    private final Mechanism2d superstructure_;


    public Telemetry(RobotState robot_state, Arm arm) {
        // Shuffleboard tab
        ShuffleboardTab tab_ = Shuffleboard.getTab("2023 Offseason");
        

        // Field Visualization
        field_ = new Field2d();
        tab_.add("Field", field_);
        periodic_registry_.add(() -> field_.setRobotPose(robot_state.getPosition()));

        // Mechanism Visualization
        superstructure_ = new Mechanism2d(1.2, 1.2);
        tab_.add("Superstructure", superstructure_);

        superstructure_.getRoot("Structure Root", 0.33, 0.1).append(
            new MechanismLigament2d("Structure", 0.5, 0));
        MechanismLigament2d arm_out = superstructure_.getRoot("Carriage Root", 0.35, 0.1).append(
            new MechanismLigament2d("Arm", 0.15, 0, 3, new Color8Bit(Color.kOrangeRed)));
        
        // Update Visualization
        periodic_registry_.add(() -> arm_out.setAngle(Math.toDegrees(arm.getAngle())));

        /**
        // Put autonomous mode selector on Shuffleboard.
        ShuffleboardLayout auto_layout = tab_.getLayout("Autonomous", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
        auto_layout.add("Routine", auto_selector.getRoutineChooser())
            .withSize(2,1);
        auto_layout.add("Grid", auto_selector.getGridChooser())
            .withSize(2,1);
        */

        // Add arm information
        ShuffleboardLayout arm_layout = tab_.getLayout("Arm", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(0, 2);
        arm_layout.addNumber("Position (deg)", () -> Math.toDegrees(arm.getAngle()))
            .withPosition(0, 0);
        arm_layout.addNumber("Velocity (dps)", () -> Math.toDegrees(arm.getAngularVelocity()))
            .withPosition(1, 0);
        arm_layout.addNumber("Velocity Setpoint (dps)",
                () -> Math.toDegrees(arm.getAngularVelocitySetpoint()))
            .withPosition(0, 1);
    }

    public void periodic() {
        for (Runnable fn : periodic_registry_)
            fn.run();
        // System.out.println(periodic_registry_);
    }
}
