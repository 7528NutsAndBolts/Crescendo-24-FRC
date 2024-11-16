package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutonomousSelector {

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("Test Auto 2.10", AutonomousMode.Test_Auto);
        autonomousModeChooser.addOption("No Auto", AutonomousMode.NoAuto);
        autonomousModeChooser.addOption("Straight Out", AutonomousMode.Straight_Out);

        autoTab.add("Mode", autonomousModeChooser);
        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case Test_Auto:
                return new PathPlannerAuto("Test Auto 2.10");
            case NoAuto:
                return new NoAuto(s_Swerve);
            case Straight_Out:
                return new PathPlannerAuto("Straight Out");
            default:
                return new PathPlannerAuto("Test Auto 2.10");
        }
    }

    public AutonomousSelector() {
        
    }

    private enum AutonomousMode {
        Test_Auto,
        NoAuto,
        Straight_Out

    }

}