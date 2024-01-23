package frc.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;


public class AutonomousSelector {

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        //autonomousModeChooser.addOption("2 Cone Auto", AutonomousMode.Two_Cone_Auto);
        //autonomousModeChooser.addOption("2 Cone Balance Auto", AutonomousMode.Two_Cone_Balance_Auto);
        // autonomousModeChooser.addOption("Center Balance Auto", AutonomousMode.Center_Balance_Auto);
        // autonomousModeChooser.addOption("Cone Cube Auto", AutonomousMode.Cone_Cube_Auto);
        // autonomousModeChooser.addOption("Cone Cube Balance Auto", AutonomousMode.Cone_Cube_Balance_Auto);
        // //autonomousModeChooser.addOption("Barrier Balance Auto", AutonomousMode.Barrier_Balance_Auto);
        // //autonomousModeChooser.addOption("Barrier Grab Balance Auto", AutonomousMode.Barrier_Grab_Balance_Auto);
        // autonomousModeChooser.addOption("Barrier Spit Auto", AutonomousMode.Barrier_Spit_Auto);
        autonomousModeChooser.addOption("No Auto", AutonomousMode.NoAuto);
        autonomousModeChooser.addOption("Default Auto", AutonomousMode.DefaultAuto);
        autonomousModeChooser.addOption("Grab Cone", AutonomousMode.GrabCone);
        autonomousModeChooser.addOption("Balance 2 to 1", AutonomousMode.Balance2to1);
        autonomousModeChooser.addOption("LeaveCom FAR", AutonomousMode.LeaveComFar);



        autoTab.add("Mode", autonomousModeChooser);
        
    }

    // public static Command getCommand(Swerve s_Swerve){
    //     AutonomousMode mode = autonomousModeChooser.getSelected();

    //     switch (mode) {
            //case Two_Cone_Auto:
            //    return new TwoConeAuto(s_Swerve);
            // case NoAuto:
            //    return new NoAuto(s_Swerve);
            // case DefaultAuto:
            // default:
            //     return new DefaultAuto(s_Swerve);
            //     //break; 
            // case GrabCone:
            // return new GrabCone(s_Swerve);
            // case Balance2to1:
            // return new Balance2to1(s_Swerve);
            // case LeaveComFar:
    //         // return new LeaveComFar(s_Swerve);
    //     }

    //     //return null;
    // }

    public AutonomousSelector() {
        
    }

    private enum AutonomousMode {
        //Two_Cone_Auto,
        //Two_Cone_Balance_Auto,
        // Center_Balance_Auto,
        // Cone_Cube_Auto,
        // Cone_Cube_Balance_Auto,
        // //Barrier_Balance_Auto,
        // //Barrier_Grab_Balance_Auto,
        // Barrier_Spit_Auto,
        // Barrier_Spit_Balance_Auto,
        // Balance_Test,
        // AutoScoreHighCone,
        NoAuto,
        DefaultAuto,
        GrabCone,
        Balance2to1,
        LeaveComFar
    }

}
