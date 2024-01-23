// package frc.robot.auto;

// import frc.robot.subsystems.*;

// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class FARLeaveCommunity extends SequentialCommandGroup {

//     public FARLeaveCommunity(Swerve s_Swerve) {

//         // PathPlannerAuto path1 = PathPlannerAuto("LeaveCommunity");
//     //     PathPlannerTrajectory path1 = PathPlannerAuto.loadpath("LeaveCommunity", new PathConstraints(4, 3, Math.PI, Math.PI));
//        PathPlannerTrajectory path4 = PathPlanner.loadPath("FARLeaveCommunity", 4.5, 3.0);
        
//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.1),
//             s_Swerve.followTrajectoryCommand(path4, true)
//         );

//     }
    
// }
