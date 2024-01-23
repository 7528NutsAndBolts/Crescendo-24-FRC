// package frc.robot.auto;

// import frc.robot.subsystems.*;

// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class Tabner extends SequentialCommandGroup {

//     public Tabner(Swerve s_Swerve) {
//         PathPlannerTrajectory path8 = PathPlanner.loadPath("Tabner", 4.5, 3.0);
        
//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.1),
//             s_Swerve.followTrajectoryCommand(path8, true)
//         );
//     }
// }