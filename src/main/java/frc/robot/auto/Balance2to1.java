// package frc.robot.auto;

// import frc.robot.subsystems.*;

// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class Balance2to1 extends SequentialCommandGroup {

//     public Balance2to1(Swerve s_Swerve) {

//        PathPlannerTrajectory path3 = PathPlanner.loadPath("Balance2to1", 4.5, 3.0);
        
//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.1),
//             new GrabCone(s_Swerve).withTimeout(8).andThen(new WaitCommand(1)),
//             s_Swerve.followTrajectoryCommand(path3, false).withTimeout(4),
//             new AutoBalance(s_Swerve).withTimeout(4)
        
//         );

//     }
    
// }