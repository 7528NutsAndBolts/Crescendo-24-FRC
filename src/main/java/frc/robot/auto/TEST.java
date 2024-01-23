// package frc.robot.auto;

// import frc.robot.subsystems.*;

// import com.pathplanner.*;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class TEST extends SequentialCommandGroup {

//     public TEST(Swerve s_Swerve) {
//         PathPlannerTrajectory path7 = ("TEST", 4.5, 3.0);
        
//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.1),
//             s_Swerve.followTrajectoryCommand(path7, true)
//         );
//     }
// }
