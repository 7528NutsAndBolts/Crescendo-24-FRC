// package frc.robot.auto;

// import frc.robot.commands.FloorIntakeCommandGroup;
// import frc.robot.commands.StoreStateCommandGroup;
// import frc.robot.commands.LEDs.SetConeMode;
// import frc.robot.subsystems.*;

// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class GrabCone extends SequentialCommandGroup {

//     public GrabCone(Swerve s_Swerve) {

//        PathPlannerTrajectory path5 = PathPlanner.loadPath("GrabCone", 4.5, 3.0);
//        PathPlannerTrajectory path2 = PathPlanner.loadPath("PickupCone2", 2.0, 3.0);
        
//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.1),
//             s_Swerve.followTrajectoryCommand(path5, true).andThen(new WaitCommand(0.5)),
//             new SetConeMode().andThen(new FloorIntakeCommandGroup()).alongWith(s_Swerve.followTrajectoryCommand(path2, false).withTimeout(8).andThen(new WaitCommand(1))),
//             new StoreStateCommandGroup().withTimeout(2)
//         );

//     }
    
// }