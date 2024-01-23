// package frc.robot.auto;

// import frc.robot.commands.LEDs.SetConeMode;
// import frc.robot.subsystems.Swerve;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class LeaveComFar extends SequentialCommandGroup {

//     public LeaveComFar(Swerve s_Swerve){

//         addCommands(
//             new AutoZero(s_Swerve).withTimeout(0.3),
//             new SetConeMode(),
//             new AutoScoreHighCone(s_Swerve),
//             new AutoHomeState().andThen(new WaitCommand(1)),
//             new FARLeaveCommunity(s_Swerve))

//         ;
//     }

// }