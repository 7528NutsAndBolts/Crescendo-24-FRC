// package frc.robot.auto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.DoubleHPCommandGroup;
// import frc.robot.commands.Elevator.GoToDoubleHPElevatorPosition;
// import frc.robot.commands.Intake.ScoreObject;
// import frc.robot.commands.Intake.StoreObject;
// import frc.robot.commands.LEDs.SetConeMode;
// import frc.robot.commands.Wrist.GoToCentralPosition;
// import frc.robot.commands.Wrist.GoToDoubleHPWristPosition;
// import frc.robot.subsystems.Swerve;


// public class AutoScoreHighCone extends SequentialCommandGroup {
    
//     public AutoScoreHighCone(Swerve s_sSwerve) {
//         addCommands(
//             new SetConeMode(),
//             // new DoubleHPCommandGroup().withTimeout(5).andThen(new WaitCommand(1)),
//             new GoToCentralPosition().withTimeout(0.5),
//             new GoToDoubleHPElevatorPosition().withTimeout(1),
//             new GoToDoubleHPWristPosition().withTimeout(1).andThen(new ScoreObject().withTimeout(3)).andThen(new WaitCommand(1)),
//             new StoreObject().withTimeout(1),
//             new GoToCentralPosition().withTimeout(1)
//         );
//     }
// }
