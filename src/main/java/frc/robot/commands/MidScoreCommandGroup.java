package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.GoToMidPosition;
import frc.robot.commands.Intake.ScoreObject;
import frc.robot.commands.Wrist.GoToCentralPosition;
import frc.robot.commands.Wrist.GoToMidWristPosition;

public class MidScoreCommandGroup extends SequentialCommandGroup {
    
    public MidScoreCommandGroup() {
        addCommands(new GoToCentralPosition().withTimeout(2).andThen(new GoToMidPosition()).withTimeout(3).andThen(new GoToMidWristPosition()).withTimeout(2).andThen(new ScoreObject()).withTimeout(5).andThen(new GoToCentralPosition()));
    }

}
