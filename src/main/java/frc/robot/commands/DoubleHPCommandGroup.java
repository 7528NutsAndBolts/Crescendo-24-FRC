package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.GoToDoubleHPElevatorPosition;
import frc.robot.commands.Intake.ScoreObject;
import frc.robot.commands.Intake.StoreObject;
import frc.robot.commands.Wrist.GoToCentralPosition;
import frc.robot.commands.Wrist.GoToDoubleHPWristPosition;

public class DoubleHPCommandGroup extends SequentialCommandGroup {
    
    public DoubleHPCommandGroup() {
        addCommands(
            new GoToCentralPosition().withTimeout(2.5).andThen(new GoToDoubleHPElevatorPosition()).withTimeout(5).andThen(new GoToDoubleHPWristPosition()).withTimeout(10).andThen(new ScoreObject()).withTimeout(2.5).andThen(new WaitCommand(2)).andThen(new StoreObject()).withTimeout(2.5));
    }

}
