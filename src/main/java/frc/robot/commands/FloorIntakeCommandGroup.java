package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.GoToSingleHPElevatorPosition;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Wrist.GoToFloorIntakePosition;

public class FloorIntakeCommandGroup extends SequentialCommandGroup {
    
    public FloorIntakeCommandGroup() {
        addCommands(new GoToSingleHPElevatorPosition().andThen(new WaitCommand(0.1).andThen(new GoToFloorIntakePosition().withTimeout(2)), new IntakeObject()));
    }
}
