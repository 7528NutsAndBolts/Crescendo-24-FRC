package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.GoToStorePosition;
import frc.robot.commands.Intake.StoreObject;
import frc.robot.commands.Wrist.GoToCentralPosition;
import frc.robot.commands.Wrist.GoToStoreWristPosition;
public class StoreStateCommandGroup extends SequentialCommandGroup {
    
    public StoreStateCommandGroup() {
        addCommands(new GoToCentralPosition().withTimeout(2).alongWith(new GoToStorePosition()).andThen(new GoToStoreWristPosition().andThen(new StoreObject())));
    }
}
