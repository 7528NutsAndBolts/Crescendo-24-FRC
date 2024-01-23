package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.GoToDoubleHPElevatorPosition;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Wrist.GoToCentralPosition;
import frc.robot.commands.Wrist.GoToValConePosition;

public class ValConeCommandGroup extends SequentialCommandGroup {
    
    public ValConeCommandGroup() {
        addCommands(new GoToCentralPosition().withTimeout(2).andThen(new GoToDoubleHPElevatorPosition()).withTimeout(0.5).andThen(new GoToValConePosition()).withTimeout(0.9), new IntakeObject());
    }

}
