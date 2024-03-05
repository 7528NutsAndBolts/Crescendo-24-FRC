package frc.robot.commands.CommandGroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeSupport.Amp;
import frc.robot.commands.Climb.GoToHomePosition;
import frc.robot.commands.Intake.*;

public class GoToAmpCommandGroup extends SequentialCommandGroup {
    
    public GoToAmpCommandGroup() {
        addCommands(
            new Amp().withTimeout(2).andThen(new WaitCommand(0.5)).andThen(new OuttakeObject().withTimeout(1)).andThen(new GoToHomePosition()));
    }

}

