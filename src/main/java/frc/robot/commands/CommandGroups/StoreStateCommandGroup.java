package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeSupport.Source;
import frc.robot.commands.Sweeper.UpSweeper;
import frc.robot.commands.Climb.GoToHomePosition;

public class StoreStateCommandGroup extends SequentialCommandGroup {
    
    public StoreStateCommandGroup() {
        addCommands(
            new Source().withTimeout(2).alongWith(new GoToHomePosition()).alongWith(new UpSweeper()));
    }

}