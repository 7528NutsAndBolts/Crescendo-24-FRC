package frc.robot.commands.Sweeper;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class UpSweeper extends Command {

    public UpSweeper() {
        addRequirements(RobotContainer.sweeper);
    }

    public void initialize() {

    }

    public void execute() {
    RobotContainer.sweeper.getHomePosition();
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {

    }
    
    protected void interrupted() {

    }
}

