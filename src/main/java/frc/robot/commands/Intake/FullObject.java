package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class FullObject extends Command {

    public FullObject() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {

    }

    public void execute() {
    RobotContainer.intake.fullOut();
    }

    public boolean isFinished() {
        // end();
        return true;
    }

    protected void end() {
        RobotContainer.intake.stopIntake();
    }
    
    protected void interrupted() {
        end();
    }
}
