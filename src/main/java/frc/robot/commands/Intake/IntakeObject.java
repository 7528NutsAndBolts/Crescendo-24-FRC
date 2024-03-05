package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeObject extends Command {

    public IntakeObject() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {

    }

    public void execute() {
    RobotContainer.intake.intake();
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {

    }
    
    protected void interrupted() {

    }
}
