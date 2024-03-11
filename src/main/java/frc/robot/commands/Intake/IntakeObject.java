package frc.robot.commands.Intake;

import frc.robot.Robot;
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
