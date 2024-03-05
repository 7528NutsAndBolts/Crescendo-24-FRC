package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class StopIntake extends Command {

    public StopIntake() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {

    }

    public void execute() {
    RobotContainer.intake.hold();
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {

    }
    
    protected void interrupted() {

    }
}
