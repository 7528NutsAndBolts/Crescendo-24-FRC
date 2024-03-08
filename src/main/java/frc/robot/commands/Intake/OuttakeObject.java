package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeObject extends Command {

    public OuttakeObject() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {

    }

    public void execute() {
    RobotContainer.intake.outtake();
    }

    public boolean isFinished() {
        return true;
    }

    protected void end() {
    RobotContainer.intake.stopIntake();
    }
    
    protected void interrupted() {
        end();
    }
}
