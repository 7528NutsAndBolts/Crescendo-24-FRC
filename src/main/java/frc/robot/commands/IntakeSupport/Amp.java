package frc.robot.commands.IntakeSupport;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;


public class Amp extends Command {
    
    private double desiredPosition = RobotContainer.intakesupport.getAmpPosition();

    public Amp() {
        addRequirements(RobotContainer.intakesupport);
    }

    public void initialize() {
        RobotContainer.intakesupport.setTargetPosition(desiredPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.intakesupport.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return RobotContainer.intakesupport.isInPosition(desiredPosition);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}

