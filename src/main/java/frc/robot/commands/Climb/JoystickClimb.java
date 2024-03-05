package frc.robot.commands.Climb;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.Command;

public class JoystickClimb extends Command {

	private int positionIncrement = 5;

	public JoystickClimb() {
		addRequirements(RobotContainer.climb);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		// joystick control
		double climbSignal = -RobotContainer.climb.JoyStickClimb();
        
		RobotContainer.climb.incrementTargetPosition((double) (climbSignal * positionIncrement));

		RobotContainer.climb.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
