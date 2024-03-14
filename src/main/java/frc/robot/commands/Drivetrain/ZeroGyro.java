package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroGyro extends Command {

    private Swerve swerve;

    public ZeroGyro() {
       addRequirements(swerve);
    }

    public void initialize() {
        swerve.zeroGyro();
    }

    public void execute() {
    
    }

    public boolean isFinished() {
        return true;
    }

    protected void end() {
    }
    
    protected void interrupted() {
        end();
    }
}
