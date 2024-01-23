package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoDebug extends Command {

    public AutoDebug() {
    }

    public void initialize() {
    }

    public void execute() {
        System.out.println("Auto debug occured");

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