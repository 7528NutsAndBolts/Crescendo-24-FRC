package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoZero extends Command {

    private Swerve s_swerve;

    public AutoZero(Swerve s_swerve) {
        this.s_swerve = s_swerve;
        addRequirements(s_swerve);
    }

    public void initialize() {
        s_swerve.zeroHeading();
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
