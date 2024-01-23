package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeObject extends Command {

    private boolean isCone;

    public IntakeObject() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {
        isCone = RobotContainer.candleSubsystem.getIsCone();
    }

    public void execute() {

        if (isCone == true){
            RobotContainer.intake.intakeCone();
        }

        else {
            RobotContainer.intake.intakeCube();
        }
    }

    public boolean isFinished() {
        return true;
    }

    protected void end() {
        if (isCone == true){
            RobotContainer.intake.holdCone();
        }

        else {
            RobotContainer.intake.holdCube();;
        }
    }

    protected void interrupted() {
        end();
    }
}