package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StoreObject extends Command {

    private boolean isCone;

    public StoreObject() {
        addRequirements(RobotContainer.intake);
    }

    public void initialize() {
    }

    public void execute() {
        isCone = RobotContainer.candleSubsystem.getIsCone();
        if (isCone == true){
            RobotContainer.intake.holdCone();
        }

        else {
            RobotContainer.intake.holdCube();

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