package frc.robot.auto;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NoAuto extends SequentialCommandGroup {

    public NoAuto(Swerve s_Swerve){

        addCommands(
            new AutoZero(s_Swerve));

    }
}