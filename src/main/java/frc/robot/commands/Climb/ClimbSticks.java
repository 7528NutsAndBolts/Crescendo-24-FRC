package frc.robot.commands.Climb;

import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimbSticks extends Command {    
    private Climb climb;
    private DoubleSupplier LeftClimbSup;
    private DoubleSupplier RightClimbSup;
    private double slowSpeed = 0.2;
    private double midSpeed = 0.75; //originally 0.5
    private double elevatorHeight = 0;

    public ClimbSticks(DoubleSupplier LeftClimbSup, DoubleSupplier RightClimbSup) {
       this.climb = climb;
       addRequirements(climb);

        this.LeftClimbSup = LeftClimbSup;
        this.RightClimbSup = RightClimbSup;
        
    }

        //TODO Auto-generated constructor stub
    

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        // double LeftClimber = MathUtil.applyDeadband(LeftClimbSup.getAsDouble(), Constants.stickDeadband);
        // double RightClimber = MathUtil.applyDeadband(RightClimbSup.getAsDouble(), Constants.stickDeadband);

        // if (elevatorHeight >= 30000) {
        //     LeftClimber = LeftClimber * slowSpeed;
        //     RightClimber = RightClimber * slowSpeed;
        // }

        // else if (elevatorHeight > 5000 && elevatorHeight < 29999) {
        //     LeftClimber = LeftClimber * midSpeed;
        //     RightClimber = RightClimber * midSpeed;
        // }

        // else {}

      
    }
}