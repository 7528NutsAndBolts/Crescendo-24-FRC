package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean robotCentricSup;
    // private double slowSpeed = 0.2;
    // private double midSpeed = 0.75; //originally 0.5
    // private double elevatorHeight = 0;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        SmartDashboard.putNumber("translationjoystickinput", translationVal);
        // if (elevatorHeight >= 30000) {
        //     translationVal = translationVal * slowSpeed;
        //     strafeVal = strafeVal * slowSpeed;
        //     rotationVal = rotationVal * slowSpeed;
        // }

        // else if (elevatorHeight > 5000 && elevatorHeight < 29999) {
        //     translationVal = translationVal * midSpeed;
        //     strafeVal = strafeVal * midSpeed;
        //     rotationVal = rotationVal * midSpeed;
        // }

        // else {}

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );
    }
}