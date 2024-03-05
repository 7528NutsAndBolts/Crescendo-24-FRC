package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlignAmp extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean robotCentricSup;

    private double tx = 0;
    private double ty = 0;

    private final PIDController angleController = new PIDController(0.012, 0.2, 0.0002);
    private double targetAngle = 0;


    public VisionAlignAmp(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.limelightamp);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.limelightamp.getX();
        ty = RobotContainer.limelightamp.getY();
        angleController.setTolerance(3);  // needs to be tuned
    }
    
    @Override
    public void execute() {
        //TODO: I'm not sure if I disabled the elevator consideration here correctly.
        //elevatorHeight = RobotContainer.elevator.getCurrentPosition();

        // find target location
        tx = RobotContainer.limelightamp.getX();
        ty = RobotContainer.limelightamp.getY();
        
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        double rotationVal = angleController.calculate(tx,targetAngle);

        /*if (elevatorHeight >= 30000) {
            translationVal = translationVal * slowSpeed;
            strafeVal = strafeVal * slowSpeed;
            rotationVal = rotationVal * slowSpeed;
        }

        else if (elevatorHeight > 5000 && elevatorHeight < 29999) {
            translationVal = translationVal * midSpeed;
            strafeVal = strafeVal * midSpeed;
            rotationVal = rotationVal * midSpeed;
        }

        else {}*/

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );
    }
}