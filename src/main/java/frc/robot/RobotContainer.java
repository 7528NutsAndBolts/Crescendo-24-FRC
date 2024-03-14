package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.AutonomousSelector;
// import frc.robot.commands.Climb.GoToHomePosition;
// import frc.robot.commands.Climb.JoystickClimb;
// import frc.robot.commands.CommandGroups.GoToAmpCommandGroup;
// import frc.robot.commands.CommandGroups.StoreStateCommandGroup;
import frc.robot.commands.Drivetrain.PIDTurnToAngle;
// import frc.robot.commands.Climb.ClimbSticks;
// import frc.robot.commands.Climb.GoToHomePosition;
// import frc.robot.commands.Climb.JoystickClimb;
// import frc.robot.commands.Drivetrain.PIDTurnToAngle;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Intake.OuttakeObject;
import frc.robot.commands.Intake.StopIntake;
// import frc.robot.commands.IntakeSupport.Source;
// import frc.robot.commands.Sweeper.UpSweeper;
import frc.robot.subsystems.*;
// import frc.robot.auto.AutonomousSelector;
// import frc.auto.AutonomousSelector;
// import frc.robot.auto.DefaultAuto;
import frc.robot.subsystems.Limelight.LimelightAmp;
import frc.robot.subsystems.Limelight.LimelightSource;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Autonomous Selector */
    private final AutonomousSelector autonomousSelector = new AutonomousSelector();
    
    /* Controllers */
    private final Joystick driver = new Joystick(1);
    private final Joystick operator = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

   
    /* Setting Bot to Field Centric */
    private final Boolean robotCentric = false;

    /* Driver Buttons */
    public final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton faceLeftButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton faceRightButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceRearButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton faceFrontButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Operator Buttons */
    // private final JoystickButton storeClimbStateButton = new JoystickButton(operator, XboxController.Button.kBack.value);
    // private final JoystickButton robotCentric = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton runIntakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton ampButton = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton defaultPositionButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton runOuttakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // private final JoystickButton runStopIntakeButton = new JoystickButton(operator, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    // public static Climb climb = new Climb();
    public static Intake intake = new Intake();
    // public static IntakeSupport intakesupport = new IntakeSupport();
    // public static Drake sweeper = new Drake();
    public static LimelightAmp limelightamp = new LimelightAmp();
    public static LimelightSource limelightsource = new LimelightSource();
 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                robotCentric
            )
        );

        // climb.setDefaultCommand(new JoystickClimb());

        // intake.setDefaultCommand(new StopIntake());

        // sweeper.setDefaultCommand(new UpSweeper());
   
       NamedCommands.registerCommand("Score in Amp", new OuttakeObject());
    //    NamedCommands.registerCommand("Zero Gyro", new ZeroGyro());

        /* Configure the button bindings */
        configureButtonBindings();

    //    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    //   SmartDashboard.putData("Auto Mode", autoChooser);

   


        // AutoBuilder.followPath(path).schedule();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // SmartDashboard.putData("Test Auto 2.10", new PathPlannerAuto("Test Auto 2.10"));

    //        SmartDashboard.putData("Pathfind to Score in Amp", AutoBuilder.pathfindToPose(
    //   new Pose2d(0.77, 6.90, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     3.0, 3.0, 
    //     Units.degreesToRadians(540), Units.degreesToRadians(720)
    //   ), 
    //   0, 
    //   2.0
    // ));
    // SmartDashboard.putData("Pathfind to Leave Starting Area", AutoBuilder.pathfindToPose(
    //   new Pose2d(1.84, 7.62, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     3.0, 3.0, 
    //     Units.degreesToRadians(540), Units.degreesToRadians(720)
    //   ), 
    //   0, 
    //   0
    // ));


        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));


        faceLeftButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            270));

        faceRightButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            90));

        faceFrontButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            180));
                
        faceRearButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            360));
        



        /* Operator Buttons */
        // storeClimbStateButton.onTrue(new GoToHomePosition());
        runIntakeButton.whileTrue(new IntakeObject());
        runIntakeButton.whileFalse(new StopIntake());
        // ampButton.onTrue(new GoToAmpCommandGroup());
        // sourceButton.onTrue(new Source());
        // defaultPositionButton.onTrue(new StoreStateCommandGroup());
        runOuttakeButton.whileTrue(new OuttakeObject());
        runOuttakeButton.whileFalse(new StopIntake());
        
        
    }

    /* Public access to joystick values */
    public Joystick getDriver() {
        return driver;
    }

    public Joystick getOperator() {
        return operator;
    }

    /* Sets Joystick Deadband */
    public static double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }
    
    /* Passes Along Joystick Values for Elevator and Wrist */
    public double getOperatorLeftStickY() {
        return stickDeadband(this.operator.getRawAxis(1), 0.2, 0.0);
    }
 
    public double getOperatorRightStickY() {
        return stickDeadband(this.operator.getRawAxis(5), 0.2, 0.0);
    }

    /* Runs the Autonomous Selector*/
    public Command getAutonomousCommand() {
        return autonomousSelector.getCommand(swerve);
    }
}
