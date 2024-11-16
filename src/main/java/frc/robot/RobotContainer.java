package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.AutoZero;
// import frc.robot.auto.AutoZeroReverse;
import frc.robot.auto.AutonomousSelector;
// import frc.robot.commands.Climb.ClimbPosition;
// import frc.robot.commands.Climb.GoToHomePosition;
// import frc.robot.commands.Climb.JoystickClimb;
import frc.robot.commands.Drivetrain.PIDTurnToAngle;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Intake.Adios;
import frc.robot.commands.Intake.Centre;
import frc.robot.commands.Intake.FullObject;
import frc.robot.commands.Intake.Hola;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Intake.OuttakeObject;
import frc.robot.commands.Intake.StopIntake;
// import frc.robot.subsystems.Climb;

// import frc.robot.commands.Climb.ClimbPosition;

import frc.robot.subsystems.*;
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
    
    /* Contr ollers */
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
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton holaButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton adiosButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton runIntakeButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton runOuttakeButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton runFullOutakeSpeedButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton centreButton = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    ///public static Climb climb = new Climb();
    public static OldIntake intake = new OldIntake();
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
   
       NamedCommands.registerCommand("Score in Amp", new FullObject());
       NamedCommands.registerCommand("Stop Intake", new StopIntake());
       NamedCommands.registerCommand("Auto Zero", new AutoZero(swerve));
    //    NamedCommands.registerCommand("Auto Zero Reverse", new AutoZeroReverse(swerve));

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

        // climb.setDefaultCommand(new JoystickClimb());

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));


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
            0));
        



        /* Operator Buttons */
        // storeClimbStateButton.onTrue(new GoToHomePosition());'

        //Climb buttons
        // climbButton.whileTrue(new ClimbPosition());
        // climbButton.whileFalse(new GoToHomePosition());

        runIntakeButton.whileTrue(new IntakeObject());
        runIntakeButton.whileFalse(new StopIntake());

        // defaultPositionButton.onTrue(new StoreStateCommandGroup());
        runOuttakeButton.whileTrue(new OuttakeObject());
        runOuttakeButton.whileFalse(new StopIntake());

        runFullOutakeSpeedButton.whileTrue(new FullObject());
        runFullOutakeSpeedButton.whileFalse(new StopIntake());

        holaButton.whileTrue(new Hola());
        holaButton.whileFalse(new StopIntake());

        adiosButton.whileTrue(new Adios());
        adiosButton.whileFalse(new StopIntake());

        centreButton.whileTrue(new Centre());
        centreButton.whileFalse(new StopIntake());
        
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
