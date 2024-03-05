package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb.GoToHomePosition;
import frc.robot.commands.Climb.JoystickClimb;
import frc.robot.commands.CommandGroups.GoToAmpCommandGroup;
import frc.robot.commands.CommandGroups.StoreStateCommandGroup;
import frc.robot.commands.Drivetrain.PIDTurnToAngle;
// import frc.robot.commands.Climb.ClimbSticks;
// import frc.robot.commands.Climb.GoToHomePosition;
// import frc.robot.commands.Climb.JoystickClimb;
// import frc.robot.commands.Drivetrain.PIDTurnToAngle;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Sweeper.UpSweeper;
// import frc.robot.commands.Elevator.ReadyStateCommandGroup;
import frc.robot.subsystems.*;
// import frc.robot.auto.AutonomousSelector;
// import frc.auto.AutonomousSelector;
// import frc.robot.auto.DefaultAuto;
import frc.robot.subsystems.Limelight.LimelightAmp;
import frc.robot.subsystems.Limelight.LimelightSource;
// import frc.robot.subsystems.Vision.VisionModule;
// import frc.robot.subsystems.Vision.VisionPipelineRunnable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Autonomous Selector */
    // private final AutonomousSelector autonomousSelector = new AutonomousSelector();
    
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
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton faceLeftButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton faceRightButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceRearButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton faceFrontButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Operator Buttons */
    private final JoystickButton storeClimbStateButton = new JoystickButton(operator, XboxController.Button.kBack.value);
    // private final JoystickButton robotCentric = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton runIntakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ampButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton defaultPositionButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public static Climb climb = new Climb();
    public static Intake intake = new Intake();
    public static IntakeSupport intakesupport = new IntakeSupport();
    public static Drake sweeper = new Drake();
    public static LimelightAmp limelightamp = new LimelightAmp();
    public static LimelightSource limelightsource = new LimelightSource();
 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                robotCentric
            )
        );

        climb.setDefaultCommand(new JoystickClimb());

        intake.setDefaultCommand(new StopIntake());

        sweeper.setDefaultCommand(new UpSweeper());
   

        /* Configure the button bindings */
        configureButtonBindings();

        // Thread visionThread = new Thread(new VisionPipelineRunnable(VisionModule.getInstance()), "visionThread");
        // visionThread.setDaemon(true);
        // visionThread.start();


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));


        faceLeftButton.whileTrue(new PIDTurnToAngle(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            270));

        faceRightButton.whileTrue(new PIDTurnToAngle(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            90));

        faceFrontButton.whileTrue(new PIDTurnToAngle(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            180));
                
        faceRearButton.whileTrue(new PIDTurnToAngle(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            360));
        



        /* Operator Buttons */
        storeClimbStateButton.onTrue(new GoToHomePosition());
        runIntakeButton.onTrue(new IntakeObject());
        ampButton.onTrue(new GoToAmpCommandGroup());
        defaultPositionButton.onTrue(new StoreStateCommandGroup());
        
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
        return stickDeadband(this.operator.getRawAxis(1), 0.05, 0.0);
    }
 
    public double getOperatorRightStickY() {
        return stickDeadband(this.operator.getRawAxis(5), 0.05, 0.0);
    }

    /* Runs the Autonomous Selector*/
    // public Command getAutonomousCommand() {
    //     return autonomousSelector.getCommand(swerve);
    //     // return AutonomousSelector.getCommand(swerve);
    // }
}
