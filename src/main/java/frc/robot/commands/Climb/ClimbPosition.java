// package frc.robot.commands.Climb;

// import frc.robot.RobotContainer;
// // import frc.robot.subsystems.Climb;
// import edu.wpi.first.wpilibj2.command.Command;

// public class ClimbPosition extends Command {
//     private double climbPosition = RobotContainer.climb.getClimbPosition();

//     public ClimbPosition() {
//         addRequirements(RobotContainer.climb);   
//     }

//     public void initialize() {
//         RobotContainer.climb.setTargetPosition(climbPosition);
//     }
    


//     //periodic equivalent/20ms
//     public void execute() {
//         RobotContainer.climb.motionMagicControl();
//     }

//     public boolean isFinished() {
//         return RobotContainer.climb.isInPosition(climbPosition);
//     }

//     protected void end() {
//     }
    
//     protected void interrupted() {
        
//     }
// }
