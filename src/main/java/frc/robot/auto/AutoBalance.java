// package frc.robot.auto;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;

// public class AutoBalance extends Command {

//     private boolean isBalanced;
//     private Swerve s_swerve;

//     public AutoBalance(Swerve s_swerve) {
//         this.s_swerve = s_swerve;
//         addRequirements(s_swerve);
//     }

//     public void initialize() {
//         isBalanced = s_swerve.isRobotBalanced();
//     }

//     public void execute() {
//         isBalanced = s_swerve.isRobotBalanced();
//         if (isBalanced == false){
//             s_swerve.AutoBalance();
//         }

//         else {
//             s_swerve.stopDrive();
//         }
//     }

//     public boolean isFinished() {
//         return false;
//     }

//     protected void end() {
//         s_swerve.stopDrive();
//     }

//     protected void interrupted() {
//         end();
//     }
// }