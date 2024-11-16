// package frc.robot.auto;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;

// public class AutoZeroReverse extends Command {

//     private Swerve s_swerve;

//     public AutoZeroReverse(Swerve s_swerve) {
//         this.s_swerve = s_swerve;
//         addRequirements(s_swerve);
//     }

//     public void initialize() {
//         s_swerve.autozeroGyro();
//     }

//     public void execute() {

//     }

//     public boolean isFinished() {
//         return true;   //originally true, changed on 3.22.24
//     }

//     protected void end() {
//     }

//     protected void interrupted() {
//         end();
//     }
// }
