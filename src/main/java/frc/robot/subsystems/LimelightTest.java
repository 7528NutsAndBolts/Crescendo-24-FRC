package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class LimelightTest extends TimedRobot {
    private Swerve swerve;

    private PIDController turnController = new PIDController(0.05, 0, 0);
    private PIDController strafeController = new PIDController(0.05, 0, 0);
    private PIDController forwardController = new PIDController(0.1, 0, 0);

    // Limelight network table
    private NetworkTable limelight;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

    }

    @Override
    public void teleopPeriodic() {
        // Read limelight values
        double tx = limelight.getEntry("tx").getDouble(0.0); // Horizontal offset
        double ty = limelight.getEntry("ty").getDouble(0.0); // Vertical offset
        double ta = limelight.getEntry("ta").getDouble(0.0); // Target area
        double tv = limelight.getEntry("tv").getDouble(0.0); // target valid (0 or 1)

        // Log data to smartdashboard
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("TargetValid", tv);

        // check if a valid target is detected
        if (tv == 1) {
            // PID-based adjustments
            double turnAdjustment = turnController.calculate(tx, 0); // rotate to center of target horizontally
            double strafeAdjustment = strafeController.calculate(tx, 0); // strafe left/right to align with target
            double forwardAdjustment = forwardController.calculate(ty, 0); // move forward/backward to approach target
        }

    }
}
