package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public CTREConfigs(){
        //camera init
        // CameraServer.startAutomaticCapture();


        double tuningP = SmartDashboard.getNumber("Tuning P", 0);
        
        var slot0Configs = new Slot0Configs();

        /* Swerve Angle Motor Configurations */

        slot0Configs.kP = Constants.Swerve.angleKP;
        slot0Configs.kI = Constants.Swerve.angleKI;
        slot0Configs.kD = Constants.Swerve.angleKD;

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;

        /* Swerve Drive Motor Configuration */

        // swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        slot0Configs.kP = tuningP;
        slot0Configs.kI = Constants.Swerve.driveKI;
        slot0Configs.kD = Constants.Swerve.driveKD;
        slot0Configs.kV = Constants.Swerve.driveKF;      
          
       swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
       swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

       swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
       swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */

        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
    
    }
}