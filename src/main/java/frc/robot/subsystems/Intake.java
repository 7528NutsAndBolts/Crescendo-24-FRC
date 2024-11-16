// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Intake extends SubsystemBase {

// 	public TalonFX IntakeFalcon = new TalonFX(10);
//     public TalonFX IntakeFalcon2 = new TalonFX(11);
//     public TalonFXConfiguration IntakeFXConfig = new TalonFXConfiguration();

// 	public Intake() {
//         /** Shooter Motor Configuration */
//         /* Motor Inverts and Neutral Mode */
//         this.IntakeFalcon2.setControl(new Follower(10, true));
// 		IntakeFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         IntakeFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         /* Current Limiting */
//         IntakeFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         IntakeFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
//         IntakeFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
//         IntakeFXConfig.CurrentLimits.SupplyTimeThreshold = 0.08;

//         /* PID Config */
//         IntakeFXConfig.Slot0.kP = 0.2;
//         IntakeFXConfig.Slot0.kI = 0;
//         IntakeFXConfig.Slot0.kD = 0;

//         /* Open and Closed Loop Ramping */
//         IntakeFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
//         IntakeFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

//         IntakeFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
//         IntakeFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

//         // Config Motor
//         IntakeFalcon.getConfigurator().apply(IntakeFXConfig);
//         IntakeFalcon.getConfigurator().setPosition(0.0);
//         IntakeFalcon2.getConfigurator().setPosition(0.0);

// 	}

// 	public void setSpeed(double speed) {
//         this.IntakeFalcon.set(speed);
//         this.IntakeFalcon2.set(-speed);
// 	}

// 	public double getCurrentDraw() {
// 		return this.IntakeFalcon.getSupplyCurrent().getValueAsDouble();
// 	}

// 	public void resetShooterEncoder() {
//         try {
// 			IntakeFalcon.getConfigurator().setPosition(0.0);
//         }
//         catch (Exception e) {
//             DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
//         }
// 	}

// 	public void updateDashboard() {
// 		SmartDashboard.putNumber("Intake Current", this.getCurrentDraw());
// 	}
// }