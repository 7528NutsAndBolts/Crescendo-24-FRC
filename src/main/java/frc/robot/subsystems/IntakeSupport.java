// package frc.robot.subsystems;

// import frc.lib.models.*;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class IntakeSupport extends SubsystemBase implements IPositionControlledSubsystem {

// 	private boolean isHoldingPosition = false;

//     // Set Different Heights
// 	private double homePosition = 0;
// 	private double maxUpTravelPosition = 15002;

//     private double ampPosition = 15000;

// 	public double upPositionLimit = maxUpTravelPosition;
// 	public double downPositionLimit = 0;
// 	private double targetPosition = 0;
//     private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
// 	private double feedForward = 0.0;

// 	private final static double onTargetThreshold = 1;
		
// 	private TalonFX intakeWrist1 = new TalonFX(12);
// 	private TalonFX intakeWrist2 = new TalonFX(13);

//     private TalonFXConfiguration intakeWristConfig = new TalonFXConfiguration();
	

// 	public IntakeSupport() {
// 		// Clear Sticky Faults
// 		this.intakeWrist1.clearStickyFaults();
// 		this.intakeWrist2.clearStickyFaults();
		
//         // Set Followers
// 		this.intakeWrist2.setControl(new Follower(12, true));

//         /** intake Motor Configuration */
//         /* Motor Inverts and Neutral Mode */
// 		intakeWristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//         intakeWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         /* Current Limiting */
//         intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         intakeWristConfig.CurrentLimits.SupplyCurrentLimit = 30;
//         intakeWristConfig.CurrentLimits.SupplyCurrentThreshold = 40;
//         intakeWristConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

//         /* PID Config */
//         intakeWristConfig.Slot0.kP = 0.05;
//         intakeWristConfig.Slot0.kI = 0;
//         intakeWristConfig.Slot0.kD = 0;

//         /* Open and Closed Loop Ramping */
//         intakeWristConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
//         intakeWristConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

//         intakeWristConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
//         intakeWristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

//         //Config Acceleration and Velocity
//         intakeWristConfig.MotionMagic.withMotionMagicAcceleration(0);
//         intakeWristConfig.MotionMagic.withMotionMagicCruiseVelocity(200);

//         // Config Motor
//         intakeWrist1.getConfigurator().apply(intakeWristConfig);
//         intakeWrist1.getConfigurator().setPosition(0.0);
// 		intakeWrist2.getConfigurator().setPosition(0);
// 	}

// 	public void motionMagicControl() {
// 		this.manageMotion(targetPosition);
//         targetPositionDutyCycle.withPosition(targetPosition);
//         targetPositionDutyCycle.withFeedForward(feedForward);
// 		this.intakeWrist1.setControl(targetPositionDutyCycle);
// 	}

// 	public double getCurrentPosition() {
// 		return this.intakeWrist1.getRotorPosition().getValueAsDouble();
// 	}

// 	public double getCurrentDraw() {
// 		return this.intakeWrist1.getSupplyCurrent().getValueAsDouble();
// 	}

// 	public boolean isHoldingPosition() {
// 		return this.isHoldingPosition;
// 	}

// 	public void setIsHoldingPosition(boolean isHoldingPosition) {
// 		this.isHoldingPosition = isHoldingPosition;
// 	}

// 	public double getTargetPosition() {
// 		return this.targetPosition;
// 	}

// 	public boolean setTargetPosition(double position) {
// 		if (!isValidPosition(position)) {
// 			return false;
// 		} else {
// 			this.targetPosition = position;
// 			return true;
// 		}
// 	}

// 	public void forceSetTargetPosition(double position) {
// 		this.targetPosition = position;
// 	}

// 	public void incrementTargetPosition(double increment) {
// 		double currentTargetPosition = this.targetPosition;
// 		double newTargetPosition = currentTargetPosition + increment;
// 		if (isValidPosition(newTargetPosition)) {		// && isWristSafe(newTargetPosition) check for other subsystems
// 			this.targetPosition = newTargetPosition;
// 		}
// 	}

// 	public boolean isValidPosition(double position) {
// 		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
// 		return withinBounds;
// 	}

//     // communicate with commands
// 	public double getHomePosition() {
// 		return this.homePosition;
// 	}

// 	public double getMaxUpTravelPosition() {
// 		return this.maxUpTravelPosition;
// 	}

//     public double getAmpPosition() {
//         return this.ampPosition;
//     }

// 	public double getFeedForward() {
// 		return this.feedForward;
// 	}

// 	public void resetWristEncoder() {
//         try {
// 			intakeWrist1.getConfigurator().setPosition(0.0);
// 			intakeWrist2.getConfigurator().setPosition(0);
//         }
//         catch (Exception e) {
//             DriverStation.reportError("Wrist.resetWristEncoders exception.  You're Screwed! : " + e.toString(), false);
//         }
// 	}

// 	public double getPositionError() {
// 		double currentPosition = this.getCurrentPosition();
// 		double targetPosition = this.getTargetPosition();
// 		double positionError = Math.abs(currentPosition - targetPosition);
// 		return positionError;
// 	}

// 	public void manageMotion(double targetPosition) {
// 		double currentPosition = getCurrentPosition();
// 		if (currentPosition < targetPosition) {
// 				// set based on gravity
// 		}
// 		else {
// 				//set based on gravity
// 		}
// 	}

// 	public void zeroTarget() {
// 		targetPosition = 0;
// 	}

// 	public void updateDashboard() {
// 		SmartDashboard.putNumber("Intake Wrist Position", this.getCurrentPosition());
// 		SmartDashboard.putNumber("Intake Wrist Target Position", this.getTargetPosition());
// 		SmartDashboard.putNumber("Intake Wrist Position Error", this.getPositionError());
// 		SmartDashboard.putNumber("Intake Wrist Velocity", this.getCurrentVelocity());
// 		SmartDashboard.putNumber("Intake Wrist Current", this.getCurrentDraw());
// 	}

// 	@Override
// 	public double getCurrentVelocity() {
// 		double currentVelocity = this.intakeWrist1.getVelocity().getValueAsDouble();
// 		return currentVelocity;
// 	}

// 	@Override
// 	public boolean isInPosition(double targetPosition) {
// 		double currentPosition = this.getCurrentPosition();
// 		double positionError = Math.abs(currentPosition - targetPosition);
// 		if (positionError < onTargetThreshold) {
// 			return true;
// 		} else {
// 			return false;
// 		}
// 	}
// }   