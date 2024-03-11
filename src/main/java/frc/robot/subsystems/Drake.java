// package frc.robot.subsystems;

// import frc.lib.models.*;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Drake extends SubsystemBase implements IPositionControlledSubsystem {

//   private boolean isHoldingPosition = false;

//     private double homePosition = 0;
//     private double maxUpTravelPosition = 15002;

//     private double sweepPosition = 15000;

//     public double upPositionLimit = maxUpTravelPosition;
//     public double downPositionLimit = 0;
//     private double targetPosition = 0;
    
// 	private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);

// 	private double feedForward = 0.0;

//     private final static double onTargetThreshold = 1; //might wanna lower this down

// 	public TalonFXConfiguration DrakeConfiguration = new TalonFXConfiguration();

//     private TalonFX Sweeper = new TalonFX(14);

//     public Drake() {

//         Sweeper.clearStickyFaults();

// 		DrakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
// 		DrakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

// 		DrakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
// 		DrakeConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
// 		DrakeConfiguration.CurrentLimits.SupplyCurrentThreshold = 40;
// 		DrakeConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;

// 		/* Pid Config */
// 		DrakeConfiguration.Slot0.kP = 0.0;
// 		DrakeConfiguration.Slot0.kI = 0;
// 		DrakeConfiguration.Slot0.kD = 0;

// 		DrakeConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
// 		DrakeConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

// 		DrakeConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
// 		DrakeConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

// 		/* Config Acceleration & Velocity */
// 		DrakeConfiguration.MotionMagic.withMotionMagicAcceleration(0);
// 		DrakeConfiguration.MotionMagic.withMotionMagicCruiseVelocity(200);

// 		/* Config Motors */
// 		Sweeper.getConfigurator().apply(DrakeConfiguration);
// 		Sweeper.getConfigurator().setPosition(0.0);
//     }

// 	public void motionMagicControl() {
// 		this.manageMotion(targetPosition);
// 	targetPositionDutyCycle.withPosition(targetPosition);
// 	targetPositionDutyCycle.withFeedForward(feedForward);
// 		this.Sweeper.setControl(targetPositionDutyCycle);
// 	}

// 	public double getCurrentPosition() {
// 		return this.Sweeper.getRotorPosition().getValueAsDouble();
// 	}

// 	public double getCurrentDraw() {
// 		return this.Sweeper.getSupplyCurrent().getValueAsDouble();
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
// 			}
// 		}

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

//     public double getSweepPosition() {
//         return this.sweepPosition;
//     }

// 	public double getFeedForward() {
// 		return this.feedForward;
// 	}

// 	public void resetWristEncoder() {
//         try {
// 			Sweeper.getConfigurator().setPosition(0.0);
//         }
//         catch (Exception e) {
//             DriverStation.reportError("Sweeper.resetSweeperEncoders exception.  You're Screwed! : " + e.toString(), false);
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

// // 	public void updateDashboard() {
// // 		SmartDashboard.putNumber("Sweeper Position", this.getCurrentPosition());
// // 		SmartDashboard.putNumber("Sweeper Target Position", this.getTargetPosition());
// // 		SmartDashboard.putNumber("Sweeper Position Error", this.getPositionError());
// // 		SmartDashboard.putNumber("Sweeper Velocity", this.getCurrentVelocity());
// // 		SmartDashboard.putNumber("Sweeper Current", this.getCurrentDraw());
// // 	}

// 	@Override
// 	public double getCurrentVelocity() {
// 		double currentVelocity = this.Sweeper.getVelocity().getValueAsDouble();
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
