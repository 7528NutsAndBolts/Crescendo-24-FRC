package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Robot;
// import frc.robot.commands.Climb.ClimbSticks;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase implements IPositionControlledSubsystem {

    private boolean isHoldingPosition = false;

    private double homePosition = 0;
    private double maxUpTravelPosition = 60000;

    public final static int Climb_UP = 0;
    public final static int Climb_DOWN = 1;

    public double upPositionLimit = maxUpTravelPosition;
    public double downPositionLimit = 0;
    private double targetPosition = 0;
    
	private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);

	private double feedForward = 0.0;

    private final static double onTargetThreshold = 1; //might wanna lower this down

	public TalonFXConfiguration ClimbConfiguration = new TalonFXConfiguration();

    private TalonFX Climber = new TalonFX(9);

    public Climb() {

        Climber.clearStickyFaults();

		ClimbConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		ClimbConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		ClimbConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
		ClimbConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
		ClimbConfiguration.CurrentLimits.SupplyCurrentThreshold = 40;
		ClimbConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;

		/* Pid Config */
		ClimbConfiguration.Slot0.kP = 0.0;
		ClimbConfiguration.Slot0.kI = 0;
		ClimbConfiguration.Slot0.kD = 0;

		ClimbConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
		ClimbConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

		ClimbConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
		ClimbConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

		/* Config Acceleration & Velocity */
		ClimbConfiguration.MotionMagic.withMotionMagicAcceleration(0);
		ClimbConfiguration.MotionMagic.withMotionMagicCruiseVelocity(200);

		/* Config Motors */
		Climber.getConfigurator().apply(ClimbConfiguration);
		Climber.getConfigurator().setPosition(0.0);
    }

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
	targetPositionDutyCycle.withPosition(targetPosition);
	targetPositionDutyCycle.withFeedForward(feedForward);
		this.Climber.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.Climber.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.Climber.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
			}
		}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isWristSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetWristEncoder() {
        try {
			Climber.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Wrist.resetWristEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoyStickClimb(){
		double value = 0;
		value = Robot.robotContainer.getOperatorLeftStickY();
		return value;
	}
	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Climber Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Climber Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Climber Position Error", this.getPositionError());
		SmartDashboard.putNumber("Climber Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Climber Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.Climber.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
