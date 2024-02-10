package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Robot;
// import frc.robot.commands.Climb.ClimbSticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase implements IPositionControlledSubsystem {

    private boolean isHoldingPosition = false;

    private int homePosition = 0;
    private int maxUpTravelPosition = 60000;

    public final static int Climb_UP = 0;
    public final static int Climb_DOWN = 1;

    public int upPositionLimit = maxUpTravelPosition;
    public int downPositionLimit = -1;
    private int targetPosition = 0;
    private double arbitraryFeedForward = 0.0;

    private final static int onTargetThreshold = 2000; //might wanna lower this down

    private final FXGains upGains = new FXGains(Climb_UP, 0.08, 0, 0.5, 0.01, 2000);
    private final FXGains downGains = new FXGains(Climb_DOWN, 0.04, 0, 0.1, 0.011, 2000);

    private MotionParameters highGearUpMotionParameters = new MotionParameters(18000, 12000, upGains);
    private MotionParameters highGearDownMotionParameters = new MotionParameters(18000, 12000, downGains);

    private double peakOutputReverse = -0.1;

    private CustomTalonFX leftClimber = new CustomTalonFX(9);
    private CustomTalonFX rightClimber = new CustomTalonFX(10);

    public Climb() {
        leftClimber.configFactoryDefault();
        leftClimber.clearStickyFaults();
        rightClimber.configFactoryDefault();
        rightClimber.clearStickyFaults();

        this.leftClimber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.rightClimber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        this.leftClimber.configForwardSoftLimitEnable(true);
        this.leftClimber.configForwardSoftLimitThreshold(upPositionLimit);
        this.rightClimber.configForwardSoftLimitEnable(true);
        this.rightClimber.configForwardSoftLimitThreshold(upPositionLimit);

        this.leftClimber.configReverseSoftLimitEnable(true);
        this.leftClimber.configReverseSoftLimitThreshold(downPositionLimit);
        this.rightClimber.configReverseSoftLimitEnable(true);
        this.rightClimber.configReverseSoftLimitThreshold(downPositionLimit);

        this.leftClimber.setInverted(false);
        this.leftClimber.setSensorPhase(false);
        this.rightClimber.setInverted(false);
        this.rightClimber.setSensorPhase(false);

        this.leftClimber.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
        this.leftClimber.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 150);
        this.rightClimber.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
        this.rightClimber.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,150);

        this.leftClimber.configMotionParameters(highGearUpMotionParameters);
        this.leftClimber.configMotionParameters(highGearDownMotionParameters);
        this.rightClimber.configMotionParameters(highGearDownMotionParameters);
        this.rightClimber.configMotionParameters(highGearUpMotionParameters);

        this.leftClimber.setNeutralMode(NeutralMode.Brake);
        this.leftClimber.configClosedloopRamp(0.25);
        this.rightClimber.setNeutralMode(NeutralMode.Brake);
        this.rightClimber.configClosedloopRamp(0.25);

        this.leftClimber.configVoltageCompSaturation(12);
        this.leftClimber.enableVoltageCompensation(true);
        this.rightClimber.configVoltageCompSaturation(12);
        this.rightClimber.enableVoltageCompensation(true);

        this.leftClimber.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 65, 70, 0.2));
        this.rightClimber.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 65, 70, 0.2));
        
        this.leftClimber.configPeakOutputReverse(peakOutputReverse);
        this.rightClimber.configPeakOutputReverse(peakOutputReverse);

        this.resetClimbEncoder();
    }

    public void setClimb(ControlMode controlMode, double signal) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		leftClimber.set(controlMode, signal);
	    rightClimber.set(controlMode, signal);
	}

	public void setClimb(ControlMode controlMode, double signal, DemandType demandType, double demand) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		leftClimber.set(controlMode, signal, demandType, demand);
		rightClimber.set(controlMode, signal, demandType, demand);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
		this.leftClimber.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
		this.rightClimber.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

	public double getCurrentPosition() {
		return this.leftClimber.getSelectedSensorPosition();
	}

	public double getCurrentDraw() {
		return this.leftClimber.getSupplyCurrent();
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

	public boolean setTargetPosition(int position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(int position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(int increment) {
		int currentTargetPosition = this.targetPosition;
		int newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isWristSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(int position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public int getHomePosition() {
		return this.homePosition;
	}

	public int getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	public void resetClimbEncoder() {
        try {
			leftClimber.setSelectedSensorPosition(0, 0, 30);
        }
        catch (Exception e) {
            DriverStation.reportError("Elevator.resetElevatorEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoyStickClimb(){
		double value = 0;
		value = Robot.m_robotContainer.getOperatorLeftStickY();
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
				leftClimber.selectMotionParameters(highGearUpMotionParameters);
		}
		else {
				leftClimber.selectMotionParameters(highGearDownMotionParameters);
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void periodic() {
		SmartDashboard.putNumber("Elevator Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Elevator Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Elevator Position Error", this.getPositionError());
		SmartDashboard.putNumber("Elevator Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Elevator Current", this.getCurrentDraw());
		SmartDashboard.putNumber("Elevator Voltage", this.leftClimber.getMotorOutputVoltage());
		
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.leftClimber.getSelectedSensorVelocity();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(int targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}

   
}

    

