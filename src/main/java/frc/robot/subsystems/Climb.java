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

    private CustomTalonFX Climber = new CustomTalonFX(9);

    public Climb() {
        Climber.configFactoryDefault();
        Climber.clearStickyFaults();

        this.Climber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        this.Climber.configForwardSoftLimitEnable(true);
        this.Climber.configForwardSoftLimitThreshold(upPositionLimit);

        this.Climber.configReverseSoftLimitEnable(true);
        this.Climber.configReverseSoftLimitThreshold(downPositionLimit);

        this.Climber.setInverted(false);
        this.Climber.setSensorPhase(false);

        this.Climber.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
        this.Climber.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 150);

        this.Climber.configMotionParameters(highGearUpMotionParameters);
        this.Climber.configMotionParameters(highGearDownMotionParameters);

        this.Climber.setNeutralMode(NeutralMode.Brake);
        this.Climber.configClosedloopRamp(0.25);

        this.Climber.configVoltageCompSaturation(12);
        this.Climber.enableVoltageCompensation(true);

        this.Climber.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 65, 70, 0.2));
        
        this.Climber.configPeakOutputReverse(peakOutputReverse);

        this.resetClimbEncoder();
    }

    public void setClimb(ControlMode controlMode, double signal) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		Climber.set(controlMode, signal);
	}

	public void setClimb(ControlMode controlMode, double signal, DemandType demandType, double demand) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		Climber.set(controlMode, signal, demandType, demand);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
		this.Climber.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

	public double getCurrentPosition() {
		return this.Climber.getSelectedSensorPosition();
	}

	public double getCurrentDraw() {
		return this.Climber.getSupplyCurrent();
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
			Climber.setSelectedSensorPosition(0, 0, 30);
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
				Climber.selectMotionParameters(highGearUpMotionParameters);
		}
		else {
				Climber.selectMotionParameters(highGearDownMotionParameters);
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
		SmartDashboard.putNumber("Elevator Voltage", this.Climber.getMotorOutputVoltage());
		
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.Climber.getSelectedSensorVelocity();
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

    

