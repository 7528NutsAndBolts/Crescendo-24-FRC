package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private int homePosition = 0;
	private int maxUpTravelPosition = 83000;
	private int intakeConePosition = 0; //6094
	private int intakeCubePosition = 8052;

	private int readyPosition = 0; //27000

	private int midConePosition = 56850; //80000
	private int valConePosition = 63076; //143000, 66422

	private int midCubePosition = 55046; //85000
	private int highCubePosition = 0;   // 155000 w/ wrist tip, 165000, 72850 same for cone, 66422

	private int doubleHPConePosition = 79734; //79000
	private int doubleHPCubePosition = 71772;
	
	private int singleHPConePosition = 0; //44000
	private int singleHPCubePosition = 8052; //38000

	public final static int Elevator_UP = 0;
	public final static int Elevator_DOWN = 1;

	public int upPositionLimit = maxUpTravelPosition;
	public int downPositionLimit = -1;
	private int targetPosition = 0;
	private double arbitraryFeedForward = 0.0;

	private final static int onTargetThreshold = 2000;
	
	private final FXGains upGains = new FXGains(Elevator_UP, 0.08, 0, 0.5, 0.01, 2000);
	private final FXGains downGains = new FXGains(Elevator_DOWN, 0.04, 0, 0.1, 0.011, 2000); // was 0.011

	//Uses PID values to go to a position
	private MotionParameters highGearUpMotionParameters = new MotionParameters(20000, 13861, upGains);//41772
	private MotionParameters highGearDownMotionParameters = new MotionParameters(20000, 13861, downGains);//a = 20000
	
	private double peakOutputReverse = -0.1;
	
	private CustomTalonFX elevatorESC = new CustomTalonFX(9);
	private CustomTalonFX elevatorMotor = new CustomTalonFX(10);

	public Elevator() {
		elevatorESC.configFactoryDefault();
		elevatorESC.clearStickyFaults();
		elevatorMotor.configFactoryDefault();
		elevatorMotor.clearStickyFaults();

		this.elevatorESC.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	
		this.elevatorESC.configForwardSoftLimitEnable(true);
		this.elevatorESC.configForwardSoftLimitThreshold(upPositionLimit);
		this.elevatorMotor.configForwardSoftLimitEnable(true);
		this.elevatorMotor.configForwardSoftLimitThreshold(upPositionLimit);

		this.elevatorESC.configReverseSoftLimitEnable(true);
		this.elevatorESC.configReverseSoftLimitThreshold(downPositionLimit);
		this.elevatorMotor.configReverseSoftLimitEnable(true);
		this.elevatorMotor.configReverseSoftLimitThreshold(downPositionLimit);

		this.elevatorESC.setInverted(false);
		this.elevatorESC.setSensorPhase(false);
		this.elevatorMotor.setInverted(true);
		this.elevatorMotor.setSensorPhase(false);

		this.elevatorESC.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
		this.elevatorESC.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 150);
		this.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
		this.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 150);

		this.elevatorESC.configMotionParameters(highGearUpMotionParameters);
        this.elevatorESC.configMotionParameters(highGearDownMotionParameters);
		this.elevatorMotor.configMotionParameters(highGearUpMotionParameters);
        this.elevatorMotor.configMotionParameters(highGearDownMotionParameters);

		this.elevatorESC.setNeutralMode(NeutralMode.Brake);
		this.elevatorESC.configClosedloopRamp(0.25);
		this.elevatorMotor.setNeutralMode(NeutralMode.Brake);
		this.elevatorMotor.configClosedloopRamp(0.25);

		this.elevatorESC.configVoltageCompSaturation(12);
		this.elevatorESC.enableVoltageCompensation(true);
		this.elevatorMotor.configVoltageCompSaturation(12);
		this.elevatorMotor.enableVoltageCompensation(true);

		this.elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 65, 70, 0.2));
		this.elevatorESC.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 65, 70, 0.2));

		this.elevatorESC.configPeakOutputReverse(peakOutputReverse);
		this.elevatorMotor.configPeakOutputReverse(peakOutputReverse);

		this.resetElevatorEncoder();
	}

	//sets control mode to motion magic
	public void setElevator(ControlMode controlMode, double signal) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		elevatorESC.set(controlMode, signal);
		elevatorMotor.set(controlMode, signal);
	}

	public void setElevator(ControlMode controlMode, double signal, DemandType demandType, double demand) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		elevatorESC.set(controlMode, signal, demandType, demand);
		elevatorMotor.set(controlMode, signal, demandType, demand);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
		this.elevatorESC.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
		this.elevatorMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

	public double getCurrentPosition() {
		return this.elevatorESC.getSelectedSensorPosition();
	}

	public double getCurrentDraw() {
		return this.elevatorESC.getSupplyCurrent();
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

	public int getReadyPosition(){
		return this.readyPosition;
	}

	public int getMidConePosition(){
		return this.midConePosition;
	}

	public int getValConePosition() {
		return this.valConePosition;
	}

	public int getMidCubePosition(){
		return this.midCubePosition;
	}

	public int getHighCubePosition() {
		return this.highCubePosition;
	}

	public int getDoubleHPConePosition(){
		return this.doubleHPConePosition;
	}
	public int getDoubleHPCubePosition(){
		return this.doubleHPCubePosition;
	}

	public int getSingleHPConePosition() {
		return this.singleHPConePosition;
	}

	public int getSingleHPCubePosition() {
		return this.singleHPCubePosition;
	}

	public int getIntakeConePosition() {
		return this.intakeConePosition;
	}

	public int getIntakeCubePosition() {
		return this.intakeCubePosition;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	public void resetElevatorEncoder() {
        try {
			elevatorESC.setSelectedSensorPosition(0, 0, 30);
        }
        catch (Exception e) {
            DriverStation.reportError("Elevator.resetElevatorEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoyStickElevator(){
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
				elevatorESC.selectMotionParameters(highGearUpMotionParameters);
		}
		else {
				elevatorESC.selectMotionParameters(highGearDownMotionParameters);
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
		SmartDashboard.putNumber("Elevator Voltage", this.elevatorESC.getMotorOutputVoltage());
		
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.elevatorESC.getSelectedSensorVelocity();
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