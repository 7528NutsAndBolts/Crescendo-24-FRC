package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.models.FXGains;

public class Wrist extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private int homePosition = 0; //0
	private int maxUpTravelPosition = 0;

	private int floorCubePosition = -38819; //-62000, 29138
	private int floorConePosition = -32100; //55000 40195, 30138, 35138, 32138

	private int singleHPPosition = -38819;  // was -23000 and -33000, 40978

	private int doubleHPConePosition = -34582; //-27900
	private int doubleHPCubePosition = -41717;
	private int centralPosition = -10872;

	private int midConePosition = -50190;
	private int midCubePosition = -49096;

	private int highCubePosition = 0; //29168
	private int valConePosition = -43112;

	public final static int Wrist_UP = 0;
	public final static int Wrist_DOWN = 1;

	public int upPositionLimit = maxUpTravelPosition;
	public int downPositionLimit = -70000;
	private int targetPosition = 0;
	private double arbitraryFeedForward = 0.0;

	private final static int onTargetThreshold = 2000;
	
	private final FXGains upGains = new FXGains(Wrist_UP, 0.04, 0, 0, 0.011, 100);
	private final FXGains downGains = new FXGains(Wrist_DOWN, 0.08, 0, 0, 0.011, 100); 

	//Uses PID values to go to a position
	private MotionParameters highGearUpMotionParameters = new MotionParameters(20000, 20000, upGains); //24000
	private MotionParameters highGearDownMotionParameters = new MotionParameters(20000, 20000, downGains);
	
	private double peakOutputReverse = -0.45;
	
	private CustomTalonFX wristESC = new CustomTalonFX(12);

	public Wrist() {
		wristESC.configFactoryDefault();
		wristESC.clearStickyFaults();

		this.wristESC.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	
		this.wristESC.configForwardSoftLimitEnable(true);
		this.wristESC.configForwardSoftLimitThreshold(upPositionLimit);

		this.wristESC.configReverseSoftLimitEnable(true);
		this.wristESC.configReverseSoftLimitThreshold(downPositionLimit);

		this.wristESC.setInverted(false);
		this.wristESC.setSensorPhase(false);

		this.wristESC.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 150);
		this.wristESC.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 150);

		this.wristESC.configMotionParameters(highGearUpMotionParameters);
        this.wristESC.configMotionParameters(highGearDownMotionParameters);

		this.wristESC.setNeutralMode(NeutralMode.Brake);
		this.wristESC.configClosedloopRamp(0.25);

		this.wristESC.configVoltageCompSaturation(10.0);
		this.wristESC.enableVoltageCompensation(true);

		this.wristESC.configPeakOutputReverse(peakOutputReverse);

		

		this.resetWristEncoder();
	}

	//sets control mode to motion magic
	public void setWrist(ControlMode controlMode, double signal) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		wristESC.set(controlMode, signal);
	}

	public void setWrist(ControlMode controlMode, double signal, DemandType demandType, double demand) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		wristESC.set(controlMode, signal, demandType, demand);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
		this.wristESC.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
	}

	public double getCurrentPosition() {
		return this.wristESC.getSelectedSensorPosition();
	}

	public double getCurrentDraw() {
		return this.wristESC.getSupplyCurrent();
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

	public int getFloorConePosition() {
		return this.floorConePosition;
	}

	public int getFloorCubePosition() {
		return this.floorCubePosition;
	}

	public int getSingleHPPosition() {
		return this.singleHPPosition;
	}

	public int getDoubleHPConePosition() {
		return this.doubleHPConePosition;
	}

	public int getDoubleHPCubePosition() {
		return this.doubleHPCubePosition;
	}

	public int getValConePosition() {
		return this.valConePosition;
	}

	public int getHighCubePosition() {
		return this.highCubePosition;
	}

	public int getMidConePosition() {
		return this.midConePosition;
	}

	public int getMidCubePosition() {
		return this.midCubePosition;
	}

	public int getCentralPosition() {
		return this.centralPosition;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	public void Up() {
		wristESC.set(ControlMode.PercentOutput,0.2);
		
	  }
	
	  public void Down() {
		wristESC.set(ControlMode.PercentOutput,-0.2);
	  }
	



	public void resetWristEncoder() {
        try {
			wristESC.setSelectedSensorPosition(0, 0, 30);
        }
        catch (Exception e) {
            DriverStation.reportError("Wrist.resetWristEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoyStickWrist(){
		double value = 0;
		value = Robot.m_robotContainer.getOperatorRightStickY();
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
				wristESC.selectMotionParameters(highGearUpMotionParameters);
		}
		else {
				wristESC.selectMotionParameters(highGearDownMotionParameters);
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void periodic() {
		SmartDashboard.putNumber("Wrist Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Wrist Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Wrist Position Error", this.getPositionError());
		SmartDashboard.putNumber("Wrist Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Wrist Current", this.getCurrentDraw());
		SmartDashboard.putNumber("Wrist Voltage", this.wristESC.getMotorOutputVoltage());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.wristESC.getSelectedSensorVelocity();
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