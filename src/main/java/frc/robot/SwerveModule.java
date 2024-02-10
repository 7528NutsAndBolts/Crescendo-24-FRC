package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* driver motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0); //was 0
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0); //was 0

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0); //was 0

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
     

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        // lastAngle = getState().angle;

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
            // double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    // private void setAngle(SwerveModuleState desiredState){
    //     Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
    //     mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
    //     lastAngle = angle;
    // }

    // private Rotation2d getAngle(){
    //     // (mAngleMotor.getSelectedSensorPosition());
    //     return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));

    // }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    // private void configAngleEncoder(){        
    //     angleEncoder.configFactoryDefault();
    //     angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // }

    // private void configAngleMotor(){
    //     mAngleMotor.configFactoryDefault();
    //     mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    //     mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    //     mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    //     resetToAbsolute();
    // }

    // private void configDriveMotor(){        
    //     mDriveMotor.configFactoryDefault();
    //     mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    //     mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    //     mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    //     mDriveMotor.setSelectedSensorPosition(0);
    // }



    // public SwerveModuleState getState(){
    //     return new SwerveModuleState(
    //         Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
    //         Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
            
    //     ); 
    // }

    // public SwerveModulePosition getPosition(){
    //     return new SwerveModulePosition(
    //         Conversions.falconToMeters(mDriveMotor.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
    //         Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
    //     );

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }
}